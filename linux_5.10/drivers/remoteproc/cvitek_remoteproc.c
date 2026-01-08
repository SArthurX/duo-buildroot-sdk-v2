// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>

#include "remoteproc_internal.h"

#define SEC_SUBSYS_BASE 0x02000000
#define SEC_SYS_BASE (SEC_SUBSYS_BASE + 0x000B0000)
#define SEC_CLK_ADDR ((__u32)0x3003024)

/* IP ID for RPMsg VirtIO kick - use IP_RGN (4) to avoid enum change issues */
#define IP_RPMSG_KICK 4  /* Same as IP_RGN in enum IP_TYPE */

/*
 * ============================================
 * RPMsg Memory Layout Configuration
 * ============================================
 * 
 * These values MUST match RTOS rpmsg_static_config.h and memmap.py:
 * 
 *   0x8FDC0000 +------------------+
 *              | Resource Table   | 4KB (RPMSG_RSC_TABLE_OFFSET)
 *   0x8FDC1000 +------------------+
 *              | VRing0           | 8KB (Linux RX <- RTOS TX)
 *   0x8FDC3000 +------------------+
 *              | VRing1           | 8KB (Linux TX -> RTOS RX)
 *   0x8FDC5000 +------------------+
 *              | Buffer Pool      | Remaining space
 *   0x8FDFFFFF +------------------+
 *
 * Per virtio_rpmsg_bus.c:
 *   - Linux uses vqs[0]=vring0 as rvq (RX/input)  - Linux reads here
 *   - Linux uses vqs[1]=vring1 as svq (TX/output) - Linux writes here
 *   - Linux pre-fills vring0 with empty buffers for RTOS to write into
 */
#define RPMSG_RSC_TABLE_OFFSET  0x0000  /* Resource table at base */
#define RPMSG_RSC_TABLE_SIZE    0x1000  /* 4KB */
#define RPMSG_VRING0_OFFSET     0x1000  /* VRing0: Linux RX (pre-filled), RTOS TX */
#define RPMSG_VRING1_OFFSET     0x3000  /* VRing1: Linux TX, RTOS RX */
#define RPMSG_VRING_SIZE        0x2000  /* 8KB per vring */
#define RPMSG_BUFFER_OFFSET     0x5000  /* Buffer pool starts here */
#define RPMSG_VRING_ALIGN       4096    /* VRing alignment */
#define RPMSG_VRING_NUM         64      /* Number of descriptors */

/*
 * cmdqu_t structure - must match the definition in rtos_cmdqu.h
 * This is the 8-byte command structure shared between Linux and RTOS
 */
struct valid_t {
	unsigned char linux_valid;
	unsigned char rtos_valid;
} __packed;

union resv_t {
	struct valid_t valid;
	unsigned short mstime;
};

struct cmdqu_t {
	unsigned char ip_id;
	unsigned char cmd_id : 7;
	unsigned char block : 1;
	union resv_t resv;
	unsigned int param_ptr;
} __packed __aligned(8);

/* 
 * rtos_cmdqu_send function pointer
 * Resolved at runtime via __symbol_get() to avoid hard dependency
 */
typedef int (*rtos_cmdqu_send_t)(struct cmdqu_t *cmdq);
static rtos_cmdqu_send_t rtos_cmdqu_send_fn = NULL;

/* 
 * request_rtos_irq function pointer for registering RTOS->Linux interrupt handler
 * IP_RPMSG_KICK (id=4, same as IP_RGN) will be registered to handle RPMsg vring notifications
 */
typedef int (*request_rtos_irq_t)(unsigned char ip_id, void *handler, const char *devname, void *dev_id);
static request_rtos_irq_t request_rtos_irq_fn = NULL;

typedef int (*free_rtos_irq_t)(unsigned char ip_id);
static free_rtos_irq_t free_rtos_irq_fn = NULL;

struct cvitek_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

struct cvitek_rproc {
	struct rproc *rproc;
	struct cvitek_rproc_mem mem[4];
	struct reset_control *reset;
	struct clk *clk;
	u32 clk_rate;
	struct regmap *boot_base;
	u32 boot_offset;
	int num_mems;
	struct platform_device *pdev;
};

/*
 * Workqueue for deferred RPMsg vring interrupt handling
 * We cannot call rproc_vq_interrupt() directly from IRQ context because
 * it may trigger device registration which requires sleeping.
 */
struct rpmsg_vring_work {
	struct work_struct work;
	struct rproc *rproc;
	int vqid;
};

static struct workqueue_struct *rpmsg_wq;
static struct rpmsg_vring_work rpmsg_work;

/**
 * cvitek_rproc_kick - Send VirtIO vring notification to remote processor
 * @rproc: remote processor handle
 * @vqid: virtqueue ID to kick
 *
 * Uses the rtos_cmdqu infrastructure to send a mailbox notification
 * to the C906L RTOS. This ensures proper hardware spinlock synchronization
 * when accessing shared mailbox registers in SRAM.
 *
 * The RTOS side handles this with ip_id=4 (IP_RGN/IP_RPMSG_KICK), calling 
 * env_isr(vq_id) to process the VirtIO vring notification.
 */
static void cvitek_rproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = rproc->dev.parent;
	struct cmdqu_t cmdq = {0};
	int ret;
	
	/* Try to get the rtos_cmdqu_send function if not already resolved */
	if (!rtos_cmdqu_send_fn) {
		rtos_cmdqu_send_fn = (rtos_cmdqu_send_t)__symbol_get("rtos_cmdqu_send");
		if (!rtos_cmdqu_send_fn) {
			dev_err_once(dev, "rtos_cmdqu module not loaded, cannot kick\n");
			return;
		}
	}
	
	/*
	 * Build RPMsg VirtIO kick command:
	 * - ip_id = 4 (IP_RGN = IP_RPMSG_KICK): RPMsg kick handler ID
	 * - cmd_id = vqid: VirtQueue ID (0 or 1)
	 * - block = 0: Non-blocking send
	 * - linux_valid will be set by rtos_cmdqu_send()
	 */
	cmdq.ip_id = IP_RPMSG_KICK;
	cmdq.cmd_id = vqid & 0x7F;  /* cmd_id is 7 bits */
	cmdq.block = 0;
	cmdq.param_ptr = 0;

	ret = rtos_cmdqu_send_fn(&cmdq);
	if (ret) {
		dev_err(dev, "Failed to kick vqid=%d, ret=%d\n", vqid, ret);
		return;
	}
	
	pr_info("RPMsg: Linux kicking RTOS vqid=%d\n", vqid);
}

static inline void clrbits_32(void __iomem *reg, u32 clear)
{
	iowrite32((ioread32(reg) & ~clear), reg);
}

static inline void setbits_32(void __iomem *reg, u32 set)
{
	iowrite32(ioread32(reg) | set, reg);
}

static int cvitek_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	void __iomem *clk_reset = ioremap(SEC_CLK_ADDR, 4);

	if (!clk_reset) {
		dev_err(dev, "Failed to remap clock reset register\n");
		return -ENOMEM;
	}

	/* Pull low then high to restart C906L */
	clrbits_32(clk_reset, 1 << 6);
	udelay(10);
	setbits_32(clk_reset, 1 << 6);

	iounmap(clk_reset);

	/* Wait for RTOS to initialize spinlock and mailbox subsystem.
	 * RTOS needs time to:
	 * 1. Boot and reach main_cvirtos()
	 * 2. Initialize cvi_spinlock
	 * 3. Setup mailbox registers
	 */
	msleep(2000);

	dev_info(dev, "Started from 0x%llx\n", rproc->bootaddr);

	return 0;
}

static int cvitek_rproc_stop(struct rproc *rproc)
{
	void __iomem *clk_reset = ioremap(SEC_CLK_ADDR, 4);

	if (clk_reset) {
		/* Pull low to stop C906L */
		clrbits_32(clk_reset, 1 << 6);
		iounmap(clk_reset);
	}

	return 0;
}

static int cvitek_rproc_mem_alloc(struct rproc *rproc,
				  struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	/*
	 * Use ioremap (not ioremap_wc) to map memory as uncached/non-cacheable.
	 * This is critical for shared memory regions like rpmsg buffers where
	 * both Linux and RTOS access the same memory. Using cached or 
	 * write-combining mappings can cause data coherency issues.
	 */
	va = ioremap(mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "Unable to map memory region: %pa+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int cvitek_rproc_mem_release(struct rproc *rproc,
				    struct rproc_mem_entry *mem)
{
	if (mem->va)
		iounmap(mem->va);

	return 0;
}

/**
 * cvitek_rproc_da_to_va - Convert device address to va for the carveout/rpmsg regions
 */
static void *cvitek_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len)
{
    struct rproc_mem_entry *carveout;
    void *ptr = NULL;

    list_for_each_entry(carveout, &rproc->carveouts, node) {
        if (carveout->da == FW_RSC_ADDR_ANY)
            continue;
        
        if (da >= carveout->da && da + len <= carveout->da + carveout->len) {
            size_t offset = da - carveout->da;
            /* va is the kernel virtual address of the carveout */
            ptr = carveout->va + offset;
            break;
        }
    }

    return ptr;
}

static int cvitek_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	struct of_phandle_iterator it;
	int index = 0;

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		phys_addr_t mem_base;
		size_t mem_size;
		char *mem_name = it.node->name;
		bool is_reserved_mem = false;
		int current_idx = index;

		index++;

		/* Try standard reserved memory lookup first */
		rmem = of_reserved_mem_lookup(it.node);
		if (rmem) {
			mem_base = rmem->base;
			mem_size = rmem->size;
			is_reserved_mem = true;
		} else {
			/* Fallback: Try to get physical address from 'reg' property */
			struct resource res;
			if (of_address_to_resource(it.node, 0, &res) == 0) {
				mem_base = res.start;
				mem_size = resource_size(&res);
			} else {
				dev_err(dev, "Failed to acquire memory-region: %s\n",
					it.node->full_name);
				return -EINVAL;
			}
		}

		/* Register all RTOS-related regions as carveouts */
		if (strncmp(it.node->name, "rpmsg", 5) == 0 ||
		    strncmp(it.node->name, "c906l", 5) == 0 || 
		    strncmp(it.node->name, "rtos", 4) == 0) {
			/* Register as carveout for ELF loading */
			mem = rproc_mem_entry_init(
				dev, NULL, (dma_addr_t)mem_base, mem_size,
				mem_base, cvitek_rproc_mem_alloc,
				cvitek_rproc_mem_release, mem_name);
			dev_info(dev, "Registered carveout: %s @ %pa size %zx\n",
				 mem_name, &mem_base, mem_size);
			
			/*
			 * For rpmsg region, register vring and buffer carveouts.
			 * This ensures proper memory sharing between Linux and RTOS.
			 * 
			 * Memory layout (must match RTOS rpmsg_static_config.h):
			 *   +0x0000: Resource table (4KB)
			 *   +0x1000: vring0 (8KB) - Linux TX -> RTOS RX
			 *   +0x3000: vring1 (8KB) - RTOS TX -> Linux RX
			 *   +0x5000: Buffer pool (remaining space)
			 *
			 * CRITICAL: The vdev0buffer carveout must be registered so
			 * that virtio_rpmsg_bus allocates message buffers from shared
			 * memory (not from Linux kernel DMA pools that RTOS cannot access).
			 */
			if (strncmp(it.node->name, "rpmsg", 5) == 0) {
				struct rproc_mem_entry *vring_mem;
				struct rproc_mem_entry *buf_mem;
				phys_addr_t vring0_addr = mem_base + RPMSG_VRING0_OFFSET;
				phys_addr_t vring1_addr = mem_base + RPMSG_VRING1_OFFSET;
				phys_addr_t buf_addr = mem_base + RPMSG_BUFFER_OFFSET;
				size_t buf_size = mem_size - RPMSG_BUFFER_OFFSET;
				
				/* Register vdev0vring0: Linux TX -> RTOS RX */
				vring_mem = rproc_mem_entry_init(
					dev, NULL, (dma_addr_t)vring0_addr, RPMSG_VRING_SIZE,
					vring0_addr, cvitek_rproc_mem_alloc,
					cvitek_rproc_mem_release, "vdev%dvring%d", 0, 0);
				if (vring_mem) {
					rproc_add_carveout(rproc, vring_mem);
					dev_info(dev, "Registered vring: vdev0vring0 @ 0x%pa size 0x%x (Linux TX)\n",
						 &vring0_addr, RPMSG_VRING_SIZE);
				}
				
				/* Register vdev0vring1: RTOS TX -> Linux RX */
				vring_mem = rproc_mem_entry_init(
					dev, NULL, (dma_addr_t)vring1_addr, RPMSG_VRING_SIZE,
					vring1_addr, cvitek_rproc_mem_alloc,
					cvitek_rproc_mem_release, "vdev%dvring%d", 0, 1);
				if (vring_mem) {
					rproc_add_carveout(rproc, vring_mem);
					dev_info(dev, "Registered vring: vdev0vring1 @ 0x%pa size 0x%x (Linux RX)\n",
						 &vring1_addr, RPMSG_VRING_SIZE);
				}
				
				/*
				 * Register vdev0buffer for message buffer allocation.
				 *
				 * For no-map regions: use rproc_mem_entry_init() with
				 * alloc/release callbacks. This will cause remoteproc_virtio
				 * to use dma_declare_coherent_memory() which maps the region
				 * as uncached, avoiding cache coherency issues.
				 *
				 * For CMA (reusable) regions: use rproc_of_resm_mem_entry_init()
				 * which will call of_reserved_mem_device_init_by_idx() to
				 * associate the CMA pool with the vdev device.
				 *
				 * We detect no-map by checking if rmem->ops is NULL (no-map
				 * regions don't get ops assigned).
				 */
				if (!rmem || !rmem->ops) {
					/* no-map mode: use ioremap for uncached access */
					buf_mem = rproc_mem_entry_init(
						dev, NULL, (dma_addr_t)buf_addr, buf_size,
						buf_addr, cvitek_rproc_mem_alloc,
						cvitek_rproc_mem_release, "vdev%dbuffer", 0);
					if (buf_mem) {
						rproc_add_carveout(rproc, buf_mem);
						dev_info(dev, "Registered vdev buffer (no-map): vdev0buffer @ 0x%pa size 0x%zx\n",
							 &buf_addr, buf_size);
					}
				} else {
					/* CMA mode: use reserved memory ops */
					buf_mem = rproc_of_resm_mem_entry_init(
						dev, current_idx,
						buf_size,
						buf_addr,
						"vdev%dbuffer", 0);
					if (buf_mem) {
						rproc_add_carveout(rproc, buf_mem);
						dev_info(dev, "Registered vdev buffer (CMA): vdev0buffer @ 0x%pa size 0x%zx (of_resm_idx=%d)\n",
							 &buf_addr, buf_size, current_idx);
					}
				}
			}
		} else {
			dev_dbg(dev, "Skipping memory region: %s\n", mem_name);
			continue;
		}

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
	}

	/* Load resource table from ELF firmware */
	if (rproc_elf_load_rsc_table(rproc, fw))
		dev_warn(&rproc->dev, "no resource table found for this firmware\n");

	return 0;
}

//remoteproc operations
static const struct rproc_ops cvitek_rproc_ops = {
	.start = cvitek_rproc_start,
	.stop = cvitek_rproc_stop,
	.kick = cvitek_rproc_kick,
	.da_to_va = cvitek_rproc_da_to_va,
	.parse_fw = cvitek_rproc_parse_fw,
	.load = rproc_elf_load_segments,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check = rproc_elf_sanity_check,
	.get_boot_addr = rproc_elf_get_boot_addr,
};

static const struct of_device_id cvitek_rproc_match[] = {
	{ .compatible = "cvitek,cv18xx-c906l-rproc", },
	{},
};
MODULE_DEVICE_TABLE(of, cvitek_rproc_match);

static const char *cvitek_rproc_get_firmware(struct platform_device *pdev)
{
	const char *fw_name;
	int ret;

	ret = of_property_read_string(pdev->dev.of_node, "firmware-name",
				      &fw_name);
	if (ret)
		return ERR_PTR(ret);

	return fw_name;
}

/**
 * rpmsg_vring_work_handler - Deferred work handler for vring processing
 * @work: work structure
 *
 * This runs in process context where sleeping is allowed, so device
 * registration (triggered by name service announcement) can complete.
 */
static void rpmsg_vring_work_handler(struct work_struct *work)
{
	struct rpmsg_vring_work *vwork = container_of(work, struct rpmsg_vring_work, work);
	irqreturn_t ret;

	pr_info("RPMsg: Deferred vring processing vqid=%d rproc=%p\n", 
		vwork->vqid, vwork->rproc);

	if (!vwork->rproc) {
		pr_err("RPMsg: rproc is NULL in work handler!\n");
		return;
	}

	/* Now safe to call - we're in process context */
	ret = rproc_vq_interrupt(vwork->rproc, vwork->vqid);
	pr_info("RPMsg: rproc_vq_interrupt returned %d for vqid=%d\n", ret, vwork->vqid);
}

/**
 * rpmsg_vring_irq_handler - Handle VirtIO vring notifications from RTOS
 * @cmd_id: VirtQueue ID from RTOS
 * @ptr: unused
 * @dev_id: rproc instance
 *
 * Called by rtos_cmdqu when RTOS sends ip_id=4 (IP_RGN/IP_RPMSG_KICK) notification.
 * Since this runs in IRQ context and rproc_vq_interrupt may trigger device
 * registration (which requires sleeping), we defer processing to a workqueue.
 */
static void rpmsg_vring_irq_handler(int cmd_id, unsigned int ptr, void *dev_id)
{
	struct rproc *rproc = (struct rproc *)dev_id;

	pr_info("RPMsg: RTOS kicked vqid=%d ptr=0x%x rproc=%p\n", cmd_id, ptr, rproc);

	if (!rproc) {
		pr_err("RPMsg: rproc is NULL!\n");
		return;
	}

	if (!rpmsg_wq) {
		pr_err("RPMsg: workqueue not initialized!\n");
		return;
	}

	/* Queue work for deferred processing in process context */
	rpmsg_work.rproc = rproc;
	rpmsg_work.vqid = cmd_id;
	queue_work(rpmsg_wq, &rpmsg_work.work);
	
	pr_info("RPMsg: Queued vring work for vqid=%d\n", cmd_id);
}

static int cvitek_rproc_parse_dt(struct platform_device *pdev)
{
	//TODO
	return 0;
}

//init
static int cvitek_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct cvitek_rproc *cproc;
	struct rproc *rproc;
	const char *firmware;
	int ret;

	match = of_match_device(cvitek_rproc_match, dev);
	if (!match) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	firmware = cvitek_rproc_get_firmware(pdev);
	if (IS_ERR(firmware)) {
		/* No firmware in DT: will fail if user tries to start manually */
		firmware = NULL;
		dev_warn(dev, "No firmware specified, manual start will fail\n");
	}

	rproc = rproc_alloc(dev, dev_name(dev), &cvitek_rproc_ops, firmware,
			    sizeof(*cproc));
	if (!rproc)
		return -ENOMEM;

	cproc = rproc->priv;
	rproc->has_iommu = false;
	rproc->auto_boot = false;
	rproc->state = RPROC_OFFLINE;
	cproc->pdev = pdev;

	platform_set_drvdata(pdev, rproc);

	ret = cvitek_rproc_parse_dt(pdev);
	if (ret)
		goto free_rproc;

	/* 
	 * Get rtos_cmdqu functions for mailbox communication
	 * These are resolved at runtime to avoid hard module dependencies
	 */
	rtos_cmdqu_send_fn = (rtos_cmdqu_send_t)__symbol_get("rtos_cmdqu_send");
	request_rtos_irq_fn = (request_rtos_irq_t)__symbol_get("request_rtos_irq");
	free_rtos_irq_fn = (free_rtos_irq_t)__symbol_get("free_rtos_irq");

	if (!rtos_cmdqu_send_fn) {
		dev_warn(dev, "rtos_cmdqu_send not available - Linux->RTOS kick disabled\n");
	}

	if (request_rtos_irq_fn && free_rtos_irq_fn) {
		/* Create workqueue for deferred vring processing */
		rpmsg_wq = alloc_workqueue("rpmsg_vring_wq", WQ_UNBOUND | WQ_HIGHPRI, 1);
		if (!rpmsg_wq) {
			dev_err(dev, "Failed to create RPMsg workqueue\n");
			ret = -ENOMEM;
			goto free_symbol;
		}
		INIT_WORK(&rpmsg_work.work, rpmsg_vring_work_handler);
		dev_info(dev, "RPMsg workqueue created\n");

		/* Register handler for RTOS->Linux RPMsg kicks (ip_id=4, IP_RGN) */
		ret = request_rtos_irq_fn(IP_RPMSG_KICK, rpmsg_vring_irq_handler,
					  "cvitek_rpmsg", rproc);
		if (ret) {
			dev_err(dev, "Failed to register RPMsg IRQ handler: %d\n", ret);
			destroy_workqueue(rpmsg_wq);
			rpmsg_wq = NULL;
			goto free_symbol;
		}
		dev_info(dev, "RPMsg IRQ handler registered (ip_id=%d)\n", IP_RPMSG_KICK);
	} else {
		dev_warn(dev, "request_rtos_irq not available - RTOS->Linux kick disabled\n");
	}

	ret = rproc_add(rproc);
	if (ret)
		goto free_symbol;

	return 0;

free_symbol:
	/* Unregister RPMsg IRQ handler on error */
	if (free_rtos_irq_fn) {
		free_rtos_irq_fn(IP_RPMSG_KICK);
		__symbol_put("free_rtos_irq");
		free_rtos_irq_fn = NULL;
	}

	if (request_rtos_irq_fn) {
		__symbol_put("request_rtos_irq");
		request_rtos_irq_fn = NULL;
	}

	if (rtos_cmdqu_send_fn) {
		__symbol_put("rtos_cmdqu_send");
		rtos_cmdqu_send_fn = NULL;
	}

free_clk:
	if (cproc->clk)
		clk_unprepare(cproc->clk);

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int cvitek_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct cvitek_rproc *cproc = rproc->priv;

	rproc_del(rproc);

	if (cproc->clk)
		clk_disable_unprepare(cproc->clk);

	/* Unregister RPMsg IRQ handler */
	if (free_rtos_irq_fn) {
		free_rtos_irq_fn(IP_RPMSG_KICK);
		__symbol_put("free_rtos_irq");
		free_rtos_irq_fn = NULL;
	}

	/* Destroy workqueue after unregistering IRQ handler */
	if (rpmsg_wq) {
		flush_workqueue(rpmsg_wq);
		destroy_workqueue(rpmsg_wq);
		rpmsg_wq = NULL;
	}

	/* Release rtos_cmdqu symbols */
	if (request_rtos_irq_fn) {
		__symbol_put("request_rtos_irq");
		request_rtos_irq_fn = NULL;
	}

	if (rtos_cmdqu_send_fn) {
		__symbol_put("rtos_cmdqu_send");
		rtos_cmdqu_send_fn = NULL;
	}

	rproc_free(rproc);

	return 0;
}

static struct platform_driver cvitek_rproc_driver = {
	.probe = cvitek_rproc_probe,
	.remove = cvitek_rproc_remove,
	.driver = {
		.name = "cvitek-rproc",
		.of_match_table = of_match_ptr(cvitek_rproc_match),
	},
};
module_platform_driver(cvitek_rproc_driver);

MODULE_SOFTDEP("pre: rtos_cmdqu");
MODULE_DESCRIPTION("Cvitek Remote Processor Control Driver");
MODULE_LICENSE("GPL v2");
