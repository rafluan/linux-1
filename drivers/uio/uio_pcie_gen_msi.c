// Copyright (c) 2016 FARO Technologies, Inc.
// Author: Terry Lim, terry.lim@faro.com

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/uio_driver.h>
#include <linux/slab.h>

#define DRIVER_NAME		"uio_pcie_gen_msi"
#define DRIVER_VERSION	"0.1"
#define DRIVER_AUTHOR	"Terry Lim, terry.lim@faro.com"
#define DRIVER_DESC		"User I/O PCIe driver that supports one MSI interrupt"

static irqreturn_t uio_pcie_gen_msi_intr(int __always_unused irq, struct uio_info *dev_info)

{
	// nothing to do here for an msi interrupt
	return IRQ_HANDLED;
}

static int uio_pcie_gen_msi_probe(struct pci_dev *pdev, const struct pci_device_id *id)

{
	struct uio_info *gdev;
	int bar, ret = 0;

	gdev = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if(gdev == NULL) {
		dev_err(&pdev->dev, "%s: kzalloc() failed: %d\n", __func__, ret);
		return -ENOMEM;
	}

	ret = pci_enable_device_mem(pdev);
	if (ret != 0)
	{
		dev_err(&pdev->dev, "%s: pci_enable_device() failed: %d\n", __func__, ret);
	    goto free_mem;
	}

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if(ret != 0)
	{
		dev_err(&pdev->dev, "%s: pci_request_regions() failed: %d\n", __func__, ret);
		goto disable_device;
	}

	pci_set_master(pdev);

	// map BARs
	for(bar = 0; bar < MAX_UIO_MAPS; bar++)
	{
		gdev->mem[bar].addr = pci_resource_start(pdev, bar);
		gdev->mem[bar].size = pci_resource_len(pdev, bar);
		gdev->mem[bar].memtype = UIO_MEM_PHYS;

		pr_info("%s: bar%d size  = 0x%lx\n", __func__, bar, (unsigned long)gdev->mem[bar].size);

		if(gdev->mem[bar].size != 0)
		{
			gdev->mem[bar].internal_addr = pci_iomap(pdev, bar, gdev->mem[bar].size);
		}
	}

	// setup the interrupt
	ret = pci_enable_msi(pdev);

	if(ret != 0)
	{
		dev_err(&pdev->dev, "%s: pci_enable_msi() failed: %d\n", __func__, ret);
	    goto release_regions;
	}

	gdev->irq = pdev->irq;
	gdev->irq_flags = 0;
	gdev->handler = uio_pcie_gen_msi_intr;

	pr_info("%s: irq = 0x%x\n", __func__, pdev->irq);

	// misc
	gdev->name = DRIVER_NAME;
	gdev->version = DRIVER_VERSION;

    // register with uio framework
	ret = uio_register_device(&pdev->dev, gdev);
    if (ret != 0)
    {
    	dev_err(&pdev->dev, "%s: uio_register_device() failed: %d\n", __func__, ret);
	    goto disable_msi;
    }

	pci_set_drvdata(pdev, gdev);

	return 0;	// 0 = take ownership of this device

disable_msi:
	pci_disable_msi(pdev);

release_regions:
	for(bar = 0; bar < MAX_UIO_MAPS; bar++)
	{
		if(gdev->mem[bar].size != 0)
		{
			pci_iounmap(pdev, gdev->mem[bar].internal_addr);
		}
	}
	pci_release_regions(pdev);

disable_device:
	pci_disable_device(pdev);

free_mem:
	kfree(gdev);

	return ret;
}

static void uio_pcie_gen_msi_remove(struct pci_dev *pdev)

{
	struct uio_info *gdev = pci_get_drvdata(pdev);
	int bar;

    uio_unregister_device(gdev);
	pci_disable_msi(pdev);

	for(bar = 0; bar < MAX_UIO_MAPS; bar++)
	{
		if(gdev->mem[bar].size != 0)
		{
			pci_iounmap(pdev, gdev->mem[bar].internal_addr);
		}
	}

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	kfree(gdev);
}

static struct pci_driver uio_pcie_gen_msi_driver = {
	.name = DRIVER_NAME,
	.id_table = NULL,					// dynamically assigned using /sys/.../new_id
	.probe = uio_pcie_gen_msi_probe,
	.remove = uio_pcie_gen_msi_remove,
};

module_driver(uio_pcie_gen_msi_driver, pci_register_driver, pci_unregister_driver);

MODULE_LICENSE("GPL v2");
