#define DEBUG
/*
 * driver for the i2c-tiny-usb adapter - 1.0
 * http://www.harbaum.org/till/spi_tiny_usb
 *
 * Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

/* include interfaces to usb layer */
#include <linux/usb.h>

/* include interface to i2c layer */
#include <linux/i2c.h>
#include <linux/spi/spi.h>

/* commands via USB, must match command ids in the firmware */
#define CMD_ECHO		0
#define CMD_GET_FUNC		1
#define CMD_SET_DELAY		2
#define CMD_GET_STATUS		3

#define CMD_I2C_IO		4
#define CMD_I2C_IO_BEGIN	(1<<0)
#define CMD_I2C_IO_END		(1<<1)

/* i2c bit delay, default is 10us -> 100kHz max
   (in practice, due to additional delays in the i2c bitbanging
   code this results in a i2c clock of about 50kHz) */
static unsigned short delay = 10;
module_param(delay, ushort, 0);
MODULE_PARM_DESC(delay, "bit delay in microseconds "
		 "(default is 10us for 100kHz max)");

static int usb_read(struct i2c_adapter *adapter, int cmd,
		    int value, int index, void *data, int len);

static int usb_write(struct i2c_adapter *adapter, int cmd,
		     int value, int index, void *data, int len);

/* ----- begin of i2c layer ---------------------------------------------- */

#define STATUS_IDLE		0
#define STATUS_ADDRESS_ACK	1
#define STATUS_ADDRESS_NAK	2

static int usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	unsigned char *pstatus;
	struct i2c_msg *pmsg;
	int i, ret;

	dev_dbg(&adapter->dev, "master xfer %d messages:\n", num);

	pstatus = kmalloc(sizeof(*pstatus), GFP_KERNEL);
	if (!pstatus)
		return -ENOMEM;

	for (i = 0 ; i < num ; i++) {
		int cmd = CMD_I2C_IO;

		if (i == 0)
			cmd |= CMD_I2C_IO_BEGIN;

		if (i == num-1)
			cmd |= CMD_I2C_IO_END;

		pmsg = &msgs[i];

		dev_dbg(&adapter->dev,
			"  %d: %s (flags %d) %d bytes to 0x%02x\n",
			i, pmsg->flags & I2C_M_RD ? "read" : "write",
			pmsg->flags, pmsg->len, pmsg->addr);

		/* and directly send the message */
		if (pmsg->flags & I2C_M_RD) {
			/* read data */
			if (usb_read(adapter, cmd,
				     pmsg->flags, pmsg->addr,
				     pmsg->buf, pmsg->len) != pmsg->len) {
				dev_err(&adapter->dev,
					"failure reading data\n");
				ret = -EREMOTEIO;
				goto out;
			}
		} else {
			/* write data */
			if (usb_write(adapter, cmd,
				      pmsg->flags, pmsg->addr,
				      pmsg->buf, pmsg->len) != pmsg->len) {
				dev_err(&adapter->dev,
					"failure writing data\n");
				ret = -EREMOTEIO;
				goto out;
			}
		}

		/* read status */
		if (usb_read(adapter, CMD_GET_STATUS, 0, 0, pstatus, 1) != 1) {
			dev_err(&adapter->dev, "failure reading status\n");
			ret = -EREMOTEIO;
			goto out;
		}

		dev_dbg(&adapter->dev, "  status = %d\n", *pstatus);
		if (*pstatus == STATUS_ADDRESS_NAK) {
			ret = -EREMOTEIO;
			goto out;
		}
	}

	ret = i;
out:
	kfree(pstatus);
	return ret;
}

static u32 usb_func(struct i2c_adapter *adapter)
{
	__le32 *pfunc;
	u32 ret;

	pfunc = kmalloc(sizeof(*pfunc), GFP_KERNEL);

	/* get functionality from adapter */
	if (!pfunc || usb_read(adapter, CMD_GET_FUNC, 0, 0, pfunc,
			       sizeof(*pfunc)) != sizeof(*pfunc)) {
		dev_err(&adapter->dev, "failure reading functionality\n");
		ret = 0;
		goto out;
	}

	ret = le32_to_cpup(pfunc);
out:
	kfree(pfunc);
	return ret;
}

/* ----- end of i2c layer ------------------------------------------------ */

/* ----- begin of usb layer ---------------------------------------------- */

/*
 * Initially the usb i2c interface uses a vid/pid pair donated by
 * Future Technology Devices International Ltd., later a pair was
 * bought from EZPrototypes
 */
static const struct usb_device_id spi_tiny_usb_table[] = {
	{ USB_DEVICE(0x16c1, 0x06db) },   /* spi-tiny-usb */
	{ }                               /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, spi_tiny_usb_table);

/* Structure to hold all of our device specific stuff */
struct spi_tiny_usb {
	struct usb_device *usb_dev; /* the usb device for this device */
	struct usb_interface *interface; /* the interface for this device */
	struct i2c_adapter adapter; /* i2c related things */
	struct spi_master *master; /* i2c related things */
	struct spi_device *spidev;
	struct spi_board_info	info;
};

static int usb_read(struct i2c_adapter *adapter, int cmd,
		    int value, int index, void *data, int len)
{
	struct spi_tiny_usb *dev = (struct spi_tiny_usb *)adapter->algo_data;

	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_rcvctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE |
			       USB_DIR_IN, value, index, data, len, 2000);
}

static int usb_write(struct i2c_adapter *adapter, int cmd,
		     int value, int index, void *data, int len)
{
	struct spi_tiny_usb *dev = (struct spi_tiny_usb *)adapter->algo_data;

	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_sndctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, data, len, 2000);
}

static void i2c_tiny_usb_free(struct spi_tiny_usb *dev)
{
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}


static int spi_tiny_usb_setup(struct spi_device *spi)
{
	unsigned int i;
	unsigned long flags;


	dev_dbg(&spi->dev, "spi_tiny_usb_setup\n");
	// spin_lock_irqsave(&ebu_lock, flags);

	// if (spi->max_speed_hz >= CLOCK_100M) {
		// /* set EBU clock to 100 MHz */
		// ltq_sys1_w32_mask(0, EBUCC_EBUDIV_SELF100, EBUCC);
		// i = 1; /* divider */
	// } else {
		// /* set EBU clock to 50 MHz */
		// ltq_sys1_w32_mask(EBUCC_EBUDIV_SELF100, 0, EBUCC);

		// /* search for suitable divider */
		// for (i = 1; i < 7; i++) {
			// if (CLOCK_50M / i <= spi->max_speed_hz)
				// break;
		// }
	// }

	// /* setup period of serial clock */
	// ltq_ebu_w32_mask(SFTIME_SCKF_POS_MASK
		     // | SFTIME_SCKR_POS_MASK
		     // | SFTIME_SCK_PER_MASK,
		     // (i << SFTIME_SCKR_POS_OFFSET)
		     // | (i << (SFTIME_SCK_PER_OFFSET + 1)),
		     // SFTIME);

	// /*
	 // * set some bits of unused_wd, to not trigger HOLD/WP
	 // * signals on non QUAD flashes
	 // */
	// ltq_ebu_w32((SFIO_UNUSED_WD_MASK & (0x8 | 0x4)), SFIO);

	// ltq_ebu_w32(BUSRCON0_AGEN_SERIAL_FLASH | BUSRCON0_PORTW_8_BIT_MUX,
			// BUSRCON0);
	// ltq_ebu_w32(BUSWCON0_AGEN_SERIAL_FLASH, BUSWCON0);
	// /* set address wrap around to maximum for 24-bit addresses */
	// ltq_ebu_w32_mask(SFCON_DEV_SIZE_MASK, SFCON_DEV_SIZE_A23_0, SFCON);

	// spin_unlock_irqrestore(&ebu_lock, flags);

	return 0;
}

static int spi_tiny_usb_prepare_xfer(struct spi_master *master)
{
	dev_dbg(&master->dev, "spi_tiny_usb_prepare_xfer\n");
	return 0;
}

static int spi_tiny_usb_unprepare_xfer(struct spi_master *master)
{
	dev_dbg(&master->dev, "spi_tiny_usb_prepare_xfer\n");
	return 0;
}

static int spi_tiny_usb_xfer_one(struct spi_master *master,
					struct spi_message *m)
{
	struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct spi_transfer *t;
	unsigned long spi_flags;
	unsigned long flags;
	int ret = 0;

	dev_dbg(&master->dev, "spi_tiny_usb_xfer_one\n");
	// priv->sfcmd = 0;
	// m->actual_length = 0;

	// spi_flags = FALCON_SPI_XFER_BEGIN;
	// list_for_each_entry(t, &m->transfers, transfer_list) {
		// if (list_is_last(&t->transfer_list, &m->transfers))
			// spi_flags |= FALCON_SPI_XFER_END;

		// spin_lock_irqsave(&ebu_lock, flags);
		// ret = spi_tiny_usb_xfer(m->spi, t, spi_flags);
		// spin_unlock_irqrestore(&ebu_lock, flags);

		// if (ret)
			// break;

		// m->actual_length += t->len;

		// WARN_ON(t->delay_usecs || t->cs_change);
		// spi_flags = 0;
	// }

	m->status = 0;
	spi_finalize_current_message(master);

	return 0;
}

static int spi_tiny_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct spi_tiny_usb *dev;
	int retval = -ENOMEM;
	int ret;
	u16 version;

	dev_dbg(&interface->dev, "probing usb device\n");

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}

	dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	version = le16_to_cpu(dev->usb_dev->descriptor.bcdDevice);
	dev_info(&interface->dev,
		 "version %x.%02x found at bus %03d address %03d\n",
		 version >> 8, version & 0xff,
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	// /* setup i2c adapter description */
	// dev->adapter.owner = THIS_MODULE;
	// dev->adapter.class = I2C_CLASS_HWMON;
	// dev->adapter.algo = &usb_algorithm;
	// dev->adapter.algo_data = dev;
	// snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 // "i2c-tiny-usb at bus %03d device %03d",
		 // dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	// if (usb_write(&dev->adapter, CMD_SET_DELAY, delay, 0, NULL, 0) != 0) {
		// dev_err(&dev->adapter.dev,
			// "failure setting delay to %dus\n", delay);
		// retval = -EIO;
		// goto error;
	// }

	// dev->adapter.dev.parent = &dev->interface->dev;

	// /* and finally attach to i2c layer */
	// i2c_add_adapter(&dev->adapter);

	/* inform user about successful attachment to i2c layer */

	dev->master = spi_alloc_master(&interface->dev, sizeof(*dev));
	if (!dev->master)
		return -ENOMEM;

	dev_info(&interface->dev, "connected spi-tiny-usb device\n");

	dev->master->mode_bits = SPI_MODE_3;
	dev->master->flags = SPI_MASTER_HALF_DUPLEX;
	dev->master->setup = spi_tiny_usb_setup;
	dev->master->prepare_transfer_hardware = spi_tiny_usb_prepare_xfer;
	dev->master->transfer_one_message = spi_tiny_usb_xfer_one;
	dev->master->unprepare_transfer_hardware = spi_tiny_usb_unprepare_xfer;
	dev->master->dev.of_node = interface->dev.of_node;
	dev->master->num_chipselect = 1;

	ret = devm_spi_register_master(&interface->dev, dev->master);
	if (ret)
		spi_master_put(dev->master);

	dev_dbg(&interface->dev, "reg %d\n", ret);

	strcpy(dev->info.modalias, "spidev");
	dev->info.max_speed_hz = 6 * 1000 * 1000;
	dev->info.chip_select = 0;
	dev->info.mode = SPI_MODE_1;

	dev->info.controller_data = dev;
	dev->spidev = spi_new_device(dev->master, &dev->info);
	dev_dbg(&interface->dev, "reg2 %p\n", dev->spidev);
	dev->info.chip_select = 1;
	dev->spidev = spi_new_device(dev->master, &dev->info);

	dev_dbg(&interface->dev, "reg2 %p\n", dev->spidev);

	return 0;

 error:
	if (dev)
i2c_tiny_usb_free(dev);

	return retval;
}

static void spi_tiny_usb_disconnect(struct usb_interface *interface)
{
	struct spi_tiny_usb *dev = usb_get_intfdata(interface);

	dev_dbg(&interface->dev, "test\n");
	usb_set_intfdata(interface, NULL);
	i2c_tiny_usb_free(dev);

	dev_dbg(&interface->dev, "disconnected\n");
}

static struct usb_driver spi_tiny_usb_driver = {
	.name		= "spi-tiny-usb",
	.probe		= spi_tiny_usb_probe,
	.disconnect	= spi_tiny_usb_disconnect,
	.id_table	= spi_tiny_usb_table,
};

module_usb_driver(spi_tiny_usb_driver);

/* ----- end of usb layer ------------------------------------------------ */

MODULE_AUTHOR("Krystian Duzynski <krystian.duzynski@gmail.com>");
MODULE_DESCRIPTION("spi-tiny-usb driver v1.0");
MODULE_LICENSE("GPL");
