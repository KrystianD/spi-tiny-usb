#define DEBUG
/*
 * driver for the spi-tiny-usb adapter - 1.0
 *
 * Copyright (C) 2014 Krystian Duzynski (krystian.duzynski@gmail.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Initially based on i2c-tiny-usb project of Till Harbaum (Till@Harbaum.org)
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

#include "../config.h"

/* commands via USB, must match command ids in the firmware */
#define CMD_ECHO		0
#define CMD_GET_FUNC		1
#define CMD_SET_DELAY		2
#define CMD_GET_STATUS		3

#define CMD_I2C_IO		4
#define CMD_I2C_IO_BEGIN	(1<<0)
#define CMD_I2C_IO_END		(1<<1)

#define FLAGS_BEGIN 0
#define FLAGS_END 0

static int usb_read(struct spi_master *master, int cmd,
		    int value, int index, void *data, int len);

static int usb_write(struct spi_master *master, int cmd,
		     int value, int index, void *data, int len);

/* ----- begin of i2c layer ---------------------------------------------- */

// #define STATUS_IDLE		0
// #define STATUS_ADDRESS_ACK	1
// #define STATUS_ADDRESS_NAK	2

// static int usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
// {
	// unsigned char *pstatus;
	// struct i2c_msg *pmsg;
	// int i, ret;

	// dev_dbg(&adapter->dev, "master xfer %d messages:\n", num);

	// pstatus = kmalloc(sizeof(*pstatus), GFP_KERNEL);
	// if (!pstatus)
		// return -ENOMEM;

	// for (i = 0 ; i < num ; i++) {
		// int cmd = CMD_I2C_IO;

		// if (i == 0)
			// cmd |= CMD_I2C_IO_BEGIN;

		// if (i == num-1)
			// cmd |= CMD_I2C_IO_END;

		// pmsg = &msgs[i];

		// dev_dbg(&adapter->dev,
			// "  %d: %s (flags %d) %d bytes to 0x%02x\n",
			// i, pmsg->flags & I2C_M_RD ? "read" : "write",
			// pmsg->flags, pmsg->len, pmsg->addr);

		// /* and directly send the message */
		// if (pmsg->flags & I2C_M_RD) {
			// /* read data */
			// if (usb_read(adapter, cmd,
				     // pmsg->flags, pmsg->addr,
				     // pmsg->buf, pmsg->len) != pmsg->len) {
				// dev_err(&adapter->dev,
					// "failure reading data\n");
				// ret = -EREMOTEIO;
				// goto out;
			// }
		// } else {
			// /* write data */
			// if (usb_write(adapter, cmd,
				      // pmsg->flags, pmsg->addr,
				      // pmsg->buf, pmsg->len) != pmsg->len) {
				// dev_err(&adapter->dev,
					// "failure writing data\n");
				// ret = -EREMOTEIO;
				// goto out;
			// }
		// }

		// /* read status */
		// if (usb_read(adapter, CMD_GET_STATUS, 0, 0, pstatus, 1) != 1) {
			// dev_err(&adapter->dev, "failure reading status\n");
			// ret = -EREMOTEIO;
			// goto out;
		// }

		// dev_dbg(&adapter->dev, "  status = %d\n", *pstatus);
		// if (*pstatus == STATUS_ADDRESS_NAK) {
			// ret = -EREMOTEIO;
			// goto out;
		// }
	// }

	// ret = i;
// out:
	// kfree(pstatus);
	// return ret;
// }

// static u32 usb_func(struct i2c_adapter *adapter)
// {
	// __le32 *pfunc;
	// u32 ret;

	// pfunc = kmalloc(sizeof(*pfunc), GFP_KERNEL);

	// /* get functionality from adapter */
	// if (!pfunc || usb_read(adapter, CMD_GET_FUNC, 0, 0, pfunc,
			       // sizeof(*pfunc)) != sizeof(*pfunc)) {
		// dev_err(&adapter->dev, "failure reading functionality\n");
		// ret = 0;
		// goto out;
	// }

	// ret = le32_to_cpup(pfunc);
// out:
	// kfree(pfunc);
	// return ret;
// }

/* ----- end of i2c layer ------------------------------------------------ */

/* ----- begin of usb layer ---------------------------------------------- */

/*
 * Initially the usb i2c interface uses a vid/pid pair donated by
 * Future Technology Devices International Ltd., later a pair was
 * bought from EZPrototypes
 */
static const struct usb_device_id spi_tiny_usb_table[] = {
	{ USB_DEVICE(VID, PID) },   /* spi-tiny-usb */
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

static int usb_read(struct spi_master *master, int cmd,
		    int value, int index, void *data, int len)
{
	struct spi_tiny_usb *dev = (struct spi_tiny_usb *)master->dev.platform_data;

	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_rcvctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE |
			       USB_DIR_IN, value, index, data, len, 2000);
}

static int usb_write(struct spi_master *master, int cmd,
		     int value, int index, void *data, int len)
{
	struct spi_tiny_usb *dev = (struct spi_tiny_usb *)master->dev.platform_data;

	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_sndctrlpipe(dev->usb_dev, 0),
			       cmd, USB_TYPE_VENDOR | USB_RECIP_INTERFACE,
			       value, index, data, len, 2000);
}

static void spi_tiny_usb_free(struct spi_tiny_usb *dev)
{
	usb_put_dev(dev->usb_dev);
	kfree(dev);
}

static int spi_tiny_usb_setup(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "spi_tiny_usb_setup\n");
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
	// struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct spi_transfer *t;
	// unsigned long spi_flags;
	// unsigned long flags;
	int ret = 0;

	dev_dbg(&master->dev, "spi_tiny_usb_xfer_one\n");
	// priv->sfcmd = 0;
	m->actual_length = 0;

	// spi_flags = FALCON_SPI_XFER_BEGIN;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		// if (list_is_last(&t->transfer_list, &m->transfers))
		// spi_flags |= FALCON_SPI_XFER_END;

		dev_dbg(&master->dev, "%p %p %d %d len: %d\n", t->tx_buf, t->rx_buf, t->tx_nbits, t->rx_nbits, t->len);

		if (t->tx_buf)
		{
			usb_write(master, 0, 0, 0, (void*)t->tx_buf, t->len);
		}
		else
		{
			void *txbuf = kmalloc(t->len, GFP_KERNEL);
			memset(txbuf, 0xff, t->len);
			if (!txbuf)
				return 1;
			usb_write(master, 0, 0, 0, txbuf, t->len);
			kfree(txbuf);
		}

		if (t->rx_buf)
			usb_read(master, 1, 0, 0, t->rx_buf, t->len);

		// spin_lock_irqsave(&ebu_lock, flags);
		// ret = spi_tiny_usb_xfer(m->spi, t, spi_flags);
		// spin_unlock_irqrestore(&ebu_lock, flags);

		if (ret)
			break;

		m->actual_length += t->len;

		WARN_ON(t->delay_usecs || t->cs_change);
		// spi_flags = 0;
	}

	m->status = ret;
	spi_finalize_current_message(master);

	return 0;
}

static int spi_tiny_usb_probe(struct usb_interface *interface,
		const struct usb_device_id *id)
{
	struct spi_tiny_usb *priv;
	int retval = -ENOMEM;
	int ret;
	u16 version;

	dev_dbg(&interface->dev, "probing usb device\n");

	/* allocate memory for our device state and initialize it */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}

	priv->master = 0;

	priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	priv->interface = interface;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, priv);



	version = le16_to_cpu(priv->usb_dev->descriptor.bcdDevice);
	dev_info(&interface->dev,
			"version %x.%02x found at bus %03d address %03d\n",
			version >> 8, version & 0xff,
			priv->usb_dev->bus->busnum, priv->usb_dev->devnum);

	priv->master = spi_alloc_master(&interface->dev, sizeof(*priv));
	if (!priv->master)
		return -ENOMEM;

	dev_info(&interface->dev, "connected spi-tiny-usb device\n");

	priv->master->mode_bits = SPI_MODE_0;
	priv->master->flags = 0;//SPI_MASTER_HALF_DUPLEX;
	priv->master->setup = spi_tiny_usb_setup;
	priv->master->prepare_transfer_hardware = spi_tiny_usb_prepare_xfer;
	priv->master->transfer_one_message = spi_tiny_usb_xfer_one;
	priv->master->unprepare_transfer_hardware = spi_tiny_usb_unprepare_xfer;
	priv->master->dev.of_node = interface->dev.of_node;
	priv->master->num_chipselect = 1;
	priv->master->dev.platform_data = priv;

	ret = devm_spi_register_master(&interface->dev, priv->master);
	if (ret)
		goto error2;

	dev_dbg(&interface->dev, "reg %d\n", ret);

	strcpy(priv->info.modalias, "spidev");
	priv->info.max_speed_hz = 6 * 1000 * 1000;
	priv->info.chip_select = 0;
	priv->info.mode = SPI_MODE_0;

	priv->info.controller_data = priv;
	priv->spidev = spi_new_device(priv->master, &priv->info);
	if (!priv->spidev)
		goto error2;

	dev_dbg(&interface->dev, "reg2 %p\n", priv->spidev);

	return 0;

error2:
	spi_master_put(priv->master);

error:
	if (priv)
		spi_tiny_usb_free(priv);

	return retval;
}

static void spi_tiny_usb_disconnect(struct usb_interface *interface)
{
	struct spi_tiny_usb *dev = usb_get_intfdata(interface);

	usb_set_intfdata(interface, NULL);
	spi_tiny_usb_free(dev);

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
