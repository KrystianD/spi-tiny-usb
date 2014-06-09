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

#include <linux/spi/spi.h>
#include <linux/uio_driver.h>

#include "../config.h"

#define FLAGS_BEGIN 1
#define FLAGS_END   2

static int usb_read(struct spi_master *master, int cmd, int value, int index,
		    void *data, int len);

static int usb_write(struct spi_master *master, int cmd, int value,
		     int index, void *data, int len);

/* ----- begin of usb layer ---------------------------------------------- */

static const struct usb_device_id spi_tiny_usb_table[] = {
	{USB_DEVICE(VID, PID)},
	{}
};

MODULE_DEVICE_TABLE(usb, spi_tiny_usb_table);

/* Structure to hold all of our device specific stuff */
struct spi_tiny_usb {
	struct usb_device *usb_dev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	struct spi_master *master;	/* spi master related things */
	struct spi_device *spidev;
	struct spi_board_info info;
	struct uio_info *uio;
};

static int
usb_read(struct spi_master *master, int cmd, int value, int index,
	 void *data, int len)
{
	struct spi_tiny_usb *dev =
	    (struct spi_tiny_usb *)master->dev.platform_data;

	/* do control transfer */
	return usb_control_msg(dev->usb_dev, usb_rcvctrlpipe(dev->usb_dev, 0),
			       cmd,
			       USB_TYPE_VENDOR | USB_RECIP_INTERFACE |
			       USB_DIR_IN, value, index, data, len, 2000);
}

static int
usb_write(struct spi_master *master, int cmd, int value, int index,
	  void *data, int len)
{
	struct spi_tiny_usb *dev =
	    (struct spi_tiny_usb *)master->dev.platform_data;

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
	dev_dbg(&master->dev, "spi_tiny_usb_unprepare_xfer\n");
	return 0;
}

static int spi_tiny_usb_freqtodiv(int freq)
{
	int div = 48 * 1000 * 1000 / freq;
	int i, divVal = 0;
	for (i = 2; i <= 256; i *= 2, divVal++)
		if (i >= div)
			break;
	return divVal;
}

static int
spi_tiny_usb_xfer_one(struct spi_master *master, struct spi_message *m)
{
	// struct spi_tiny_usb *priv = spi_master_get_devdata(master);
	struct spi_transfer *t;
	int spi_flags;
	int ret = 0;

	dev_dbg(&master->dev, "spi_tiny_usb_xfer_one\n");
	m->actual_length = 0;

	spi_flags = FLAGS_BEGIN;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (list_is_last(&t->transfer_list, &m->transfers))
			spi_flags |= FLAGS_END;

		spi_flags |= spi_tiny_usb_freqtodiv(t->speed_hz) << 2;

		dev_dbg(&master->dev,
			"%p %p %d %d len: %d speed: %d flags: %d\n", t->tx_buf,
			t->rx_buf, t->tx_nbits, t->rx_nbits, t->len,
			t->speed_hz, spi_flags);

		if (t->cs_change)
			spi_flags |= FLAGS_END;

		if (t->tx_buf) {
			ret = usb_write(master, 0, 0, spi_flags,
					(void *)t->tx_buf, t->len);
			if (ret < 0)
				break;
		} else {
			void *txbuf = kmalloc(t->len, GFP_KERNEL);
			memset(txbuf, 0x00, t->len);
			if (!txbuf) {
				ret = -ENOMEM;
				break;
			}
			ret = usb_write(master, 0, 0, spi_flags, txbuf, t->len);
			kfree(txbuf);
			if (ret < 0)
				break;
		}

		if (t->rx_buf)
		{
			ret = usb_read(master, 1, 0, 0, t->rx_buf, t->len);
			if (ret < 0)
				break;
		}

		// spin_lock_irqsave(&ebu_lock, flags);
		// ret = spi_tiny_usb_xfer(m->spi, t, spi_flags);
		// spin_unlock_irqrestore(&ebu_lock, flags);

		m->actual_length += t->len;

		if (t->delay_usecs)
			udelay(t->delay_usecs);
		spi_flags = 0;

		if (t->cs_change)
			spi_flags |= FLAGS_BEGIN;
	}

	m->status = ret;
	spi_finalize_current_message(master);

	return 0;
}

static int spi_tiny_usb_irqcontrol(struct uio_info *info, s32 irq_on)
{
	struct spi_tiny_usb *priv = (struct spi_tiny_usb *)info->priv;
	dev_dbg(&priv->interface->dev, "spi_tiny_usb_irqcontrol\n");
	return 0;
}

static int
spi_tiny_usb_probe(struct usb_interface *interface,
		   const struct usb_device_id *id)
{
	struct spi_tiny_usb *priv;
	int retval = -ENOMEM;
	int ret;
	u16 version;

	dev_dbg(&interface->dev, "probing usb device\n");

	/* allocate memory for our device state and initialize it */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	priv->interface = interface;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, priv);

	version = le16_to_cpu(priv->usb_dev->descriptor.bcdDevice);
	dev_info(&interface->dev,
		 "version %x.%02x found at bus %03d address %03d\n",
		 version >> 8, version & 0xff, priv->usb_dev->bus->busnum,
		 priv->usb_dev->devnum);

	dev_info(&interface->dev, "connected spi-tiny-usb device\n");

	priv->master = spi_alloc_master(&interface->dev, sizeof(*priv));
	if (!priv->master)
		goto error;
	priv->master->mode_bits = SPI_MODE_0;
	priv->master->flags = 0;
	priv->master->setup = spi_tiny_usb_setup;
	priv->master->prepare_transfer_hardware = spi_tiny_usb_prepare_xfer;
	priv->master->transfer_one_message = spi_tiny_usb_xfer_one;
	priv->master->unprepare_transfer_hardware = spi_tiny_usb_unprepare_xfer;
	priv->master->dev.of_node = interface->dev.of_node;
	priv->master->num_chipselect = 1;
	priv->master->max_speed_hz = 48 * 1000 * 1000 / 2;
	priv->master->min_speed_hz = 48 * 1000 * 1000 / 256;
	priv->master->dev.platform_data = priv;

	ret = devm_spi_register_master(&interface->dev, priv->master);
	if (ret)
		goto error2;

	strcpy(priv->info.modalias, "spidev");
	priv->info.max_speed_hz = 48 * 1000 * 1000 / 2;
	priv->info.chip_select = 0;
	priv->info.mode = SPI_MODE_0;

	priv->info.controller_data = priv;
	priv->spidev = spi_new_device(priv->master, &priv->info);
	if (!priv->spidev)
		goto error2;

	// UIO
	priv->uio = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!priv->uio)
		goto error2;
	priv->uio->priv = priv;
	priv->uio->name = "spi-tiny-usb";
	priv->uio->version = "1.0.0";

	priv->uio->mem[0].size = 0;
	priv->uio->port[0].size = 0;

	priv->uio->irq = UIO_IRQ_CUSTOM;
	priv->uio->irq_flags = IRQF_SHARED;
	priv->uio->irqcontrol = spi_tiny_usb_irqcontrol;

	if (uio_register_device(&interface->dev, priv->uio))
		goto error3;

	return 0;

 error3:
	kfree(priv->uio);

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

	uio_unregister_device(dev->uio);

	usb_set_intfdata(interface, NULL);
	spi_tiny_usb_free(dev);

	dev_dbg(&interface->dev, "disconnected\n");
}

static struct usb_driver spi_tiny_usb_driver = {
	.name = "spi-tiny-usb",
	.probe = spi_tiny_usb_probe,
	.disconnect = spi_tiny_usb_disconnect,
	.id_table = spi_tiny_usb_table,
};

module_usb_driver(spi_tiny_usb_driver);

/* ----- end of usb layer ------------------------------------------------ */

MODULE_AUTHOR("Krystian Duzynski <krystian.duzynski@gmail.com>");
MODULE_DESCRIPTION("spi-tiny-usb driver v1.0");
MODULE_LICENSE("GPL");
