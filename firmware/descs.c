#include <kdusb.h>
#include "kdusb_config.h"

#include "../config.h"

const uint8_t USB_DeviceDescriptor[] =
{
	18,   /* bLength */
	DEVICE_DESCRIPTOR,     /* bDescriptorType */
	0x00,
	0x02,   /* bcdUSB = 2.00 */
	0xff,   /* bDeviceClass: CDC */
	0xff,   /* bDeviceSubClass */
	0xff,   /* bDeviceProtocol */
	64,   /* bMaxPacketSize0 */
	0xc1, 0x16,   /* idVendor = 0x0483 */
	0xdb, 0x06,   /* idProduct = 0x7540 */
	0x01,
	0x00,   /* bcdDevice = 2.00 */
	1,              /* Index of string descriptor describing manufacturer */
	2,              /* Index of string descriptor describing product */
	3,              /* Index of string descriptor describing the device's serial number */
	0x01    /* bNumConfigurations */
};

const uint8_t USB_ConfigDescriptor[] = {    /* USB configuration descriptor */
	9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
	CONFIG_DESCRIPTOR,    /* descriptor type */
	USB_CONFIG_DESCRIPTOR_LEN, 0, /* total length of data returned (including inlined descriptors) */
	1,          /* number of interfaces in this configuration */
	1,          /* index of this configuration */
	0,          /* configuration name string index */
	(1 << 7),                           /* attributes */
	100/2,            /* max USB current in 2mA units */
	/* interface descriptor follows inline: */
	9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
	INTERFACE_DESCRIPTOR, /* descriptor type */
	0,          /* index of this interface */
	0,          /* alternate setting for this interface */
	1, /* endpoints excl 0: number of endpoint descriptors to follow */
	0,
	0,
	0,
	0,          /* string index for interface */

	7,
	ENDPOINT_DESCRIPTOR,
	0x81,
	0x03,
	64, 0,
	10,
/*
	7,
	ENDPOINT_DESCRIPTOR,
	0x02,
	0x03,
	64, 0,
	10,*/
};
