#ifndef __KDUSB_CONFIG_H__
#define __KDUSB_CONFIG_H__

// Descriptors
extern const uint8_t USB_DeviceDescriptor[];
#define USB_DEVICE_DESCRIPTOR_LEN 18

extern const uint8_t USB_ConfigDescriptor[];
#define USB_CONFIG_DESCRIPTOR_LEN (18+7)

// Device info
#define USB_VENDOR 'K', 'D'
#define USB_VENDOR_LEN 2

#define USB_PRODUCT 'U', 'S', 'B', 't', 'e', 's', 'a'
#define USB_PRODUCT_LEN 7

#define USB_SERIAL '1'
#define USB_SERIAL_LEN 1

#define USB_PACKET_SIZE 64
// #define USB_PACKET_SIZE 8


#define USB_IMPLEMENT_PREPAREUSERDATA 0

//////////////////////////////////////////////////////////////////////
////////////////////////////// Endpoints /////////////////////////////
//////////////////////////////////////////////////////////////////////

#define USB_ENABLEINTERRUPT1 1

//////////////////////////////////////////////////////////////////////
////////////////////////////// Hardware //////////////////////////////
//////////////////////////////////////////////////////////////////////

// USB ports
#define USB_PORT GPIOA
#define USB_DMINUS_PIN 11
#define USB_DPLUS_PIN  12

// Defines if pullup is connected to pin (0) or connected directly to VCC (1)
#define USB_PULLUP 1

#define USB_PULLUP_PORT GPIOA
#define USB_PULLUP_PIN  8

#endif
