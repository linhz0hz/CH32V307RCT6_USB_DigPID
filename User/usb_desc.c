/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : usb device descriptor,configuration descriptor,
 *                      string descriptors and other descriptors.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "usb_desc.h"

/* Device Descriptor */
const uint8_t  MyDevDescr[ ] =
{
    0x12,       // bLength
    0x01,       // bDescriptorType (Device)
    0x00, 0x02, // bcdUSB 2.00
    0xFF,       // bDeviceClass (Vendor Specific)
    0xFF,       // bDeviceSubClass (Free to choose)
    0xFF,       // bDeviceProtocol (Free to choose)
    DEF_USBD_UEP0_SIZE,   // bMaxPacketSize0 64
    (uint8_t)DEF_USB_VID, (uint8_t)(DEF_USB_VID >> 8),  // idVendor 0x1A86 (Assigned by USB Org)
    (uint8_t)DEF_USB_PID, (uint8_t)(DEF_USB_PID >> 8),  // idProduct 0x5537 (Assigned by Manufacturer)
    DEF_IC_PRG_VER, 0x00, // bcdDevice 0.01
    0x01,       // iManufacturer (String Index)
    0x02,       // iProduct (String Index)
    0x03,       // iSerialNumber (String Index)
    0x01,       // bNumConfigurations 1
};

/* Configuration Descriptor(HS) */
const uint8_t  MyCfgDescr_HS[ ] =
{
    0x09,        // bLength
    0x02,        // bDescriptorType (Configuration)
    0x3C, 0x00,  // wTotalLength 60
    0x01,        // bNumInterfaces 1
    0x01,        // bConfigurationValue
    0x00,        // iConfiguration (String Index)
    0x80,        // bmAttributes
    0x32,        // bMaxPower 100mA

    0x09,        // bLength
    0x04,        // bDescriptorType (Interface)
    0x00,        // bInterfaceNumber 0
    0x00,        // bAlternateSetting
    0x06,        // bNumEndpoints 6
    0xFF,        // bInterfaceClass
    0xFF,        // bInterfaceSubClass
    0xFF,        // bInterfaceProtocol
    0x00,        // iInterface (String Index)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x01,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP1_HS_SIZE, (uint8_t)( DEF_USB_EP1_HS_SIZE >> 8 ), // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x81,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP2_HS_SIZE, (uint8_t)( DEF_USB_EP2_HS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x03,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP3_HS_SIZE, (uint8_t)( DEF_USB_EP3_HS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x84,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP4_HS_SIZE, (uint8_t)( DEF_USB_EP4_HS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x05,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP5_HS_SIZE, (uint8_t)( DEF_USB_EP5_HS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x86,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP6_HS_SIZE, (uint8_t)( DEF_USB_EP6_HS_SIZE >> 8 ), // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)
};

/* Configuration Descriptor */
const uint8_t  MyCfgDescr_FS[ ] =
{
    0x09,        // bLength
    0x02,        // bDescriptorType (Configuration)
    0x3C, 0x00,  // wTotalLength 60
    0x01,        // bNumInterfaces 1
    0x01,        // bConfigurationValue
    0x00,        // iConfiguration (String Index)
    0x80,        // bmAttributes
    0x32,        // bMaxPower 100mA

    0x09,        // bLength
    0x04,        // bDescriptorType (Interface)
    0x00,        // bInterfaceNumber 0
    0x00,        // bAlternateSetting
    0x06,        // bNumEndpoints 6
    0xFF,        // bInterfaceClass
    0xFF,        // bInterfaceSubClass
    0xFF,        // bInterfaceProtocol
    0x00,        // iInterface (String Index)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x01,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP1_FS_SIZE, (uint8_t)( DEF_USB_EP1_FS_SIZE >> 8 ), // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x81,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP2_FS_SIZE, (uint8_t)( DEF_USB_EP2_FS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x03,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP3_FS_SIZE, (uint8_t)( DEF_USB_EP3_FS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x84,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP4_FS_SIZE, (uint8_t)( DEF_USB_EP4_FS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x05,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP5_FS_SIZE, (uint8_t)( DEF_USB_EP5_FS_SIZE >> 8 ),  // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x86,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP6_FS_SIZE, (uint8_t)( DEF_USB_EP6_FS_SIZE >> 8 ), // wMaxPacketSize 8
    0x00,        // bInterval 0 (unit depends on device speed)
};

/* Language Descriptor */
const uint8_t  MyLangDescr[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t  MyManuInfo[] = {
        /* linhz@BEC5 */
        22,      /*!< bLength 10*2+2=22 */
        0x03,      /*!< bDescriptorType */
        'l', 0x00,
        'i', 0x00,
        'n', 0x00,
        'h', 0x00,
        'z', 0x00,
        '@', 0x00,
        'B', 0x00,
        'E', 0x00,
        'C', 0x00,
        '5', 0x00
};
/* Product Information */
const uint8_t  MyProdInfo[] =
{
        /* ADC_MONITOR_A00 */
        32,     /*!< bLength 2*15+2=32 */
        0x03,     /*!< bDescriptorType */
        'A', 0x00,
        'D', 0x00,
        'C', 0x00,
        '_', 0x00,
        'M', 0x00,
        'O', 0x00,
        'N', 0x00,
        'I', 0x00,
        'T', 0x00,
        'O', 0x00,
        'R', 0x00,
        '_', 0x00,
        'A', 0x00,
        '0', 0x00,
        '0', 0x00,
};

/* Serial Number Information */
const uint8_t  MySerNumInfo[] =
{
    /* 0123456789 */
        22,     /*!< bLength 2*10+2=22 */
        0x03,     /*!< bDescriptorType */
        '0', 0x00,
        '1', 0x00,
        '2', 0x00,
        '3', 0x00,
        '4', 0x00,
        '5', 0x00,
        '6', 0x00,
        '7', 0x00,
        '8', 0x00,
        '9', 0x00,
        '0', 0x00,
};

/* Device Qualified Descriptor */
const uint8_t MyQuaDesc[ ] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x40, 0x01, 0x00,
};

/* Device BOS Descriptor */
const uint8_t MyBOSDesc[ ] =
{
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};

/* USB Full-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_FS_OSC_DESC[ sizeof(MyCfgDescr_HS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};

/* USB High-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_HS_OSC_DESC[ sizeof(MyCfgDescr_FS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};
