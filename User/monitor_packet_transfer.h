/*
 * monitor_packet_transfer.h
 *
 *  Created on: Jun 3, 2023
 *      Author: linhz
 */

#ifndef MONITOR_PACKET_TRANSFER_H_
#define MONITOR_PACKET_TRANSFER_H_

#include "usb_desc.h"

#define MONITOR_PACKET_BUFFER_N 8

extern __attribute__ ((aligned(16))) uint8_t EP1_Tx_Pkt_Buf[MONITOR_PACKET_BUFFER_N][ DEF_USBD_HS_PACK_SIZE ];
//extern __attribute__ ((aligned(16))) uint8_t EP2_Rx_Pkt_Buf[MONITOR_PACKET_BUFFER_N][ DEF_USBD_HS_PACK_SIZE ];

extern __attribute__ ((aligned(16))) uint16_t EP1_Tx_Pkt_Len[MONITOR_PACKET_BUFFER_N];
//extern __attribute__ ((aligned(16))) uint16_t EP2_Rx_Pkt_Len[MONITOR_PACKET_BUFFER_N];

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t filled;
} PacketBufferStatus;



extern PacketBufferStatus EP1_Tx_Buf_Status;
extern PacketBufferStatus EP2_Rx_Buf_Status;


#endif /* MONITOR_PACKET_TRANSFER_H_ */
