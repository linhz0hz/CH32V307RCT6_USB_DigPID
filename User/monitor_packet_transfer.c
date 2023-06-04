#include "monitor_packet_transfer.h"
#include "ch32v30x_usb.h"


PacketBufferStatus EP1_Tx_Buf_Status = {0,0,0};
PacketBufferStatus EP2_Rx_Buf_Status = {0,0,0};

//const uint8_t msg_1_tmp[] = "in1 ";

//Monitor Packet format:
// Byte 0-1:  Packet type
// Byte 2-7:  Lowest 6 bytes of epoch timestamp in ms.
// The range is 2^48 ms, ~9000 year, sufficient for us.
// Byte 8-end:  Data. Depending on packet type can have
// more structure. For now just raw data.

// Packet Type: CD (Channel Description)
// Byte 0-1: "CD"
// Byte 2-7:  Lowest 6 bytes of epoch timestamp in ms.
// Byte 8-10: Reserved
// Byte 11: Number of data categories (Usually 1)
// Following bytes: each 12 bytes describes a data category.
// Sub byte 0-1: Number of channesl in this category.
// Sub byte 2: Reserved
// Sub byte 3: Number of bits
// Sub byte 4-7: Sampling frequency (in Hz) 
// Sub byte 9-12: Reserved


// Packet Type: DT (Data)
// Byte 0-1: "DT"
// Byte 2-7:  Lowest 6 bytes of epoch timestamp in ms.
//            This specifies the time stamp of the first data
// Byte 8-end:  Data. Data from all channel at specific time 
//            is listed out first, then next time stamp.

// This packet 
size_t Endp1_Tx_inpacket_index = 0;

void USBHS_Endp1_Tx_Callback( void ){
    //printf("in1\r\n");
    // A Tx packet has been sent out.
    EP1_Tx_Buf_Status.head = (EP1_Tx_Buf_Status.head+1) % MONITOR_PACKET_BUFFER_N;
    
    EP1_Tx_Buf_Status.filled--;
    
    if (EP1_Tx_Buf_Status.filled == 0) {
        // No remaining packets. Stop sending.
        USBHSD->UEP1_TX_CTRL = (USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
        //USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_ACK;
    } else {
        // There is still remaining data. Keep sending.
        USBHSD->UEP1_TX_DMA = (uint32_t)(uint8_t *)EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.head];
        USBHSD->UEP1_TX_LEN  = EP1_Tx_Pkt_Len[EP1_Tx_Buf_Status.head];
        USBHSD->UEP1_TX_CTRL = (USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
    }
}


void USBHS_Endp1_Tx_InsertData_2x32(uint32_t data1, uint32_t data2 ){

    uint32_t *packet_buf_base = (uint32_t*) EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.tail];
    
    // First timestep, include time stamp
    if (Endp1_Tx_inpacket_index ==0 ){
        EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.tail][0] = 'D';
        EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.tail][1] = 'T';
        // TODO: include time stamps here.
    }

    // Save the new data to the buffer.
    // Be careful with pointer arithmetics. I think this is correct.
    *(packet_buf_base + Endp1_Tx_inpacket_index + 8) = data1;
    *(packet_buf_base + Endp1_Tx_inpacket_index + 12) = data2;
    Endp1_Tx_inpacket_index += 8;
    

    if (Endp1_Tx_inpacket_index > DEF_USBD_HS_PACK_SIZE - 16) {
        //The available space happens to be a multiple of 8, so this is fine.
        
        // This packet is already full. Confirm the total length.
        EP1_Tx_Pkt_Len[EP1_Tx_Buf_Status.tail] = Endp1_Tx_inpacket_index + 8;

        //If no data is in the queue, initiate the first transmission
        if (EP1_Tx_Buf_Status.filled == 0){
            USBHSD->UEP1_TX_DMA = (uint32_t)(uint8_t *)EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.head];
            USBHSD->UEP1_TX_LEN  = EP1_Tx_Pkt_Len[EP1_Tx_Buf_Status.head];
            USBHSD->UEP1_TX_CTRL = (USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
        }

        // Increment packet buffer indices
        EP1_Tx_Buf_Status.filled++;
        EP1_Tx_Buf_Status.tail = (EP1_Tx_Buf_Status.tail+1) % MONITOR_PACKET_BUFFER_N;

        // For a new packet, start fresh
        Endp1_Tx_inpacket_index = 0;
    }
}

/*
void USBHS_Endp2_Rx_callback( void ){
    // A Rx packet has been received. 
    EP1_Rx_Pkt_Len[EP1_Rx_Buf_Status.tail] = USBHSD->RX_LEN;
    //EP1_Rx_Buf_Status.tail++;
    EP1_Rx_Buf_Status.tail = (EP1_Rx_Buf_Status.tail+1) & USBHS_MULTI_BUFFER_MASK;
    EP1_Rx_Buf_Status.filled++;
    
    if(EP1_Rx_Buf_Status.filled == USBHS_MULTI_BUFFER_N){
        // Buffer is full. Stop receiving.
        USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_NAK;
    } else {
        // There is still remaining space. Keep receiving.
        USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_ACK;
    }
    USBHSD->UEP1_RX_DMA = (UINT32)(UINT8 *)EP1_Rx_Pkt_Buf[EP1_Rx_Buf_Status.tail];
}*/
/*
void copy_usb_data(void){
    int i,len;
    if (EP1_Rx_Buf_Status.filled>0) {
        // Move data packet from head of RX buffer to tail of TX buffer
        len = EP1_Rx_Pkt_Len[EP1_Rx_Buf_Status.head];
        EP1_Tx_Pkt_Len[EP1_Tx_Buf_Status.tail] = len;
        for (i=0;i<len;i++){
            EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.tail][i] = EP1_Rx_Pkt_Buf[EP1_Rx_Buf_Status.head][i];
        }
        if (EP1_Tx_Buf_Status.filled == 0){ //If previously empty buffer, initiate transaction.
            USBHSD->UEP1_TX_DMA = (UINT32)(UINT8 *)EP1_Tx_Pkt_Buf[EP1_Tx_Buf_Status.head];
            USBHSD->UEP1_TX_LEN  = EP1_Tx_Pkt_Len[EP1_Tx_Buf_Status.head];
            USBHSD->UEP1_TX_CTRL = (USBHSD->UEP1_TX_CTRL & ~USBHS_EP_T_RES_MASK) | USBHS_EP_T_RES_ACK;
        } 
        EP1_Tx_Buf_Status.filled++;
        EP1_Tx_Buf_Status.tail = (EP1_Tx_Buf_Status.tail+1) & USBHS_MULTI_BUFFER_MASK;
        //debug_uart_tx_head=(debug_uart_tx_head+1)&DEBUG_UART_BUF_MASK;
        //EP1_Tx_Buf_Status.tail++;
        if (EP1_Rx_Buf_Status.filled == USBHS_MULTI_BUFFER_N){
            USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_ACK;
        }
        EP1_Rx_Buf_Status.filled--;
        EP1_Rx_Buf_Status.head = (EP1_Rx_Buf_Status.head+1) & USBHS_MULTI_BUFFER_MASK;
    
    }
    
}*/
