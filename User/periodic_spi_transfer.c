/*
 * periodic_spi_transfer.c
 *
 *  Created on: Jul 16, 2022
 *      Author: linhz
 */

#include "periodic_spi_transfer.h"
#include "ch32v30x_spi.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_tim.h"
//#include "ch32v30x.h"
#include "system_ch32v30x.h"


#include "preprog_waveform.h"
#include "PID_controller_core.h"
#include <stdint.h>
/* Global define */
//#define SPI_BUF_SIZE 128

/* Global Variable */
uint16_t TxData1[SPI_BUF_SIZE];
uint16_t TxData2[SPI_BUF_SIZE];
//int16_t RxData[SPI_BUF_SIZE][2] = {};
//uint16_t TxData1;
//int16_t RxData;

//uint16_t TxData3[SPI_BUF_SIZE] = {FULL_RANGE_128SP_WF};
//uint16_t TxData4[SPI_BUF_SIZE] = {FULL_RANGE_128SP_WF};
int16_t RxData1[SPI_BUF_SIZE] = {0};
int16_t RxData2[SPI_BUF_SIZE] = {0};
//uint8_t RxData3[SPI_BUF_SIZE][3] = {};


#define TIMER_PERIOD 285 
//285 at 144MHz
//275 at 120MHz
//This number needs to be large enough to finish all transactions,
// but also smaller than the full period.

/*********************************************************************
 * @fn      Data_Buf_Clear
 *
 * @brief   Initialize all data buffer to appropriate values.
 *
 * @param   none
 *
 * @return  none
 */
void Data_Buf_Clear( void )
{
    for (size_t i = 0; i < SPI_BUF_SIZE; i++)
    {
        TxData1[i] = 0x7fff; // Midscale for unsigned int
        TxData2[i] = 0x7fff; // Midscale for unsigned int
        RxData1[i] = 0;
        RxData2[i] = 0;
    }
}

/*********************************************************************
 * @fn      DMA1_Tx_Init
 *
 * @brief   Initializes the DMA1 Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init( DMA_Channel_TypeDef* DMA_CHx, uint32_t ppadr, uint32_t memadr, uint16_t bufsize)
{
    DMA_InitTypeDef DMA_InitStructure={0};

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE );

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    // If enabled, each time the data is transferred, automatically fetch next item in memory.
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA_CHx, &DMA_InitStructure );
}

/*********************************************************************
 * @fn      DMA_Rx_Init
 *
 * @brief   Initializes the SPI1 DMA Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Rx_Init( DMA_Channel_TypeDef* DMA_CHx, uint32_t ppadr, uint32_t memadr, uint16_t bufsize )
{
	DMA_InitTypeDef DMA_InitStructure={0};

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE );

	DMA_DeInit(DMA_CHx);

	DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
	DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = bufsize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init( DMA_CHx, &DMA_InitStructure );
}


/*********************************************************************
 * @fn      Setup_Periodic_Update_TIM2
 *
 * @brief   Setup TIM2 to schedule all DMA transfer to SPI1 and 
 *          related control signals with the right timing for the 
 *          DAC (MAX5719/5717 ) and ADC (ADS8883/8863)
 *
 * @param   
 *
 * @return  none
 */
void Setup_Periodic_Update_TIM2(void){

    // Update itself takes 2.33us at 120MHz.
    // The actual update peri
    uint16_t arr = TIMER_PERIOD;

    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    // All channels of TIM2 set to output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    // TIM2 period setup
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0; // No prescalar
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

    // Common setting for all channels of TIM2
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    

    // TIM2 CH1 setup 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_Pulse = 1;
    TIM_OC1Init( TIM2, &TIM_OCInitStructure );
    
    // TIM2 CH2 setup 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_Pulse = 60;
    TIM_OC2Init( TIM2, &TIM_OCInitStructure );
    

    // TIM2 CH3 setup 
    // -- Used as the CS signal on the DAc and CONV signal on ADS8883/8863 ADC
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 142;
    TIM_OC3Init( TIM2, &TIM_OCInitStructure );

    // TIM2 CH4 setup 
    // -- Used as the LDAC signal.
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_Pulse = 145;
    TIM_OC4Init( TIM2, &TIM_OCInitStructure );
    TIM_ForcedOC4Config( TIM2, TIM_ForcedAction_Active);

    // Set up TIM2 to be triggered by the output trigger (TRGO) from TIM4.
    // See the TIM4 setting to find out what TRGO actually is wired to.
    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR3);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Trigger);
    TIM_SelectOnePulseMode( TIM2,TIM_OPMode_Single );

    // enable DMA 
    TIM_DMACmd( TIM2, TIM_DMA_CC1, ENABLE);
    //TIM_DMACmd( TIM2, TIM_DMA_CC2, ENABLE);

}

/*********************************************************************
 * @fn      Setup_Periodic_Update_TIM5
 *
 * @brief   Setup TIM5 to schedule all DMA transfer to SPI2. No trigger
 *          signal is generated here, as they are shared with TIM2.
 *
 * @param   
 *
 * @return  none
 */
void Setup_Periodic_Update_TIM5(void){

    // Update itself takes 2.33us at 120MHz.
    // The actual update peri
    uint16_t  arr = TIMER_PERIOD;

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

    // Note that as TIM2 and TIM5 are synchronized, TIM5 
    // do not need to generate its own CS/LDAC signal.
    // Hence there is no GPIO setting here.

    // TIM5 period setup
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0; // No prescalar
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM5, &TIM_TimeBaseInitStructure);

    // Common setting for all channels of TIM5. Note output is diabled.
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

    // TIM5 CH1 setup 
    
    TIM_OCInitStructure.TIM_Pulse = 3;
    TIM_OC1Init( TIM5, &TIM_OCInitStructure );
    
    // TIM5 CH1 setup 
    
    TIM_OCInitStructure.TIM_Pulse = 27;
    TIM_OC2Init( TIM5, &TIM_OCInitStructure );

    // TIM5 CH1 setup 
    
    TIM_OCInitStructure.TIM_Pulse = 51;
    TIM_OC3Init( TIM5, &TIM_OCInitStructure );
    

    // Set up TIM5 to be triggered by the output trigger (TRGO) from TIM4.
    // See the TIM4 setting to find out what TRGO actually is wired to.
    TIM_SelectInputTrigger(TIM5, TIM_TS_ITR2);
    TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Trigger);
    TIM_SelectOnePulseMode( TIM5,TIM_OPMode_Single );

    // enable DMA 
    TIM_DMACmd( TIM5, TIM_DMA_CC1, ENABLE);
    //TIM_DMACmd( TIM5, TIM_DMA_CC2, ENABLE);
    //TIM_DMACmd( TIM5, TIM_DMA_CC3, ENABLE);
}

/*********************************************************************
 * @fn      Setup_Periodic_Update_TIM10
 *
 * @brief   Setup TIM10 to schedule all DMA transfer to SPI2. No trigger
 *          signal is generated here, as they are shared with TIM2.
 *
 * @param   
 *
 * @return  none
 */
void Setup_Periodic_Update_TIM10(void){

    // Update itself takes 2.33us at 120MHz.
    // The actual update peri
    uint16_t  arr = TIMER_PERIOD;

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM10, ENABLE );

    // Note that as TIM2 and TIM10 are synchronized, TIM10 
    // do not need to generate its own CS/LDAC signal.
    // Hence there is no GPIO setting here.

    // TIM10 period setup
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0; // No prescalar
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM10, &TIM_TimeBaseInitStructure);

    // Common setting for all channels of TIM10. Note output is diabled.
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

    // TIM10 CH1 setup 
    
    TIM_OCInitStructure.TIM_Pulse = 5;
    TIM_OC1Init( TIM10, &TIM_OCInitStructure );
    
    // TIM10 CH2 setup 
    
    TIM_OCInitStructure.TIM_Pulse = 29;
    TIM_OC2Init( TIM10, &TIM_OCInitStructure );

    // TIM10 CH3 setup 
    
    TIM_OCInitStructure.TIM_Pulse = 53;
    TIM_OC3Init( TIM10, &TIM_OCInitStructure );
    

    // Set up TIM10 to be triggered by the output trigger (TRGO) from TIM4.
    // See the TIM4 setting to find out what TRGO actually is wired to.
    TIM_SelectInputTrigger(TIM10, TIM_TS_ITR2);
    TIM_SelectSlaveMode(TIM10, TIM_SlaveMode_Trigger);
    TIM_SelectOnePulseMode( TIM10,TIM_OPMode_Single );

    // enable DMA 
    TIM_DMACmd( TIM10, TIM_DMA_CC1, ENABLE);
    TIM_DMACmd( TIM10, TIM_DMA_CC2, ENABLE);
    TIM_DMACmd( TIM10, TIM_DMA_CC3, ENABLE);
}
/*********************************************************************
 * @fn      Setup_Periodic_Update_SPI1
 *
 * @brief   Configuring SPI1 for full-duplex communication, which will 
 *          be used for talking to DAC/ADC.
 *
 * @return  none
 */
void Setup_Periodic_Update_SPI1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	SPI_InitTypeDef SPI_InitStructure={0};

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );

    //Not using the hard-ware CS pin. TIM2 is used to generate correct CS signal.
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init( GPIOA, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );


	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    //Original Config. The DAC indeed triggers on the rising edge.
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

    // The ADC SPI interface takes the data on the falling edge.
    // It seems that the following configuration is better when using at
    // 36MHz, slightly above the ADS8863 rating at 32MHz
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;


    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;

    // Prescalar = 4, at 120MHz this is 30MHz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure );
    //SPI_SSOutputCmd(SPI1, ENABLE);
    SPI_SSOutputCmd(SPI1, DISABLE );

    // Do not enable TX DMA, as TIM2 are used to time TX transfers.
    // However, RX DMA can still be used as usual.
	//SPI_I2S_DMACmd( SPI1, SPI_I2S_DMAReq_Tx, ENABLE );
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable RX Interrupt as well. Depending on the odd-even 
    // SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Cmd( SPI1, ENABLE );
}


/*********************************************************************
 * @fn      Setup_Periodic_Update_SPI2
 *
 * @brief   Configuring SPI2 for full-duplex communication, which will 
 *          be used for talking to DAC/ADC.
 *
 * @return  none
 */
void Setup_Periodic_Update_SPI2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	SPI_InitTypeDef SPI_InitStructure={0};

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
    //Not using the hard-ware CS pin.

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( GPIOB, &GPIO_InitStructure );


	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    //SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    
    //Original Config. The DAC indeed triggers on the rising edge.
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

    // The ADC SPI interface takes the data on the falling edge.
    // It seems that the following configuration is better when using at
    // 36MHz, slightly above the ADS8863 rating at 32MHz
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    //SPI_SSOutputCmd(SPI2, ENABLE);
    

    // Prescalar = 4, at 120MHz this is 30MHz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI2, &SPI_InitStructure );

    SPI_SSOutputCmd(SPI2, DISABLE );

    // Do not enable TX DMA, as TIM2 are used to time TX transfers.
    // However, RX DMA can still be used as usual.
	//SPI_I2S_DMACmd( SPI2, SPI_I2S_DMAReq_Tx, ENABLE );
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable RX Interrupt as well. Depending on the odd-even 
    // SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Cmd( SPI2, ENABLE );
}


/*********************************************************************
 * @fn      Setup_Periodic_Update_SPI3
 *
 * @brief   Configuring SPI3 for full-duplex communication, which will 
 *          be used for talking to DAC/ADC.
 *
 * @return  none
 */
void Setup_Periodic_Update_SPI3(void)
{
	GPIO_InitTypeDef GPIO_InitStructure={0};
	SPI_InitTypeDef SPI_InitStructure={0};

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI3 , ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE);
    //Not using the hard-ware CS pin.

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( GPIOB, &GPIO_InitStructure );


	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    //Original Config. The DAC indeed triggers on the rising edge.
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

    // The ADC SPI interface takes the data on the falling edge.
    // It seems that the following configuration is better when using at
    // 36MHz, slightly above the ADS8863 rating at 32MHz
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    //SPI_SSOutputCmd(SPI3, ENABLE);
    

    // Prescalar = 4, at 120MHz this is 30MHz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI3, &SPI_InitStructure );

    SPI_SSOutputCmd(SPI3, DISABLE );

    // Do not enable TX DMA, as TIM2 are used to time TX transfers.
    // However, RX DMA can still be used as usual.
	//SPI_I2S_DMACmd( SPI3, SPI_I2S_DMAReq_Tx, ENABLE );
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable RX Interrupt as well. Depending on the odd-even 
    // SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Cmd( SPI3, ENABLE );
}
/*********************************************************************
 * @fn      Setup_Periodic_Trigger
 *
 * @brief   Initializes TIM4 to generate a fixed square wave at a duty
 *          cycle of 50%
 *          It is used to trigger TIM2, which in turn trigger DMA to 
 *          transfer data to SPI1 and eventually result in data 
 *          transfer to DAC/ADC
 *
 *
 * @param   period - period of the signal in 0.25us
 *
 * @return  none
 */
void Setup_Periodic_Trigger(uint16_t period)
{

    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );


    TIM_TimeBaseInitStructure.TIM_Period = period-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock/12000000-1; // One count is 0.0833 (1/12) us
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 11; //1us trigger
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init( TIM4, &TIM_OCInitStructure );

    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init( TIM4, &TIM_OCInitStructure );


    TIM_CtrlPWMOutputs(TIM4, ENABLE );
    TIM_OC3PreloadConfig( TIM4, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig( TIM4, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM4, ENABLE );
    
    TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);

    TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);

    TIM_Cmd( TIM4, ENABLE );
}

/*********************************************************************
 * @fn      Setup_Periodic_Update
 *
 * @brief   Initializes TIM2 and SPI1 to provide periodic update signals.
 *
 * @param   period - period of the signal in 0.25us
 *
 * @return  none
 */
void Setup_Periodic_Update(uint16_t period)
{
    Data_Buf_Clear();
    //Setup basic functions of TIM2 and SPI1
    Setup_Periodic_Update_TIM2();
    Setup_Periodic_Update_SPI1();
    Setup_Periodic_Update_TIM5();
    Setup_Periodic_Update_SPI2();
    //Setup_Periodic_Update_TIM10();
    //Setup_Periodic_Update_SPI3();
    //Setup the DMA channels properly.

    //Relavent DMA for SPI1 transfers:
    //DMA request by SPI1_RX
    DMA_Rx_Init( DMA1_Channel2, (uint32_t)&SPI1->DATAR, (uint32_t)RxData1, SPI_BUF_SIZE );
    DMA_Cmd( DMA1_Channel2, ENABLE );
	//DMA request by TIM2_CH1 compare
	//DMA_Tx_Init( DMA1_Channel5, (uint32_t)&SPI1->DATAR, (uint32_t)TxData1, SPI_BUF_SIZE );
    DMA_Tx_Init( DMA1_Channel5, (uint32_t)&SPI1->DATAR, (uint32_t)TxData1, SPI_BUF_SIZE );
	DMA_Cmd( DMA1_Channel5, ENABLE );
	//DMA request by TIM2_CH2 compare
	//DMA_Tx_Init( DMA1_Channel7, (uint32_t)&SPI1->DATAR, (uint32_t)TxData2, SPI_BUF_SIZE );
	//DMA_Cmd( DMA1_Channel7, ENABLE );

    //Relavent DMA for SPI2 transfers:
    //DMA request by SPI2_RX
    DMA_Rx_Init( DMA1_Channel4, (uint32_t)&SPI2->DATAR, (uint32_t)RxData2, SPI_BUF_SIZE );
    DMA_Cmd( DMA1_Channel4, ENABLE );
    //DMA request by TIM5_CH1 compare
	DMA_Tx_Init( DMA2_Channel5, (uint32_t)&SPI2->DATAR, (uint32_t)TxData2, SPI_BUF_SIZE );
	DMA_Cmd( DMA2_Channel5, ENABLE );
	////DMA request by TIM5_CH2 compare
	//DMA_Tx_Init( DMA2_Channel4, (uint32_t)&SPI2->DATAR, (uint32_t)TxData3, SPI_BUF_SIZE );
	//DMA_Cmd( DMA2_Channel4, ENABLE );
    ////DMA request by TIM5_CH3 compare
	//DMA_Tx_Init( DMA2_Channel2, (uint32_t)&SPI2->DATAR, (uint32_t)TxData3, SPI_BUF_SIZE );
	//DMA_Cmd( DMA2_Channel2, ENABLE );

    /*
    //Relavent DMA for SPI3 transfers:
    //DMA request by SPI3_RX
    DMA_Rx_Init( DMA2_Channel1, (uint32_t)&SPI3->DATAR, (uint32_t)RxData3, SPI_BUF_SIZE*3 );
    DMA_Cmd( DMA1_Channel4, ENABLE );
    //DMA request by TIM10_CH1 compare
	DMA_Tx_Init( DMA2_Channel8, (uint32_t)&SPI3->DATAR, (uint32_t)TxData4, SPI_BUF_SIZE );
	DMA_Cmd( DMA2_Channel8, ENABLE );
	//DMA request by TIM10_CH2 compare
	DMA_Tx_Init( DMA2_Channel9, (uint32_t)&SPI3->DATAR, (uint32_t)TxData4, SPI_BUF_SIZE );
	DMA_Cmd( DMA2_Channel9, ENABLE );
    //DMA request by TIM10_CH3 compare
	DMA_Tx_Init( DMA2_Channel10, (uint32_t)&SPI3->DATAR, (uint32_t)TxData4, SPI_BUF_SIZE );
	DMA_Cmd( DMA2_Channel10, ENABLE );
    */
    //Enable  interrupt 
    //NVIC_InitTypeDef NVIC_InitStructure = {0};
    //NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
    NVIC_EnableIRQ(TIM4_IRQn);
    SetVTFIRQ((uint32_t)TIM4_IRQHandler,TIM4_IRQn,0,ENABLE);
    
    //Start the transfer
    Setup_Periodic_Trigger(period);
}




