#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"


/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void USARTS_Init(void);
void NVIC_Configure(void);

void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI2_IRQHandler(void);

void Delay(void);

void sendDataUART1(uint16_t data);
void sendDataUART2(uint16_t data);

uint16_t lumiValue;


//---------------------------------------------------------------------------------------------------

void RCC_Configure(void) // stm32f10x_rcc.h 참고
{
    // ADC! pin clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* USART pin clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);

	/* USART2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);

	/* AFIO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* UART between PC pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	//RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    /* UART between bluetooth pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	//RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
    ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus) {}
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus) {}

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
}


void USARTS_Init(void)
{
	USART_InitTypeDef USART_InitStructure;

	// Enable the USART1, USART2 peripheral
	USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART2, ENABLE);
	
	/* 
       BaudRate: 9600
       WordLength: 8bits
       Parity: None
       StopBits: 1bit
       Hardware Flow Control: None
     */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = (uint16_t) USART_WordLength_8b;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = (uint16_t) USART_Parity_No;
    USART_InitStructure.USART_StopBits = (uint16_t) USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = (uint16_t) USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);
    USART_Init(USART2, &USART_InitStructure);
    

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) { // misc.h 참고

    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    // UART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    //ADC1
    NVIC_EnableIRQ(ADC1_2_IRQn);
    
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        flagPC = 1;
        dataBufferFromPC = USART_ReceiveData(USART1);
        
    	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void USART2_IRQHandler() {
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        flagBT = 1;
        dataBufferFromBT = USART_ReceiveData(USART2);
        
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void ADC1_2_IRQHandler() {
    if(ADC_GetITStatus(ADC1,ADC_IT_EOC)!=RESET) {
        lumiValue = ADC_GetConversionValue(ADC1);
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

void sendDataUART1(uint16_t data) {
	USART_SendData(USART1, data);
}

void sendDataUART2(uint16_t data) {
	USART_SendData(USART2, data);
}

int main(void)
{   
    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    USARTS_Init();

    NVIC_Configure();

    while (1) {
    	if(flagPC == 1) {
            sendDataUART2(dataBufferFromPC);
            flagPC = 0;
        } else if (flagBT == 1) {
            sendDataUART1(dataBufferFromBT);
            flagBT = 0;
        }
    }
    return 0;
}
