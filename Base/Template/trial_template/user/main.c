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
void NVIC_Configure(void);

void ADC1_2_IRQHandler(void)

void Delay(void);

void sendDataUART1(uint16_t data);
void sendDataUART2(uint16_t data);

uint16_t lumiValue;

int color[12] = {WHITE, CYAN, BLUE,, MAGENTA, LGRAY, GREEN, YELLOW, BROWM, BRRED, GRAY};

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void) // stm32f10x_rcc.h 참고
{
    // ADC! pin clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* USART pin clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
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

int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();
    //---------------

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    uint16_t touchX = 0;
    uint16_t touchY = 0;
    while (1) {
    	Touch_GetXY(*touchX, *touchY, 1);
        LCD_ShowCharString(40, 40, "THU_TEAM09", BLACK, WHITE);
        LCD_ShowCharString(40, 60, touchX, BLACK, WHITE);
        LCD_ShowCharString(40, 70, touchY, BLACK, WHITE);
        LCD_ShowCharString(40, 70, touchY, BLACK, WHITE);
        LCD_ShowNum(40, 90, lumiValue, 10, BLACK, WHITE);
        LCD_DrawCircle(touchX, touchY, 3);
    }
    return 0;
}
