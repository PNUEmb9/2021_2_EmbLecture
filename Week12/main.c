#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"


void RCC_Configure(void);
void GPIO_Configure(void);
void DMA_Configure(void);
void ADC_Configure(void);

void ADC1_2_IRQHandler(void);

void Delay(void);

int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};
uint32_t lumiThreshold = 500;

volatile uint32_t ADCValue[1];
uint32_t prevADCValue = 0;
uint32_t colorFlag = WHITE;

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void) // stm32f10x_rcc.h 참고
{
    // DMA1 clock enable
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // ADC1 clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// CDS cell pin clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void GPIO_Configure(void) // stm32f10x_gpio.h 참고
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // CDS cell GPIO configure
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void DMA_Configure(void) {
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = &(ADC1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADCValue[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1,ENABLE);
    ADC_DMACmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) {}
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)) {}

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

int main(void)
{   
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    DMA_Configure();
    ADC_Configure();
    //---------------

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    uint16_t rawTouchX = 0;
    uint16_t rawTouchY = 0;
    uint16_t touchX = 0;
    uint16_t touchY = 0;
    while (1) {
        // Clear LCD when ADC value crossing threshold.
        if(ADCValue[0] < lumiThreshold) {
            if(prevADCValue > lumiThreshold) {
                LCD_Clear(GRAY);
                colorFlag = GRAY;
            }
        } else {
            if(prevADCValue < lumiThreshold) {
                LCD_Clear(WHITE);
                colorFlag = WHITE;
            }
        }
        prevADCValue = ADCValue[0];
        LCD_ShowString(40, 40, "THU_TEAM09", BLACK, colorFlag);
        LCD_ShowNum(40, 100, ADCValue[0], 4, BLACK, colorFlag);
    }
    return 0;
}
