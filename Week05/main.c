#include "stm32f10x.h"

void delay() {
  int i;
  for(i=0; i<10000000; i++) {}
}

int main(void) {
  // GPIO 설정을 위한 InitTypeDef 변수 선언
  GPIO_InitTypeDef GPIO_InitInput;
  GPIO_InitTypeDef GPIO_InitLEDOutput;
  GPIO_InitTypeDef GPIO_InitRelayOutput;

  // Input(버튼) 설정
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  // Port D의 11,12번 핀에 대하여 설정
  GPIO_InitInput.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
  // Pull-Up 모드로 설정(버튼을 누르면 Low, 가만히 두면 High)
  GPIO_InitInput.GPIO_Mode = GPIO_Mode_IPU;
  
  //Output(릴레이모듈) 설정
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  // Port B의 8번 핀에 대하여 설정
  GPIO_InitRelayOutput.GPIO_Pin = GPIO_Pin_8;
  // Push-Pull 모드로 설정
  GPIO_InitRelayOutput.GPIO_Mode = GPIO_Mode_Out_PP;
  // 저전력 모드인 2MHz로 설정
  GPIO_InitRelayOutput.GPIO_Speed = GPIO_Speed_2MHz;
  
  // OutPut(LED) 설정
  // Port D의 7번 핀에 대하여 설정
  GPIO_InitLEDOutput.GPIO_Pin = GPIO_Pin_7;
  // Push-Pull 모드로 설정
  GPIO_InitLEDOutput.GPIO_Mode = GPIO_Mode_Out_PP;
  // 저전력 모드인 2MHz로 설정
  GPIO_InitLEDOutput.GPIO_Speed = GPIO_Speed_2MHz;

  // 위에서 지정한 설정대로 각 포트를 초기화
  GPIO_Init(GPIOC, &GPIO_InitInput);
  GPIO_Init(GPIOD, &GPIO_InitRelayOutput);
  GPIO_Init(GPIOD, &GPIO_InitLEDOutput);

  // 초기화 이후 아래 동작을 계속 반복
  while (1) {
    uint8_t relayInputPin = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11);
    uint8_t ledInputPin = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12);

    // S1 버튼을 누를 경우(Pull-up이므로 0이 인가될 경우)
    // 릴레이 모듈(PB8)에 delay동안 신호를 줌
    if(relayInputPin == Bit_RESET) {
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      delay();
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
    }
    
    // S2 버튼을 누를 경우
    // LED(PD7)에 delay동안 신호를 줌
    if(ledInputPin == Bit_RESET) {
      GPIO_SetBits(GPIOD,GPIO_Pin_7);
      delay();
      GPIO_ResetBits(GPIOD,GPIO_Pin_7);
    }
  }
  return 0;
}