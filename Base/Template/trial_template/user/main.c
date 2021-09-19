#include "stm32f10x.h"

int main(void) {
  // GPIO 설정을 위한 InitTypeDef 변수 선언
  GPIO_InitTypeDef GPIO_InitInput;
  GPIO_InitTypeDef GPIO_InitOutput;

  // Input(조이스틱) 설정
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  // Port C의 2, 3, 4, 5번 핀에 대하여 설정
  GPIO_InitInput.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  // Pull-Up 모드로 설정(스틱을 꺾으면 Low, 가만히 두면 High)
  GPIO_InitInput.GPIO_Mode = GPIO_Mode_IPU;
  
  // OutPut(LED) 설정
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  // Port D의 2, 3, 4, 7번 핀에 대하여 설정
  GPIO_InitOutput.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_7;
  // Push-Pull 모드로 설정(High가 인가되면 1, Low가 인가되면 0)
  GPIO_InitOutput.GPIO_Mode = GPIO_Mode_Out_PP;
  // 저전력 모드인 2MHz로 설정
  GPIO_InitOutput.GPIO_Speed = GPIO_Speed_2MHz;

  // 위에서 지정한 설정대로 각 포트를 초기화
  GPIO_Init(GPIOC, &GPIO_InitInput);
  GPIO_Init(GPIOD, &GPIO_InitOutput);

  // 초기화 이후 아래 동작을 계속 반복
  while (1) {
    uint16_t ledPin;  // 점등할 LED의 핀 번호를 저장할 변수
    uint16_t inputPin = GPIO_ReadInputData(GPIOC); // 입력된 조이스틱의 핀 값

    // 원하는 핀 번호를 얻기 위해 AND 연산과 XOR 연산 사용
    inputPin &= (GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
    inputPin ^= (GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);

    // 입력된 조이스틱의 방향에 따라 점등할 LED 결정
    switch (inputPin) {
    case GPIO_Pin_2:
      ledPin = GPIO_Pin_3;
      break;
    case GPIO_Pin_3:
      ledPin = GPIO_Pin_4;
      break;
    case GPIO_Pin_4:
      ledPin = GPIO_Pin_7;
      break;
    case GPIO_Pin_5:
      ledPin = GPIO_Pin_2;
      break;
    default:
      // 조이스틱의 입력이 없을 경우 점등하지 않음
      ledPin = (uint16_t)0x0000;
      break;
    }

    // 위에서 결정된 LED만 점등
    GPIO_Write(GPIOD, ledPin);
  }
  return 0;
}