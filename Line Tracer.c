#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
/*코드 구현 목적
/ 스탭모터 타이머 3000hz 정도
/ 적외선 120hz 정도
/ 적외선 숫자에따라 flag값이 변하고, 
/ 그에따라 인자에 다른 숫자를 넘겨
/ 타이머펄스변경
*/

#define TIMER_PRESCALER_FREQ 1000000 
#define TIMER_FREQ_2 1000 //우측
#define TIMER_FREQ_3 1000 //좌측

#define TIMER_FREQ_5 100//120hz

uint16_t adc1RH,adc2RT,adc3LH,adc4LT;
int t2 = 0;
int RightadjustG,RightadjustB, LeftadjustG,LeftadjustB;
int stopflag;


int echostate = 0;
int sonicwavestate = 0;
int count = 999999;

int check;
char cPC;
char cBT;
int pcState = 0;
int btState = 0;


int L=0;
int R= 0; // GPIO High/Low 구분 용 변수
uint16_t x, y, z = 0; //3개의 수광부로부터 얻어온 값 저장
char buffer[32]; //UART 버퍼

uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel);
void USARTPrint(USART_TypeDef* USARTx, volatile char *s);


//PE8 초음파 trig
void Init_GPIO_OUT_SonicWave()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOE, &GPIO_InitStructure);
}

//PE7 초음파 echo(in)
void Init_GPIO_IN_SonicWave()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOE, &GPIO_InitStructure);
}




void Init_GPIO_OUT_Right_Rpm()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Init_GPIO_OUT_Right_Dir()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
}



void Init_GPIO_OUT_Left_Rpm()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void Init_GPIO_OUT_Left_Dir()
{
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOD, &GPIO_InitStructure);
}




void Init_USART(void){

 //구조체변수 선언
GPIO_InitTypeDef GPIO_InitStructure; 
 USART_InitTypeDef USART_InitStructure; 
 NVIC_InitTypeDef NVIC_InitStructure; 
 // Enable peripheral
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
 // Configure USART Interrupt
 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f; 
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
 NVIC_Init(&NVIC_InitStructure); 

 // GPIO AF config
 GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
 GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // USART(또는 UART)의 TX 핀번호
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 // USART(또는 UART)의 RX 핀번호
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 // Configure UART peripheral
 USART_InitStructure.USART_BaudRate = 115200; //Baudrate설정.
USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
 USART_InitStructure.USART_StopBits = USART_StopBits_1; 
 USART_InitStructure.USART_Parity = USART_Parity_No; 
 USART_InitStructure.USART_HardwareFlowControl 
 = USART_HardwareFlowControl_None;
 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 

 USART_Init(USART1, &USART_InitStructure); 

 // Enable USART receive interrupt
 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 USART_Cmd(USART1, ENABLE);
}


//ADC1 설정 및 활성화
void Init_ADC1(){

 GPIO_InitTypeDef GPIO_InitStructure;
 ADC_InitTypeDef ADC_InitStructure;
 ADC_CommonInitTypeDef ADC_CommonInitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

 ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
 ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
 ADC_CommonInitStructure.ADC_DMAAccessMode =
 ADC_DMAAccessMode_Disabled;
 ADC_CommonInitStructure.ADC_TwoSamplingDelay =
 ADC_TwoSamplingDelay_5Cycles;
 ADC_CommonInit(&ADC_CommonInitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

 ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
 ADC_InitStructure.ADC_ScanConvMode = DISABLE;
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
 ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStructure.ADC_NbrOfConversion = 1;
 ADC_Init(ADC1, &ADC_InitStructure);
 ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
 ADC_Cmd(ADC1, ENABLE);
}


//ADC2 설정및 활성화
void Init_ADC2(){

 GPIO_InitTypeDef GPIO_InitStructure;
 ADC_InitTypeDef ADC_InitStructure;
 ADC_CommonInitTypeDef ADC_CommonInitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

 ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
 ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
 ADC_CommonInitStructure.ADC_DMAAccessMode =
 ADC_DMAAccessMode_Disabled;
 ADC_CommonInitStructure.ADC_TwoSamplingDelay =
 ADC_TwoSamplingDelay_5Cycles;
 ADC_CommonInit(&ADC_CommonInitStructure);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

 ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
 ADC_InitStructure.ADC_ScanConvMode = DISABLE;
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
 ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStructure.ADC_NbrOfConversion = 1;
 ADC_Init(ADC1, &ADC_InitStructure);
 ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
 ADC_Cmd(ADC2, ENABLE);
}

//ADC3 설정및 활성화
void Init_ADC3(){

 GPIO_InitTypeDef GPIO_InitStructure;
 ADC_InitTypeDef ADC_InitStructure;
 ADC_CommonInitTypeDef ADC_CommonInitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

 ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;

 ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
 ADC_CommonInitStructure.ADC_DMAAccessMode =
 ADC_DMAAccessMode_Disabled;
 ADC_CommonInitStructure.ADC_TwoSamplingDelay =
 ADC_TwoSamplingDelay_5Cycles;
 ADC_CommonInit(&ADC_CommonInitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

 ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
 ADC_InitStructure.ADC_ScanConvMode = DISABLE;
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
 ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStructure.ADC_NbrOfConversion = 1;
 ADC_Init(ADC1, &ADC_InitStructure);
 ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
 ADC_Cmd(ADC3, ENABLE);
}

void Init_ADC4(){ 
 GPIO_InitTypeDef GPIO_InitStructure;
 ADC_InitTypeDef ADC_InitStructure;
 ADC_CommonInitTypeDef ADC_CommonInitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

 ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
 ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;

 ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
 ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
 ADC_CommonInit(&ADC_CommonInitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
 ADC_InitStructure.ADC_ScanConvMode = DISABLE;
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
 ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

 ADC_InitStructure.ADC_NbrOfConversion = 1;
 ADC_Init(ADC1, &ADC_InitStructure);

 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles); 
 ADC_Cmd(ADC1, ENABLE);
}

//ADC로 수광부값을 받기위한 타이머 초기화
void Init_Timer_TIM5(){

 uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

 //Precalervalue 값을 계산한다. 168mhz /2 = 84mhz , 84mhz/1mhz = 84
 //마지막에 -1을 해줘야 타이머가 반복되면서 발생하는 오버플로우를 막을수 있다.
PrescalerValue = (uint16_t)(SystemCoreClock / 2 / TIMER_PRESCALER_FREQ) - 1;

 //타이머의 주기를 설정해준다 1Mhz / 120 = 8,333 hz
 TIM_TimeBaseStructure.TIM_Period=TIMER_PRESCALER_FREQ / TIMER_FREQ_5-1;

 //Prescaler 값을 설정한다. 위에서 계산한 84로 지정한다.
 //따라서 1/168mhz * 84(Prescaler) * 8.3khz(Period) = 0.004 sec 값이 계산된다. 
 //즉 0.004 초마다 타이머 인터럽트가 발생되고 
//그에 따라 인터럽트 핸들러가 동작된다.
TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM5, ENABLE);

}

//우측 바퀴
void Init_Timer_TIM2_R(){

 // uint16_t PrescalerValue;
 // TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);

 uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//Timer관련 구조체
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

 SystemCoreClockUpdate();

 //P168mhz /2 = 84mhz , 84mhz/1mhz = 84
 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 TIM_TimeBaseStructure.TIM_Period = 1000000 / TIMER_FREQ_2 - 1;
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM2, ENABLE);


}

//좌측 바퀴
void Init_Timer_TIM3_L(){

 // uint16_t PrescalerValue;
 // TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);

 uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//Timer관련 구조체
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

 SystemCoreClockUpdate();

 //P168mhz /2 = 84mhz , 84mhz/1mhz = 84
 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 TIM_TimeBaseStructure.TIM_Period = 1000000 / TIMER_FREQ_3 - 1;
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM3, ENABLE);


}
void Init_Timer_TIM4_USART(void)
{

 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);

 uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//Timer관련 구조체
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 SystemCoreClockUpdate();
 //P168mhz /2 = 84mhz , 84mhz/1mhz = 84
 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 TIM_TimeBaseStructure.TIM_Period = 1000000 / 50 - 1;
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM4, ENABLE);

}




void Init_USART2_BT(void){

 GPIO_InitTypeDef GPIO_InitStructure; //GPIO 초기화용 Structure를 선언
USART_InitTypeDef USART_InitStructure; // USART 초기화용 Structure를 선언
NVIC_InitTypeDef NVIC_InitStructure; // NVIC 초기화용 Structure 선언

// Enable peripheral
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //GPIOA를 Enable시킴 GPIOA는 AHB1에 위치
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART1을 Enalble 시킴 USART1은 APB2에 위치

// Configure USART Interrupt
 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // UART Interrupt을 등록함.핸들러는 TImer처럼 고정되어있음
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f; // Preemption 인터럽트 우선순위 설정
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f; // Sub 인터럽트 우선순위 설정
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ채널의 cmd를 Enable함.
NVIC_Init(&NVIC_InitStructure); // NVIC 초기화

// GPIO AF config
 GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
 GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // USART(또는 UART)의 TX 핀번호
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOD, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // USART(또는 UART)의 RX 핀번호
GPIO_Init(GPIOD, &GPIO_InitStructure);

 // Configure UART peripheral
 USART_InitStructure.USART_BaudRate = 115200; //Baudrate설정.
USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 전송하는 패킷의 길이는 8bit씩 전송함을 명시
USART_InitStructure.USART_StopBits = USART_StopBits_1; // stopbit는 1개임을 의미
USART_InitStructure.USART_Parity = USART_Parity_No; // 패리티 비트는 없다. 패리티 비트는 단지 에러가 있음을 알려줄뿐, 에러를 직접 고치지 못하기 때문에, 이는 UART데이터 전송속도에 안좋은 영향을 끼칠 것.(매우 긴 패킷에서만..)
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // HardwareFlowControl을 없다고 설정

USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //USART_Mode에서 RX, TX를 설정

USART_Init(USART2, &USART_InitStructure); // USART초기화

// Enable USART receive interrupt
 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

 USART_Cmd(USART2, ENABLE);
}


//유동적으로 Hz를 변경하여 펄스신호에 따라 모터 회전수 조절
void changeTimerXperiodRight(uint32_t TIMER_FREQ){

 //구조체변수 선언
uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

 //버스에 클럭인가
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 SystemCoreClockUpdate();

 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 //입력받은 인자에따라 주기를 변경시켜 타이머 인터럽트 호출 시간을 조절
TIM_TimeBaseStructure.TIM_Period = 1000000 / TIMER_FREQ - 1;
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM2, ENABLE);
}

void changeTimerXperiodLeft(uint32_t TIMER_FREQ){

 //구조체변수 선언
uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

 //버스에 클럭인가
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 SystemCoreClockUpdate();

 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 //입력받은 인자에따라 주기를 변경시켜 타이머 인터럽트 호출 시간을 조절
TIM_TimeBaseStructure.TIM_Period = 1000000 / TIMER_FREQ - 1;
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM3, ENABLE);
}

void Trig()
{

 delay_us(2);
 GPIOE->BSRRH = GPIO_Pin_8; 
 delay_us(10);
 GPIOE->BSRRL = GPIO_Pin_8;
 t2=1;
}


int main(void){

 Init_USART();
 Init_USART2_BT();


 Init_GPIO_OUT_SonicWave();
 Init_GPIO_IN_SonicWave();
 Init_GPIO_OUT_Left_Rpm();
 Init_GPIO_OUT_Left_Dir();
 Init_GPIO_OUT_Right_Rpm();
 Init_GPIO_OUT_Right_Dir();


 Init_Timer_TIM5();
 Init_Timer_TIM4_USART();
 Init_Timer_TIM2_R();
 Init_Timer_TIM3_L();


 Init_ADC1();
 Init_ADC2();
 Init_ADC3();
 Init_ADC4();


 while (1){

 if(t2==0)
 {
 Trig();
 }
 else if(t2 ==1)
 {
 if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7) != 0)
 {
 sonicwavestate = 1;
 count++;

 // if(count < 2000) stopflag = 1;
 // else stopflag = 0;

 }
 else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7) == 0 && sonicwavestate == 1)
 { 
 sonicwavestate = 0; 
 echostate = 1;

 }
 }


 }

}




//TIM5 인터럽트 핸들러 , 3개의 ADC 채널로부터 수광부 값을 읽어옴
//각각 x,y,z 변수에 저장하고 그에따라 flag값을 변환시킨다.
void TIM5_IRQHandler(void)
{
 if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
 {
 TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

 //ADC_SoftwareStartConv(ADC1);
 //ADC_SoftwareStartConv(ADC2);
 //ADC_SoftwareStartConv(ADC3);


 adc1RH = ADC_Read(ADC1,ADC_Channel_10);
 adc2RT = ADC_Read(ADC2,ADC_Channel_12);
 adc3LH = ADC_Read(ADC3,ADC_Channel_13);
 adc4LT = ADC_Read(ADC1,ADC_Channel_0); 



 }
}




/*
마프 <-> 모터 드라이버
PA14 : 1번(오른쪽 모터 클럭)
PA13 : 2번(오른쪽 모터 방향)
PD2 : 3번(왼쪽 모터 클럭)
PD0 : 4번(왼쪽 모터 방향)
*/

//x축
// 0 : 앞으로 저속 , 1 : 앞으로 중간속도, 2: 앞으로 최고속도
// 3 : 뒤로 저속 , 4: 뒤로 중간속도 , 5: 뒤로 최고속도

//y축
// 6 : 좌로 저속 회전 , 7: ,좌로 중간 회전 
// 8: 우로 저속 회적 , 9 우로 중간 회전 




//적외선 센서 값 uint16_t adcRH,adc2RT,adc3LH,adc4LT

//우측 바퀴 핸들러
void TIM2_IRQHandler(void)
{



 //인터럽트를 받을 때마다 아래 구문을 실행한다.
if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
 { 


 if(count < 900 && (check < 3))
 {
 changeTimerXperiodRight(1000);
 // GPIOA->BSRRL = GPIO_Pin_14;
 }
 else{
 if(adc3LH>=70 && check < 3) RightadjustG = 200;
 else if(adc3LH<70 )RightadjustG = 0;

 if (adc4LT>=90 && (check >= 3 && check < 6)) RightadjustB = 200;
 else if(adc4LT<90 )RightadjustB = 0;

 if(check == 10) {changeTimerXperiodRight(60);}
 else if(check == 0) {changeTimerXperiodRight(4000-RightadjustG);}
 else if(check == 1) {changeTimerXperiodRight(7000-RightadjustG);}
 else if(check == 2) {changeTimerXperiodRight(12000-RightadjustG);}
 else if(check == 3) {changeTimerXperiodRight(2000-RightadjustB);}
 else if(check == 4) {changeTimerXperiodRight(4000-RightadjustB);}
 else if(check == 5) {changeTimerXperiodRight(6000-RightadjustB);}
 else if(check == 6) {changeTimerXperiodRight(4000); }
 else if(check == 7) {changeTimerXperiodRight(8000); }
 else if(check == 8) {changeTimerXperiodRight(5000); }
 else if(check == 9) {changeTimerXperiodRight(5000); }
 } 



 //카운트를 다하면 자동으로 0으로 초기화시켜준다.
TIM_ClearITPendingBit(TIM2, TIM_IT_Update);


 if( check == 10)
 {
 GPIOA->BSRRL = GPIO_Pin_14;

 }

 else if(check < 3)
 {


 GPIOA->BSRRH = GPIO_Pin_13;



 if (R ==0)
 {
 GPIOA->BSRRH = GPIO_Pin_14;
 R=1;
 }

 else if(R==1)
 {
 GPIOA->BSRRL = GPIO_Pin_14;
 R=0;
 }
 }
 else if(check <6)
 {



 GPIOA->BSRRL = GPIO_Pin_13;

 if (R ==0)
 {
 GPIOA->BSRRH = GPIO_Pin_14;
 R=1;
 }

 else if(R==1)
 {
 GPIOA->BSRRL = GPIO_Pin_14;
 R=0;
 }
 }
 else if(check < 8)
 {

 GPIOA->BSRRH = GPIO_Pin_13;

 if (R ==0)
 {
 GPIOA->BSRRH = GPIO_Pin_14;
 R=1;
 }

 else if(R==1)
 {
 GPIOA->BSRRL = GPIO_Pin_14;
 R=0;
 }

 }
 else if(check <10)
 {

 GPIOA->BSRRH = GPIO_Pin_13;

 GPIOA->BSRRL = GPIO_Pin_14;


 }






 }





}


//좌측 바퀴 핸들러
void TIM3_IRQHandler(void)
{

 //인터럽트를 받을 때마다 아래 구문을 실행한다.
if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
 {

 if(count < 900 && (check < 3))
 {
 // GPIOD->BSRRL = GPIO_Pin_0;
 changeTimerXperiodLeft(1000);
 }
 else
 {
 if(adc1RH>=70 && check < 3) LeftadjustG = 200;
 else if(adc1RH<70 )LeftadjustG = 0;

 if (adc2RT>=70 && (check >= 3 && check < 6)) LeftadjustB = 200;
 else if(adc2RT<70 )LeftadjustB = 0;


 if(check == 10) {changeTimerXperiodLeft(60);}
 else if(check == 0) {changeTimerXperiodLeft(4350-LeftadjustG);}
 else if(check == 1) {changeTimerXperiodLeft(7240-LeftadjustG);}
 else if(check == 2) {changeTimerXperiodLeft(12350-LeftadjustG); }
 else if(check == 3) {changeTimerXperiodLeft(2000-LeftadjustB); }
 else if(check == 4) {changeTimerXperiodLeft(4000-LeftadjustB); }
 else if(check == 5) {changeTimerXperiodLeft(6000-LeftadjustB); }
 else if(check == 6) {changeTimerXperiodLeft(5000); }
 else if(check == 7) {changeTimerXperiodLeft(5000); }
 else if(check == 8) {changeTimerXperiodLeft(4000); }
 else if(check == 9) {changeTimerXperiodLeft(8000); }
 } 



 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);


 if( check == 10)
 {
 GPIOD->BSRRL = GPIO_Pin_0;

 }

 else if(check <3)
 {


 GPIOD->BSRRL = GPIO_Pin_2;

 if (L ==0)
 {
 GPIOD->BSRRH = GPIO_Pin_0;
 L=1;
 }

 else if(L==1)
 {
 GPIOD->BSRRL = GPIO_Pin_0;
 L=0;
 }


 }
 else if(check<6)
 {



 GPIOD->BSRRH = GPIO_Pin_2;

 if (L ==0)
 {
 GPIOD->BSRRH = GPIO_Pin_0;
 L=1;
 }

 else if(L==1)
 {
 GPIOD->BSRRL = GPIO_Pin_0;
 L=0;
 }

 }
 else if(check < 8)
 {
 GPIOD->BSRRL = GPIO_Pin_2;

 GPIOD->BSRRL = GPIO_Pin_0;



 }
 else if(check <10)
 {


 GPIOD->BSRRL = GPIO_Pin_2;

 if (L ==0)
 {
 GPIOD->BSRRH = GPIO_Pin_0;
 L=1;
 }

 else if(L==1)
 {
 GPIOD->BSRRL = GPIO_Pin_0;
 L=0;
 }



 }



 }
}



//pc로 보내는 USART 용 타이머
void TIM4_IRQHandler(void)
{

 //인터럽트를 받을 때마다 아래 구문을 실행한다.
if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
 {

 TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

 //sprintf(buffer, "%d",check); 가속도센서 값 출력
//sprintf(buffer, "%d stop:%d \n",count,stopflag); 


 sprintf(buffer, "a1:%d a2:%d a3:%d a4:%d \n",adc1RH,adc2RT,adc3LH,adc4LT); //적외선 센서값 출력
USARTPrint(USART1, buffer);


 t2=0;
 count = 0;
 echostate=0;

 }
}



int t = 0;

char buffer[32]; //UART 버퍼
void USART2_IRQHandler(void)
{
 while (USART_GetITStatus(USART2, USART_IT_RXNE)) // RX에 무슨 값이 들어왔는가?를 확인하는 방법.
 {
cBT = USART_ReceiveData(USART2);//마이크로 컨트롤러 내부의 RX에 일정 값이 들어모녀 한글자, 한글자씩 저장함.
 //USART_SendData(USART1, cBT); //오로지 한개의 문자만 보낼수 있음. 

if (cBT == '0')
 {
 check = 0;
 }
 else if (cBT == '1')
 {
 check = 1;
 }
 else if (cBT == '2')
 {
 check = 2;
 }
 else if (cBT == '3')
 {
 check = 3;
 }
 else if (cBT == '4')
 {
 check = 4;
 }
 else if (cBT == '5')
 {
 check = 5;
 }
 else if (cBT == '6')
 {
 check = 6;
 }
 else if (cBT == '7')
 {
 check = 7;
 }
 else if (cBT == '8')
 {
 check = 8;
 }
 else if (cBT == '9')
 {
 check = 9;
 }
 else if (cBT == 's')
 {
 check = 10;
 }


 }

}

uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel) {
 uint32_t timeout = 0xFFF;

 ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_15Cycles);

 /* Start software conversion */
 ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;

 /* Wait till done */
 while (!(ADCx->SR & ADC_SR_EOC)) {
 if (timeout-- == 0x00) {
 return 0;
 }
 }

 /* Return result */
 return ADCx->DR;
} 


//UART 인터럽트 핸들러
void USART1_IRQHandler(void)
{
 if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
 {
 char c = USART_ReceiveData(USART1);

 }
}


void USARTPrint(USART_TypeDef* USARTx, volatile char *s){
 while (*s){
 // wait until data register is empty
 while (!(USARTx->SR & 0x00000040));
 USART_SendData(USARTx, *s);
 *s++;
 }
} 