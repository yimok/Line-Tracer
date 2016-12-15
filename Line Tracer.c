#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
/*�ڵ� ���� ����
/ ���Ǹ��� Ÿ�̸� 3000hz ����
/ ���ܼ� 120hz ����
/ ���ܼ� ���ڿ����� flag���� ���ϰ�, 
/ �׿����� ���ڿ� �ٸ� ���ڸ� �Ѱ�
/ Ÿ�̸��޽�����
*/

#define TIMER_PRESCALER_FREQ 1000000 
#define TIMER_FREQ_2 1000 //����
#define TIMER_FREQ_3 1000 //����

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
int R= 0; // GPIO High/Low ���� �� ����
uint16_t x, y, z = 0; //3���� �����ηκ��� ���� �� ����
char buffer[32]; //UART ����

uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel);
void USARTPrint(USART_TypeDef* USARTx, volatile char *s);


//PE8 ������ trig
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

//PE7 ������ echo(in)
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

 //����ü���� ����
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
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // USART(�Ǵ� UART)�� TX �ɹ�ȣ
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 // USART(�Ǵ� UART)�� RX �ɹ�ȣ
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 // Configure UART peripheral
 USART_InitStructure.USART_BaudRate = 115200; //Baudrate����.
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


//ADC1 ���� �� Ȱ��ȭ
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


//ADC2 ������ Ȱ��ȭ
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

//ADC3 ������ Ȱ��ȭ
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

//ADC�� �����ΰ��� �ޱ����� Ÿ�̸� �ʱ�ȭ
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

 //Precalervalue ���� ����Ѵ�. 168mhz /2 = 84mhz , 84mhz/1mhz = 84
 //�������� -1�� ����� Ÿ�̸Ӱ� �ݺ��Ǹ鼭 �߻��ϴ� �����÷ο츦 ������ �ִ�.
PrescalerValue = (uint16_t)(SystemCoreClock / 2 / TIMER_PRESCALER_FREQ) - 1;

 //Ÿ�̸��� �ֱ⸦ �������ش� 1Mhz / 120 = 8,333 hz
 TIM_TimeBaseStructure.TIM_Period=TIMER_PRESCALER_FREQ / TIMER_FREQ_5-1;

 //Prescaler ���� �����Ѵ�. ������ ����� 84�� �����Ѵ�.
 //���� 1/168mhz * 84(Prescaler) * 8.3khz(Period) = 0.004 sec ���� ���ȴ�. 
 //�� 0.004 �ʸ��� Ÿ�̸� ���ͷ�Ʈ�� �߻��ǰ� 
//�׿� ���� ���ͷ�Ʈ �ڵ鷯�� ���۵ȴ�.
TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM5, ENABLE);

}

//���� ����
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
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//Timer���� ����ü
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

//���� ����
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
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//Timer���� ����ü
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
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//Timer���� ����ü
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

 GPIO_InitTypeDef GPIO_InitStructure; //GPIO �ʱ�ȭ�� Structure�� ����
USART_InitTypeDef USART_InitStructure; // USART �ʱ�ȭ�� Structure�� ����
NVIC_InitTypeDef NVIC_InitStructure; // NVIC �ʱ�ȭ�� Structure ����

// Enable peripheral
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //GPIOA�� Enable��Ŵ GPIOA�� AHB1�� ��ġ
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART1�� Enalble ��Ŵ USART1�� APB2�� ��ġ

// Configure USART Interrupt
 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // UART Interrupt�� �����.�ڵ鷯�� TImeró�� �����Ǿ�����
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f; // Preemption ���ͷ�Ʈ �켱���� ����
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f; // Sub ���ͷ�Ʈ �켱���� ����
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQä���� cmd�� Enable��.
NVIC_Init(&NVIC_InitStructure); // NVIC �ʱ�ȭ

// GPIO AF config
 GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
 GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // USART(�Ǵ� UART)�� TX �ɹ�ȣ
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOD, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // USART(�Ǵ� UART)�� RX �ɹ�ȣ
GPIO_Init(GPIOD, &GPIO_InitStructure);

 // Configure UART peripheral
 USART_InitStructure.USART_BaudRate = 115200; //Baudrate����.
USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �����ϴ� ��Ŷ�� ���̴� 8bit�� �������� ���
USART_InitStructure.USART_StopBits = USART_StopBits_1; // stopbit�� 1������ �ǹ�
USART_InitStructure.USART_Parity = USART_Parity_No; // �и�Ƽ ��Ʈ�� ����. �и�Ƽ ��Ʈ�� ���� ������ ������ �˷��ٻ�, ������ ���� ��ġ�� ���ϱ� ������, �̴� UART������ ���ۼӵ��� ������ ������ ��ĥ ��.(�ſ� �� ��Ŷ������..)
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // HardwareFlowControl�� ���ٰ� ����

USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //USART_Mode���� RX, TX�� ����

USART_Init(USART2, &USART_InitStructure); // USART�ʱ�ȭ

// Enable USART receive interrupt
 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

 USART_Cmd(USART2, ENABLE);
}


//���������� Hz�� �����Ͽ� �޽���ȣ�� ���� ���� ȸ���� ����
void changeTimerXperiodRight(uint32_t TIMER_FREQ){

 //����ü���� ����
uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

 //������ Ŭ���ΰ�
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 SystemCoreClockUpdate();

 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 //�Է¹��� ���ڿ����� �ֱ⸦ ������� Ÿ�̸� ���ͷ�Ʈ ȣ�� �ð��� ����
TIM_TimeBaseStructure.TIM_Period = 1000000 / TIMER_FREQ - 1;
 TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM2, ENABLE);
}

void changeTimerXperiodLeft(uint32_t TIMER_FREQ){

 //����ü���� ����
uint16_t PrescalerValue;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

 //������ Ŭ���ΰ�
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 SystemCoreClockUpdate();

 PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

 //�Է¹��� ���ڿ����� �ֱ⸦ ������� Ÿ�̸� ���ͷ�Ʈ ȣ�� �ð��� ����
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




//TIM5 ���ͷ�Ʈ �ڵ鷯 , 3���� ADC ä�ηκ��� ������ ���� �о��
//���� x,y,z ������ �����ϰ� �׿����� flag���� ��ȯ��Ų��.
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
���� <-> ���� ����̹�
PA14 : 1��(������ ���� Ŭ��)
PA13 : 2��(������ ���� ����)
PD2 : 3��(���� ���� Ŭ��)
PD0 : 4��(���� ���� ����)
*/

//x��
// 0 : ������ ���� , 1 : ������ �߰��ӵ�, 2: ������ �ְ�ӵ�
// 3 : �ڷ� ���� , 4: �ڷ� �߰��ӵ� , 5: �ڷ� �ְ�ӵ�

//y��
// 6 : �·� ���� ȸ�� , 7: ,�·� �߰� ȸ�� 
// 8: ��� ���� ȸ�� , 9 ��� �߰� ȸ�� 




//���ܼ� ���� �� uint16_t adcRH,adc2RT,adc3LH,adc4LT

//���� ���� �ڵ鷯
void TIM2_IRQHandler(void)
{



 //���ͷ�Ʈ�� ���� ������ �Ʒ� ������ �����Ѵ�.
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



 //ī��Ʈ�� ���ϸ� �ڵ����� 0���� �ʱ�ȭ�����ش�.
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


//���� ���� �ڵ鷯
void TIM3_IRQHandler(void)
{

 //���ͷ�Ʈ�� ���� ������ �Ʒ� ������ �����Ѵ�.
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



//pc�� ������ USART �� Ÿ�̸�
void TIM4_IRQHandler(void)
{

 //���ͷ�Ʈ�� ���� ������ �Ʒ� ������ �����Ѵ�.
if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
 {

 TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

 //sprintf(buffer, "%d",check); ���ӵ����� �� ���
//sprintf(buffer, "%d stop:%d \n",count,stopflag); 


 sprintf(buffer, "a1:%d a2:%d a3:%d a4:%d \n",adc1RH,adc2RT,adc3LH,adc4LT); //���ܼ� ������ ���
USARTPrint(USART1, buffer);


 t2=0;
 count = 0;
 echostate=0;

 }
}



int t = 0;

char buffer[32]; //UART ����
void USART2_IRQHandler(void)
{
 while (USART_GetITStatus(USART2, USART_IT_RXNE)) // RX�� ���� ���� ���Դ°�?�� Ȯ���ϴ� ���.
 {
cBT = USART_ReceiveData(USART2);//����ũ�� ��Ʈ�ѷ� ������ RX�� ���� ���� ����� �ѱ���, �ѱ��ھ� ������.
 //USART_SendData(USART1, cBT); //������ �Ѱ��� ���ڸ� ������ ����. 

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


//UART ���ͷ�Ʈ �ڵ鷯
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