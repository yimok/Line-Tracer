/*블루투스 슬레이브 코드*/
/*CREATED BY Byeon*/
/*2015.06.09*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"


#define TIMER_PRESCALER_FREQ        1000000          // timer 입력 클력 1MHz
#define TIMER_FREQ_2     100            //1//timer 2 주파수 1000Hz      //신호 제어    
#define TIMER_FREQ_3    1



void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USARTPrint(USART_TypeDef* USARTx, volatile char *s);
void TIM3_IRQHandler(void);
void USART2_IRQHandler(void);

char cBT;
int btState=0;
char buffer_bt[32];


void Init_ADC1(){ 
        

       //사용할 구조체 변수 설정 
       GPIO_InitTypeDef GPIO_InitStructure;
       ADC_InitTypeDef ADC_InitStructure;
       ADC_CommonInitTypeDef ADC_CommonInitStructure;
       
        //ADC1은 APB2 버스에 물려있으므로 활성화    
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
       //ADC1은 GPIOC를 사용하므로 물려있는 AHB1 버스 활성화 
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


       //independent 모드로 설정 
       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
       //ADC Prescaler 설정
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
       //CPU 참여없이 DMA를 쓰는 모드 비활성화
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       //추출된 두개의 샘픈간의 간격을 설정함. 5~20사이클 이 있음, 샘플링이 짧을수록 정밀
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
       //설정된 구조체 변수를 넘겨 활성화
       ADC_CommonInit(&ADC_CommonInitStructure); 
       
       //ADC 채널 10 은 gpio 핀 PC 0 를 쓰므로 맞게 설정
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;


       //speed를 더 빠르게 하면 출력되는 Digital값이 더 부드럽게 나옴(sampling rate와 연관)
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
       GPIO_Init(GPIOC, &GPIO_InitStructure);
           //ADC 구조체 값 설정
       //분해능을 12bit로 설정(12, 10 , 8, 6 가능)
       ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
       //ScanConvMode를 disable로 해 ADC 싱글채널로 사용
       ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
       //ContinuousConvMode는 ADC를 백그라운드로 돌릴 건지 말건지 결정
       ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
       
       //Edge부터 시작해서 외부의 Signal을 받아들인다., 
       //ADC 신호의 시작을 어디서부터 받아들일지 설정 하는것, 굳이 안건드려도 됨
       ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

       //외부 트리거를 이용해서 변환을 시도할건지 설정하는것 T1(TIM1) CC(Capture/Compare)
       ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
      
       //Right 와 Left 설정, ADC에서 이진화된 데이터를 처리하는데 
       //8bit는 왼쪽에서 채워도 문제없지만, 10bit의 경우 2bit가 남는다. 이를 막기위해 right로 
       //사용하여 레지스트 낭비를 막는다.
       ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
       ADC_InitStructure.ADC_NbrOfConversion = 1;
       ADC_Init(ADC1, &ADC_InitStructure);
       
       //ADC 채널 config , (ADC_SampleTime 은 3,5,28 ... 가능
       ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles); 

       //ADC 가동
       ADC_Cmd(ADC1, ENABLE); 
}



void Init_ADC2(){ 
       //사용할 구조체 변수 설정 
       GPIO_InitTypeDef GPIO_InitStructure;
       ADC_InitTypeDef ADC_InitStructure;
       ADC_CommonInitTypeDef ADC_CommonInitStructure;

       //버스 활성화        
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
       ADC_CommonInit(&ADC_CommonInitStructure); 

       //PC2 번 핀 ADC12채널 활성화	
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
       GPIO_Init(GPIOC, &GPIO_InitStructure);
    
      //ADC 설정값 지정 및 초기화
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

void Init_ADC3(){ 
       //사용할 구조체 변수 설정         
       GPIO_InitTypeDef GPIO_InitStructure;
       ADC_InitTypeDef ADC_InitStructure;
       ADC_CommonInitTypeDef ADC_CommonInitStructure;

       //버스 활성화                
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); 
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
       ADC_CommonInit(&ADC_CommonInitStructure); 
       

       //PC3 번 핀 ADC13채널 활성화	
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
       GPIO_Init(GPIOC, &GPIO_InitStructure);

       //ADC 설정값 지정 및 초기화
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



void Init_GPIO() {
        GPIO_InitTypeDef GPIO_InitStructure;            
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;                 
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;        
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        
        
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
        GPIO_Init(GPIOA, &GPIO_InitStructure);                         
}
void Init_Timer(){
       //Prescaler 값을 저장할 변수 선언.
       uint16_t PrescalerValue;    
       //타이머 설정값을 세팅할 구조체 변수 , 인터럽트 벡터테이블과 인터럽트를 사용하기         //위해 NVIC 구조체 변수 선언
       TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
       NVIC_InitTypeDef NVIC_InitStructure;

       //NVIC 구조체 설정 및 초기화후 인터럽트 활성화
       NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);

       //TIM2 타이머는 APB1 버스를 사용하므로 APB1 버스 활성화
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
       //Precalervalue 값을 계산한다. 168mhz /2 = 84mhz    ,   84mhz/1mhz = 84
       //마지막에 -1을 해줘야 타이머가 반복되면서 발생하는 오버플로우를 막을수 있다.
       PrescalerValue = (uint16_t) (SystemCoreClock / 2 / TIMER_PRESCALER_FREQ) - 1;
       //타이머의 주기를 설정해준다 1Mhz / 100  = 10,000 - 1
       TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / TIMER_FREQ_2 - 1;           
        //Prescaler 값을 설정한다.  위에서 계산한 84로 지정한다.
        //따라서 1/168mhz  * 84(Prescaler)  * 10khz(Period) = 0.005 sec 값이 계산된다.
        //즉 0.0055 초마다 타이머 인터럽트가 발생되고 
        //그에 따라 인터럽트 핸들러가 동작된다.
       TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
       
             //클럭을 나누는 변수로 나눌 필요없이 0으로 한다 , 나머지 설정값을 세팅해준다.
       TIM_TimeBaseStructure.TIM_ClockDivision = 0;
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

       //타이머2를 초기화 한다.
       TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
       //TIM2를 타이머 인터럽트로 쓰기위해 활성화 시킨다
       TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
       //TIM2 카운트를 시작한다.
       TIM_Cmd(TIM2, ENABLE);       
}
       
       

void Init_USART2_BT(void){
        
        GPIO_InitTypeDef GPIO_InitStructure; //GPIO 초기화용 Structure를 선언
        USART_InitTypeDef USART_InitStructure; // USART 초기화용  Structure를 선언
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
        
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5; // USART(또는 UART)의 TX 핀번호
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6; // USART(또는 UART)의 RX 핀번호
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        // Configure UART peripheral
        USART_InitStructure.USART_BaudRate   = 115200; //Baudrate설정.
        USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 전송하는 패킷의 길이는 8bit씩 전송함을 명시
        USART_InitStructure.USART_StopBits   = USART_StopBits_1; // stopbit는 1개임을 의미
        USART_InitStructure.USART_Parity     = USART_Parity_No ; // 패리티 비트는 없다. 패리티 비트는 단지 에러가 있음을 알려줄뿐, 에러를 직접 고치지 못하기 때문에, 이는 UART데이터 전송속도에 안좋은 영향을 끼칠 것.(매우 긴 패킷에서만..)
        USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None; // HardwareFlowControl을 없다고 설정
        
        USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx; //USART_Mode에서 RX, TX를 설정
        
        USART_Init(USART2, &USART_InitStructure); //  USART초기화
        
        // Enable USART receive interrupt
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
        
        USART_Cmd(USART2, ENABLE); 
}
int main(){
        
  
  	Init_ADC1();
	Init_ADC2();
	Init_ADC3();
        
        
        Init_GPIO();
        Init_USART2_BT();
        Init_Timer();      
        while(1){
        }
}

uint16_t x ,y ,z =0;
char buffer[32];

void TIM2_IRQHandler(void)
{
       if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
       {                
	      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);        
	      
	       ADC_SoftwareStartConv(ADC1);  
                ADC_SoftwareStartConv(ADC2);    
                ADC_SoftwareStartConv(ADC3);   
	      if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET)      
	      {
		   x = ADC_GetConversionValue(ADC1);
                     y = ADC_GetConversionValue(ADC2);
                     z = ADC_GetConversionValue(ADC3);
	      }  
	       
    
              
             
              
                      // 0 : 앞으로 저속 , 1 : 앞으로 중간속도, 2: 앞으로 최고속도
              // 3 : 뒤로 저속 , 4: 뒤로 중간속도 , 5: 뒤로 최고속도
              
              //x축
              if(x>2150)
              {
                    // +X축
                    if(x>2650)
                    {
                          sprintf(buffer, "%d",2); 
                          USARTPrint(USART2, buffer);
              
                    }
                    else if (x>2450)
                    {
                         sprintf(buffer, "%d",1); 
                          USARTPrint(USART2, buffer);
                     }
                    else if(x>2300)
                    {
                         sprintf(buffer, "%d",0); 
                          USARTPrint(USART2, buffer);
 
                    }
              }
              else if(x<2000)
              {
                  
                  // -X축
                  if(x<1650)
                  {
                          sprintf(buffer, "%d",5); 
                          USARTPrint(USART2, buffer);
                  }
                  else if (x<1700)
                  {
                          sprintf(buffer, "%d",4); 
                          USARTPrint(USART2, buffer);
                     
                  }
                  else if(x<1800)
                  {
                          sprintf(buffer, "%d",3); 
                          USARTPrint(USART2, buffer);
                  }              
              }
       
              
              // 6 : 좌로   저속 회전 , 7: ,좌로 중간 회전  
              // 8: 우로  저속 회적 , 9 우로 중간 회전  
                   //Y축
              if(y>2150)
              {
                  // +Y축
                  if(y>2450)
                  {
                        sprintf(buffer, "%d",7); 
                          USARTPrint(USART2, buffer);
                   }
                  else if (y>2300)
                  {
                        sprintf(buffer, "%d",6); 
                          USARTPrint(USART2, buffer);
                   }
            
              }
              else if(y<2000)
              {
                  
                  // -Y축
                  if(y<1650)
                  {
                          sprintf(buffer, "%d",9); 
                          USARTPrint(USART2, buffer);
              }
                  else if (y<1800)
                  {
                        sprintf(buffer, "%d",8); 
                          USARTPrint(USART2, buffer);
                   }
                         
              }
              
              
              
              if((x<2110 && x>2020) && (y<2110 && y>2020))
              {
                 sprintf(buffer, "%c",'s'); 
                 USARTPrint(USART2, buffer);
              }
              

       }
}


int t=0;
void USART2_IRQHandler(void)
{
        if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) // RX에 무슨 값이 들어왔는가?를 확인하는 방법.
        {
                cBT = USART_ReceiveData(USART2);//마이크로 컨트롤러 내부의 RX에 일정 값이 들어모녀 한글자, 한글자씩 저장함.
                
        }
}
void USART_puts(USART_TypeDef* USARTx, volatile char *s){
	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USARTPrint(USART_TypeDef* USARTx, volatile char *s){
	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}        

