/*������� �����̺� �ڵ�*/
/*CREATED BY Byeon*/
/*2015.06.09*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"


#define TIMER_PRESCALER_FREQ        1000000          // timer �Է� Ŭ�� 1MHz
#define TIMER_FREQ_2     100            //1//timer 2 ���ļ� 1000Hz      //��ȣ ����    
#define TIMER_FREQ_3    1



void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USARTPrint(USART_TypeDef* USARTx, volatile char *s);
void TIM3_IRQHandler(void);
void USART2_IRQHandler(void);

char cBT;
int btState=0;
char buffer_bt[32];


void Init_ADC1(){ 
        

       //����� ����ü ���� ���� 
       GPIO_InitTypeDef GPIO_InitStructure;
       ADC_InitTypeDef ADC_InitStructure;
       ADC_CommonInitTypeDef ADC_CommonInitStructure;
       
        //ADC1�� APB2 ������ ���������Ƿ� Ȱ��ȭ    
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
       //ADC1�� GPIOC�� ����ϹǷ� �����ִ� AHB1 ���� Ȱ��ȭ 
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


       //independent ���� ���� 
       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
       //ADC Prescaler ����
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
       //CPU �������� DMA�� ���� ��� ��Ȱ��ȭ
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       //����� �ΰ��� ���°��� ������ ������. 5~20����Ŭ �� ����, ���ø��� ª������ ����
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
       //������ ����ü ������ �Ѱ� Ȱ��ȭ
       ADC_CommonInit(&ADC_CommonInitStructure); 
       
       //ADC ä�� 10 �� gpio �� PC 0 �� ���Ƿ� �°� ����
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;


       //speed�� �� ������ �ϸ� ��µǴ� Digital���� �� �ε巴�� ����(sampling rate�� ����)
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
       GPIO_Init(GPIOC, &GPIO_InitStructure);
           //ADC ����ü �� ����
       //���ش��� 12bit�� ����(12, 10 , 8, 6 ����)
       ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
       //ScanConvMode�� disable�� �� ADC �̱�ä�η� ���
       ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
       //ContinuousConvMode�� ADC�� ��׶���� ���� ���� ������ ����
       ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
       
       //Edge���� �����ؼ� �ܺ��� Signal�� �޾Ƶ��δ�., 
       //ADC ��ȣ�� ������ ��𼭺��� �޾Ƶ����� ���� �ϴ°�, ���� �Ȱǵ���� ��
       ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

       //�ܺ� Ʈ���Ÿ� �̿��ؼ� ��ȯ�� �õ��Ұ��� �����ϴ°� T1(TIM1) CC(Capture/Compare)
       ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
      
       //Right �� Left ����, ADC���� ����ȭ�� �����͸� ó���ϴµ� 
       //8bit�� ���ʿ��� ä���� ����������, 10bit�� ��� 2bit�� ���´�. �̸� �������� right�� 
       //����Ͽ� ������Ʈ ���� ���´�.
       ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
       ADC_InitStructure.ADC_NbrOfConversion = 1;
       ADC_Init(ADC1, &ADC_InitStructure);
       
       //ADC ä�� config , (ADC_SampleTime �� 3,5,28 ... ����
       ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles); 

       //ADC ����
       ADC_Cmd(ADC1, ENABLE); 
}



void Init_ADC2(){ 
       //����� ����ü ���� ���� 
       GPIO_InitTypeDef GPIO_InitStructure;
       ADC_InitTypeDef ADC_InitStructure;
       ADC_CommonInitTypeDef ADC_CommonInitStructure;

       //���� Ȱ��ȭ        
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
       ADC_CommonInit(&ADC_CommonInitStructure); 

       //PC2 �� �� ADC12ä�� Ȱ��ȭ	
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
       GPIO_Init(GPIOC, &GPIO_InitStructure);
    
      //ADC ������ ���� �� �ʱ�ȭ
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
       //����� ����ü ���� ����         
       GPIO_InitTypeDef GPIO_InitStructure;
       ADC_InitTypeDef ADC_InitStructure;
       ADC_CommonInitTypeDef ADC_CommonInitStructure;

       //���� Ȱ��ȭ                
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); 
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
       ADC_CommonInit(&ADC_CommonInitStructure); 
       

       //PC3 �� �� ADC13ä�� Ȱ��ȭ	
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
       GPIO_Init(GPIOC, &GPIO_InitStructure);

       //ADC ������ ���� �� �ʱ�ȭ
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
       //Prescaler ���� ������ ���� ����.
       uint16_t PrescalerValue;    
       //Ÿ�̸� �������� ������ ����ü ���� , ���ͷ�Ʈ �������̺�� ���ͷ�Ʈ�� ����ϱ�         //���� NVIC ����ü ���� ����
       TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
       NVIC_InitTypeDef NVIC_InitStructure;

       //NVIC ����ü ���� �� �ʱ�ȭ�� ���ͷ�Ʈ Ȱ��ȭ
       NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);

       //TIM2 Ÿ�̸Ӵ� APB1 ������ ����ϹǷ� APB1 ���� Ȱ��ȭ
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
       //Precalervalue ���� ����Ѵ�. 168mhz /2 = 84mhz    ,   84mhz/1mhz = 84
       //�������� -1�� ����� Ÿ�̸Ӱ� �ݺ��Ǹ鼭 �߻��ϴ� �����÷ο츦 ������ �ִ�.
       PrescalerValue = (uint16_t) (SystemCoreClock / 2 / TIMER_PRESCALER_FREQ) - 1;
       //Ÿ�̸��� �ֱ⸦ �������ش� 1Mhz / 100  = 10,000 - 1
       TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / TIMER_FREQ_2 - 1;           
        //Prescaler ���� �����Ѵ�.  ������ ����� 84�� �����Ѵ�.
        //���� 1/168mhz  * 84(Prescaler)  * 10khz(Period) = 0.005 sec ���� ���ȴ�.
        //�� 0.0055 �ʸ��� Ÿ�̸� ���ͷ�Ʈ�� �߻��ǰ� 
        //�׿� ���� ���ͷ�Ʈ �ڵ鷯�� ���۵ȴ�.
       TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
       
             //Ŭ���� ������ ������ ���� �ʿ���� 0���� �Ѵ� , ������ �������� �������ش�.
       TIM_TimeBaseStructure.TIM_ClockDivision = 0;
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

       //Ÿ�̸�2�� �ʱ�ȭ �Ѵ�.
       TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
       //TIM2�� Ÿ�̸� ���ͷ�Ʈ�� �������� Ȱ��ȭ ��Ų��
       TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
       //TIM2 ī��Ʈ�� �����Ѵ�.
       TIM_Cmd(TIM2, ENABLE);       
}
       
       

void Init_USART2_BT(void){
        
        GPIO_InitTypeDef GPIO_InitStructure; //GPIO �ʱ�ȭ�� Structure�� ����
        USART_InitTypeDef USART_InitStructure; // USART �ʱ�ȭ��  Structure�� ����
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
        
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5; // USART(�Ǵ� UART)�� TX �ɹ�ȣ
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6; // USART(�Ǵ� UART)�� RX �ɹ�ȣ
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        // Configure UART peripheral
        USART_InitStructure.USART_BaudRate   = 115200; //Baudrate����.
        USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �����ϴ� ��Ŷ�� ���̴� 8bit�� �������� ���
        USART_InitStructure.USART_StopBits   = USART_StopBits_1; // stopbit�� 1������ �ǹ�
        USART_InitStructure.USART_Parity     = USART_Parity_No ; // �и�Ƽ ��Ʈ�� ����. �и�Ƽ ��Ʈ�� ���� ������ ������ �˷��ٻ�, ������ ���� ��ġ�� ���ϱ� ������, �̴� UART������ ���ۼӵ��� ������ ������ ��ĥ ��.(�ſ� �� ��Ŷ������..)
        USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None; // HardwareFlowControl�� ���ٰ� ����
        
        USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx; //USART_Mode���� RX, TX�� ����
        
        USART_Init(USART2, &USART_InitStructure); //  USART�ʱ�ȭ
        
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
	       
    
              
             
              
                      // 0 : ������ ���� , 1 : ������ �߰��ӵ�, 2: ������ �ְ�ӵ�
              // 3 : �ڷ� ���� , 4: �ڷ� �߰��ӵ� , 5: �ڷ� �ְ�ӵ�
              
              //x��
              if(x>2150)
              {
                    // +X��
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
                  
                  // -X��
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
       
              
              // 6 : �·�   ���� ȸ�� , 7: ,�·� �߰� ȸ��  
              // 8: ���  ���� ȸ�� , 9 ��� �߰� ȸ��  
                   //Y��
              if(y>2150)
              {
                  // +Y��
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
                  
                  // -Y��
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
        if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) // RX�� ���� ���� ���Դ°�?�� Ȯ���ϴ� ���.
        {
                cBT = USART_ReceiveData(USART2);//����ũ�� ��Ʈ�ѷ� ������ RX�� ���� ���� ����� �ѱ���, �ѱ��ھ� ������.
                
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

