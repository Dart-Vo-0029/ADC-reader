
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "system_timetick.h"

#define MAX 14
#define volt_lvl 3300
void init_main(void);
void IntToStr4(int16_t u, uint8_t *y){
	uint16_t a;
	a=u;
	if(a<0){
		a=-a;
		y[0] ='-';
	}
	else y[0] =' ';
	y[4] =a%10 +0x30;
	a=a/10;
	y[3] =a%10 +0x30;
	a=a/10;
	y[2] =a%10 +0x30;
	a=a/10;
	y[1] =a +0x30;
}
uint16_t ADC_read(void) {
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
		ADC_SoftwareStartConv(ADC1);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}
int main(void)
{
	/* Enable SysTick at 10ms interrupt */
	SysTick_Config(SystemCoreClock/100);

	init_main();
	uint16_t adc_val,truncate_val;
	uint8_t send_val[5], *temp, done_arr[5]={'V','a','l',':',' '},unit_need[3]={'m','V','\n'};
	int flag_next;
	while(1){
		 flag_next=0;
			adc_val= ADC_GetConversionValue(ADC1);
		  adc_val=ADC_read();
			float tempo_val= adc_val*volt_lvl /4095; // lay mau
			truncate_val=(int16_t)tempo_val;
			temp= send_val;
			IntToStr4(truncate_val,temp);
			
			while(flag_next==0){
					if (tick_count == 100) {
						tick_count = 0;
							for(int i=0;i<5;i++){
							while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
								USART_SendData(UART5, done_arr[i]);
							} // Xuat val:
								
							for(int i=0;i<5;i++){
							while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
							USART_SendData(UART5, send_val[i]); 
							}
							for(int i=0;i<3;i++){
							while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
							USART_SendData(UART5, unit_need[i]); 
							}
							flag_next=1;

			}
    }
	}
}
void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;	
	
	  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	
	
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); 

  /* GPIO Configuration for UART5 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

	
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;//115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(UART5, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(UART5, ENABLE);



	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  // Analog mode
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		ADC_StructInit(&ADC_InitStructure);
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel7 configuration **************************************/
  


 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC1, DISABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC1, ENABLE);
}

