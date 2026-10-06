#include "app_usart1.h"// 调试串口 USART1（PA9/PA10，115200）



//对串口1进行初始化，作为平衡车的调试接口

void App_USART1_Init(void)
{
	//1.对PA9和PA10进行初始化，PA9-Tx-AF_PP,PA10-Rx-IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	//2.开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	//3.初始化USART1的参数
	USART_InitTypeDef USART_InitStruct = {0};
	
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
			
	USART_Init(USART1,&USART_InitStruct);
	//4.闭合USART1的总开关
	USART_Cmd(USART1,ENABLE);
}
