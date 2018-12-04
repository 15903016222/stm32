/**
  ******************************************************************************
  * @file    Project/Template/main.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

unsigned char *P_RXD;//接收数据指针

unsigned int Num_RXD=0;//要打印字节区位码的字节数


unsigned char TxBuffer[64]={0,2,3,};//串口发送缓冲区

unsigned char RxBuffer[64]; //串口接收缓冲区

unsigned char Key0=0;

unsigned char Key0_Value=0;

unsigned char Key0_State=0;

unsigned char LED0_State=0;

unsigned char t;

unsigned char JG;//数据比较结果


void Key_Delay (void)
{
	unsigned int i;
	
	for(i=0;i<0xfff;i++);
	
	
}




void Time_Delay (void)
{
	unsigned int i;
		unsigned int j;
	
	for(i=0;i<0xf;i++)
	{
		
		for(j=0;j<0xFFFF;j++);
  };
	
	
}





/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

 

/* Private function prototypes -----------------------------------------------*/

//void Delay(int nCount);

int fputc(int ch,FILE *f)
{
    USART1->SR;  //USART_GetFlagStatus(USART1, USART_FLAG_TC)
    
    USART_SendData(USART1, (unsigned char) ch);
    
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
    
    return(ch);
}
uint8_t ch[16] = {0};
unsigned int flag = 0;
uint8_t count = 0;

void USART1_IRQHandler(void)
{
//    uint8_t ch;

	if ((count < 14) && (flag == 0)) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {     
        //ch = USART1->DR;
          ch[count++] = USART_ReceiveData(USART1);     
//          printf( "%c, [%d]\r\n", ch[count], count );
    }
	}
	else {
//		printf( "%s", ch );
		flag = 1;
	}
	USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
     
}

typedef enum {
  STM32F0,
  STM32F1,
  STM32F2,
  STM32F3,
  STM32F4,
  STM32F7,
  STM32L0,
  STM32L1,
  STM32L4,
  STM32H7,
}MCUTypedef;

u32 idAddr[]={
	0x1FFFF7AC,	/*STM32F0唯一ID起始地址*/
	0x1FFFF7E8,	/*STM32F1唯一ID起始地址*/
	0x1FFF7A10,	/*STM32F2唯一ID起始地址*/
	0x1FFFF7AC,	/*STM32F3唯一ID起始地址*/
	0x1FFF7A10,	/*STM32F4唯一ID起始地址*/
	0x1FF0F420,	/*STM32F7唯一ID起始地址*/
	0x1FF80050,	/*STM32L0唯一ID起始地址*/
	0x1FF80050,	/*STM32L1唯一ID起始地址*/
	0x1FFF7590,	/*STM32L4唯一ID起始地址*/
	0x1FF0F420  /*STM32H7唯一ID起始地址*/
}; 

// 库函数的方式操作SysTick的计时
//void SysTick_Config(void)
//{
//	// 失能SysTick计数器
//	SysTick_CounterCmd(SysTick_Counter_Disable);
//	// 失能SysTick的中断
//	SysTick_ITConfig(DISABLE);
//	// 设置SysTick的时钟源
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
//	// 设置SysTick重载值
//	SysTick_SetReload(9000 * 1000); // 9000代表1ms 9000*1000代表1秒 
//	// 使能SysTick的中断
//	// SysTick_ITConfig(); //暂时不使用计数完成中断
//	// 使能SysTick的计数器
//	SysTick_CounterCmd(SysTick_Counter_Enable);
//}

extern unsigned int count_flag;

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
/*
	u8 i,data;
	u8 *ID = NULL;
	u32 sec, min;

	
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	SysTick_init();

	
	data='A';
	for(i=0;i<30;i++)
	{
		USART_GetFlagStatus(USART1, USART_FLAG_TC);
		USART_SendData(USART1, data);
		data++;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET) ;
	}
	printf("\n\twww.yxarm.net");
	printf("\n\ti value is   %d",i);
	printf("\n\ti value is   %o",i);
	printf("\n\ti value is   %d,  %d",i+i,i*i);
	printf("\n\t-----------------------------");
	
	// 获取STM32F1XX的ID号
	printf ("\r\n STM32 ID is : \r\n");
	ID = (u8 *)idAddr[STM32F1];
	for (i = 0; i < 12; ++i) {
		printf ("%.2x ", ID[i]);
	}
		min = 1;
		sec = 30;
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	// 打印USART1的接收的数据打印到串口，每次只能接收到10个字节就会打印一次
	while (1){
//		delay_ms(300);
//		GPIO_SetBits(GPIOC, GPIO_Pin_13);
//		delay_ms(300);
//		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
//		if (1 == flag) {
//			printf ("%s", ch);
//			flag = 0;
//			count = 0;
//		}

		// SysTick的计时
		if(count_flag)
		{
			count_flag = 0;
			if ((++sec) == 60) {
				++min;
				sec = 0;
			}
			printf ("%.2d:%.2d 1\r\n", min, sec);
		}

	}
*/

	/* **** GPIO库的使用 **** */
/*	u8 i, data;
	RCC_Configuration();
	GPIO_Configuration();
	
	// CLK:PB5上升沿 CLA:PC11 == 1 DATA:PC10 输出数据
	// 模拟164芯片
	data = 0x30;
	GPIO_SetBits(GPIOC, GPIO_Pin_11);
	for (i = 0; i < 8; ++i) {
		GPIO_ResetBits(GPIOB, GPIO_Pin_5); // PB5 = 0;
		
		if ((data&0x80) == 0x00)
			GPIO_ResetBits(GPIOC, GPIO_Pin_10);
		else
			GPIO_SetBits(GPIOC, GPIO_Pin_10);
		
		GPIO_SetBits(GPIOB, GPIO_Pin_5);      // PB5 = 1;
		data <<= 1;
	}
	
	while (1) {
		Time_Delay();
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		Time_Delay();
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
	*/
	
	// 初始化
	
  /* Setup STM32 system (clock, PLL and Flash configuration) */
  SystemInit();

  RCC_Configuration();

  NVIC_Configuration();

  GPIO_Configuration();

  SysTick_init();		     //延时初始化

  SPI2_Init(); 			     //初始化SPI硬件口

	P_RXD=RxBuffer;//接收指针指向接收缓冲区
	
  USART_Configuration();  //USART1配置 

  TIM2_Config();			//定时器初始化 
	
	GPIOA->CRL = 0x00;
	GPIOA->CRH = 0xffffffff;

 
    OLED_Init();			 //初始化OLED      
  	OLED_ShowString(1,0, "0.96' OLED TEST");  
  	OLED_ShowString(1,16,"mcudev.taobao  ");  
   	OLED_ShowString(1,32,"2014-06-16");  
  	OLED_ShowString(1,48,"ASCII: ");  
  	OLED_ShowString(63,48,"CODE: "); 

		{
			unsigned char i;
			
			for(i=0;i<32;i++)TxBuffer[i]=i;
			
    }
		
		I2CWriteByte(0,TxBuffer,32); //写入长度
			
		I2CReadByte(0,RxBuffer,32);
			
		JG=Compare_Mem (TxBuffer,RxBuffer,32);
		
		if(JG==0)//对Eeprom进行读写判断;
		{
			OLED_ShowString(1,32,"Eeprom--OK  ");
			}
		else
			{
				OLED_ShowString(1,32,"Eeprom--Error  ");
      }

 
  /* Infinite loop */
  while (1)
  {
		
		OLED_ShowChar(48,48,t,16,1);// OLED_Refresh_Gram();
		
		t++;
		if(t>'~')t=' ';
		
		OLED_ShowNum(103,48,t,3,16);//
		
    
		if(Key0_State==0xff)
		{
      LED0_State=!LED0_State;
			 delay_ms(300);
			Key0_State=0;
     
      }
		
		if(LED0_State==0)
				 {
					 LED0_ON();//LED亮
					 delay_ms(1000);
					 LED0_OFF();//LED灭
					 delay_ms(1000); 
			 }
		
  }

}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {


   

  }
}
#endif



/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
