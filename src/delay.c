
/******************************************/
/*           ����������������޹�˾     */        
/*Project:      FCT��������               */
/*Guest:                                  */
/*Name:             delay.c               */
/*Mcu chip:         Atmega64              */
/*Main clock:       �ڲ�����8MHz          */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

/*-----------------------------------------------------------------------
��ʱ����
ϵͳʱ�ӣ�8M�ڲ����ⲿ11.0592M
-----------------------------------------------------------------------*/
#define xtal 11

void delay_1us(void)                 //1us��ʱ����
{
	asm("nop");
}

void delay_nus(unsigned int n)       //N us��ʱ����
{
	unsigned int i=0;
	for (i=0;i<n;i++)
	delay_1us();
}
  
void delay_1ms(void) 				 //1ms��ʱ����
{
	unsigned int i;
	for(i=0;i<(unsigned int)(xtal*143-2);i++)
	{
		;
	}
}

void delay_nms(unsigned int n)		 //N ms��ʱ����				 
{
	unsigned int i;
	for(i=0;i<n;i++)
	{
		delay_1ms();
	}
}