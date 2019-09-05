
/******************************************/
/*           ����������������޹�˾     */        
/*Project:      FCT��������               */
/*Guest:                                  */
/*Name:             delay.h               */
/*Mcu chip:         Atmega64              */
/*Main clock:       �ڲ�����8MHz          */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

#ifndef _DELAY_H
#define _DELAY_H

void delay_1us(void);
void delay_1ms(void);    
void delay_nus(unsigned int n);
void delay_nms(unsigned int n);

#endif