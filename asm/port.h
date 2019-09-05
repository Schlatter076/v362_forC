
/******************************************/
/*           ����������������޹�˾     */
/*Project:      FCT��������               */
/*Guest:                                  */
/*Name:             port.h                */
/*Mcu chip:         Atmega64              */
/*Main clock:       �ⲿ����11.0592MHz    */
/*Rtc clock:                              */
/*Author:           Jack.Fu               */
/*Create date:      2008.11.20            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/

#ifndef _PORT_H
#define _PORT_H


/*================================================*/
/************����MCU I/O�� start*******************/
//PORTA
#define LCD_data PORTA
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7

//PORTB 
#define PB0 0
#define PB1 1
#define PB2 2
#define PB3 3
#define PB4 4
#define PB5 5
#define PB6 6
#define PB7 7

//PORTC
#define PC0 0
#define PC1 1
#define PC2 2
#define PC3 3
#define PC4 4
#define PC5 5
#define PC6 6
#define PC7 7 

//PORTD
#define PD0  0
#define PD1  1
#define RXD1     2  //RXD���룬��������Ч
#define TXD1     3  //TXD���
#define key1 4  //�Զ����Կ�ʼ�İ���,����
#define key2 5  //��ͣ����������
#define bell 6  //���������ƣ�pnp�ܣ��͵�ƽ��Ч��  ���
/////////////
#define PD2 2
#define PD3 3
#define PD4 4
#define PD5 5
#define PD6 6
//////////////
#define PD7  7
 
//PORTE
#define RXD0     0  //RXD���룬��������Ч    
#define TXD0     1  //TXD���
#define LCD_RS   2
#define LCD_EN   3  //LCD�Ŀ��ƿڣ����
/////////////
#define PE2 2
#define PE3 3
//////////////
#define PE4  4
#define PE5  5
#define PE6  6
#define PE7  7
//PORTF
#define PF0 0
#define PF1 1
#define PF2 2
#define PF3 3
#define PF4 4
#define PF5 5
#define PF6 6
#define PF7 7

//PORTG
#define PG0 0
#define PG1 1
#define PG2 2
#define PG3 3
#define PG4 4

/************����MCU I/O�� end*********************/
/************����MCU Flag��־λ start**************/
#pragma global_register flag1:20 flag2:21 flag3:22 flagerr:23
unsigned char flag1;
#define keyprq_flag1     0	//�м����±�־λ
#define keyeff_flag1     1	//������Ч��־λ
#define Usart0_RECVFlag1 2 //=1��ʾ����0���յ���һ�����������ݰ�
#define Usart1_RECVFlag1 3 //=1��ʾ����1���յ���һ�����������ݰ�
#define error_flag1      4   //ͨ�Ŵ����־λ
#define AD0_RECVFlag1    5 //=1��ʾADC0���յ���һ�����������ݰ�
#define time_goFlag1     6
#define recv_okFlag1     7

unsigned char flag2;
#define allowMotorFlag2 0 //�����������־λ
#define posi_RECVFlag2 1 //�ɹ���ȡ�����λ��ֵ
#define init_COMPFlag2 2 //��ʼ����ɱ�־λ
#define motor_GOFlag2 3  //������б�־λ
#define motor_ARRIVALSFlag2 4 //����ָ��λ��
#define Alarm_LIMITFlag2 5 //������������λ��
#define is_NGFlag2 6 //NG��־λ
#define is_PASSFlag2 7 //������ɻع�����λ��

unsigned char flag3;
#define isRECFlag3 0
#define isCOMPFlag3 1
#define Z_BACKFlag3 2
#define timelimit_RECVFlag3 3//��ʱ���ձ�־λ
unsigned char flagerr;


/************����MCU Flag��־λ end****************/

#endif

