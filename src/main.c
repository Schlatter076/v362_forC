/******************************************/
/*           ����������������޹�˾     */
/*Project:     V362�����ſ���             */
/*Guest:                                  */
/*Name:             main.c                */
/*Mcu chip:         Atmega64              */
/*Main clock:       �ⲿ����11.0592MHz    */
/*Rtc clock:                              */
/*Author:                                 */
/*Create date:      2019.05.05            */
/*Design date:                            */
/*Complete date:                          */
/******************************************/
#include <iom64v.h>
#include <stdio.h>
#include <macros.h>
#include <port.h>
#include <default.h>
#include <delay.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <commands.h>

//===========================================
int tnum = 0;
uchar key_now = 0;
uchar key_old = 0;
uchar key_code = 0;
uchar key_cnt = 0;
uchar cnt = 0;

#define KEY_COUNTER 10
#define RETYR_TIMES 3

char *uart_statu = "init";
char *isStarted = "false";

void open_13_8V(void);
void close_13_8V(void);
void opentestprd_one(void);
void closetestprd_one(void);
void opentestprd_two(void);
void closetestprd_two(void);
void opentestprd_three(void);
void closetestprd_three(void);
void open_insu_test_left12(void);
void close_insu_test_left12(void);
void open_insu_test_left23(void);
void close_insu_test_left23(void);
void open_insu_test_right12(void);
void close_insu_test_right12(void);
void open_insu_test_right23(void);
void close_insu_test_right23(void);
void open_test(void);
void open_reset(void);
void open_speaker(void);
void close_speaker(void);
void open_limit_valve(void);
void close_limit_valve(void);
void open_terminal_valve(void);
void close_terminal_valve(void);
void open_product_valve(void);
void close_product_valve(void);

/*============================================================*/
/****************************�̵���1***************************/
void open_13_8V(void)
{
    set_bit(PORTB, 0);
}

void close_13_8V(void)//��Ʒ��Դ13.8V,5A
{
    clr_bit(PORTB, 0);
}

void opentestprd_one(void)//1�ŵ�ѹ������
{
    set_bit(PORTB, 1);
}

void closetestprd_one(void)
{
    clr_bit(PORTB, 1);
}

void opentestprd_two(void)//2�ŵ�ѹ������
{
    set_bit(PORTB, 2);
}

void closetestprd_two(void)
{
    clr_bit(PORTB, 2);
}

void opentestprd_three(void)//3�ŵ�ѹ������
{
    set_bit(PORTB, 3);
}

void closetestprd_three(void)
{
    clr_bit(PORTB, 3);
}

void open_insu_test_left12(void)//��Ʒ��1-2�ž�Ե�������
{
    set_bit(PORTB, 4); //��һ��Ϊ��Ե������
    set_bit(PORTB, 5); //�ڶ���Ϊ��Ե�Ǹ���
}

void close_insu_test_left12(void)
{
    clr_bit(PORTB, 4);
    clr_bit(PORTB, 5);
}

void open_insu_test_left23(void)//��Ʒ��2-3�ž�Ե�������
{
    set_bit(PORTB, 5);
    set_bit(PORTB, 6);
}

void close_insu_test_left23(void)
{
    clr_bit(PORTB, 5);
    clr_bit(PORTB, 6);
}

void open_insu_test_right12(void)//��Ʒ��1-2�ž�Ե�������
{
    set_bit(PORTC, 0);
    set_bit(PORTC, 1);
}

void close_insu_test_right12(void)
{
    clr_bit(PORTC, 0);
    clr_bit(PORTC, 1);
}

void open_insu_test_right23(void)//��Ʒ��2-3�ž�Ե�������
{
    set_bit(PORTC, 1);
    set_bit(PORTC, 2);
}

void close_insu_test_right23(void)
{
    clr_bit(PORTC, 1);
    clr_bit(PORTC, 2);
}

void open_test(void)//��Ե�ǿ�ʼ����
{
    delay_nms(200);
	set_bit(PORTC, 4);
    delay_nms(200);
    clr_bit(PORTC, 4);
    while(!Insulation_Pass() && !Insulation_Fail());
}

void open_reset(void)//��Ե�Ǹ�λ
{
    set_bit(PORTC, 5);
    delay_nms(200);
    clr_bit(PORTC, 5);
}

void open_speaker(void)//����
{
    set_bit(PORTC, 7);
}

void close_speaker(void)
{
    clr_bit(PORTC, 7);
}

void open_limit_valve(void)//��λ��ŷ�
{
    set_bit(PORTB, 7);
}

void close_limit_valve(void)
{
    clr_bit(PORTB, 7);
}

void open_terminal_valve(void)//���ӵ�ŷ�
{
    set_bit(PORTC, 3);
}

void close_terminal_valve(void)
{
    clr_bit(PORTC, 3);
}

void open_product_valve(void)//��Ʒ�պϵ�ŷ�
{
    set_bit(PORTC, 6);
}

void close_product_valve(void)
{
    clr_bit(PORTC, 6);
}
/****************************�̵���0***************************/
/*============================================================*/

/*============================================================*/
/***********USART0�����жϷ����� start**********************/
//USART���ջ�����
#define RX_BUFFER_SIZE 11                  //���ջ�������С���ɸ�����Ҫ�޸ġ�
unsigned char rx_buffer[RX_BUFFER_SIZE];   //������ջ�����
unsigned char rx_counter = 0;              //����rx_counterΪ����ڶ����е��ѽ��յ��ַ�������

//����һ����־λUsart0_RECVFlag1:=1��ʾ����0���յ���һ�����������ݰ�
//��port.h�ж���

#pragma interrupt_handler usart0_rxc_isr:19  //�����жϷ������
void usart0_rxc_isr(void)
{
    uchar status, data;
    status = UCSR0A;
    data = UDR0;
    if((flag1 & (1 << Usart0_RECVFlag1)) == 0) //�ж��Ƿ��������һ���µ����ݰ�
    {
        if ((status & (USART0_FRAMING_ERROR | USART0_PARITY_ERROR | USART0_DATA_OVERRUN)) == 0)
        {
            rx_buffer[rx_counter] = data;
            rx_counter++;
            switch (rx_counter)
            {
            case 1:       // ������ʼ�ַ�
            {
                if (data != FIRST_TEXT) rx_counter = 0;
            }
            break;
            case 2:
            {
                if (data != SECOND_TEXT) rx_counter = 0;
            }
            break;
            case 10:
            {
                if (data == 0x46) 
				{
				    uart_statu = "finished";
				}
                else uart_statu = "init";
            }
            break;
            case 11:      // ��������ַ�
            {
                rx_counter = 0;
                if (data == END_TEXT) set_bit(flag1, Usart0_RECVFlag1); // Usart0_RecvFlag=1����ʾ��ȷ���յ�һ�����ݰ�
            }
            break;
            default:
                break;
            }
        }
    }
}
/***************USART0�����жϷ����� end**********************/
/*============================================================*/
/*============================================================*/
/***************USART0�����жϷ����� start********************/
#define TX_BUFFER_SIZE 11
unsigned char tx_buffer[TX_BUFFER_SIZE];
unsigned char tx_wr_index = 0, tx_rd_index = 0, tx_counter = 0;

#pragma interrupt_handler usart0_txc_isr:21  //�����жϷ������
void usart0_txc_isr(void)
{
    if (tx_counter)//���в�Ϊ��
    {
        --tx_counter;//������
        UDR0 = tx_buffer[tx_rd_index];
        if (++tx_rd_index == TX_BUFFER_SIZE) tx_rd_index = 0;
    }
}
/***********USART0�����жϷ����� end**********************/

/*============================================================*/
/***********USART0����һ���ַ����� start**********************/
void USART0_putchar(unsigned char c)
{
    while (tx_counter == TX_BUFFER_SIZE);
    CLI();//#asm("cli")�ر�ȫ���ж�����
    if (tx_counter || ((UCSR0A & USART0_DATA_REGISTER_EMPTY) == 0)) //���ͻ�������Ϊ��
    {
        tx_buffer[tx_wr_index] = c; //���ݽ������
        if (++tx_wr_index == TX_BUFFER_SIZE) tx_wr_index = 0; //��������
        ++tx_counter;
    }
    else
        UDR0 = c;
    SEI(); //#asm("sei")��ȫ���ж�����
}
/***********USART0���ͷ����� end**********************/
/**
 * ���Ͷ��ֽں���
 * @arrs �ַ�����
 */
void send(unsigned char *arrs)
{
    uchar i = 0;
    for(i = 0; i < TX_BUFFER_SIZE; i++)
    {
        USART0_putchar(arrs[i]);
    }
}
/**
 * �����ַ�������(����)
 * @arrs �ַ���
 */
void trans(const unsigned char *arrs)
{
    while(*(arrs++) != '\0')
    {
        USART0_putchar(*(arrs - 1));
    }
}
//���Ͳ��躯��
void send_step(unsigned char step_counter, unsigned char test_mode, unsigned char insulation_result)
{
    step[2] = step_counter; //���ò��Բ���
    step[3] = test_mode; //ѡ���Ե���Ի��ѹ������
    step[4] = insulation_result; //����Ե���Խ��
    send(step);
}
//��ʱ��0��ʼ��
void init_TIMER0_OVF(void)
{
    TCCR0 = 0x06; //256��Ƶ
    TCNT0 = 256 - CRYSTAL / 256 / 2 * 0.5; //0.5s��ʱ
    TIMSK |= (1 << TOIE0); //��ʱ��0�ж�ʹ��
    SREG = 0x80;
}
#pragma interrupt_handler timer0_isr:17
void timer0_isr(void)
{
    TCNT0 = 256 - CRYSTAL / 256 / 2 * 0.05; //��װ0.5s��ʱ
    if(isStarted == "true")
    {
        if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
        {
            if(uart_statu == "finished")
            {
                isStarted = "false";
                clr_bit(flag1, Usart0_RECVFlag1);
            }
        }
    }
}


/************USART0�����ַ����麯��end****************/
//��װָ��ͺ���
void function(void (*fuc)())
{
    uchar cnt = 0;
    do
    {
        if(cnt++ >= RETYR_TIMES)
        {
            cnt = 0;
            break; //������Դ�������RETYR_TIMES,����ѭ��
        }
        (*fuc)();    //���Ͳ���ָ��
        delay_nms(300); //�ȴ���������ݷ���
    }
    while((flag1 & (1 << Usart0_RECVFlag1)) == 0);
}
void steps(uchar step, uchar statu1, uchar statu2)
{
    uchar cnt = 0;
	do
    {
        if(cnt++ >= RETYR_TIMES)
        {
            cnt = 0;
            break; //������Դ�������RETYR_TIMES,����ѭ��
        }
        send_step(step, statu1, statu2);    //���Ͳ���ָ��
        delay_nms(300); //�ȴ���λ�����ݷ���
    }
    while((flag1 & (1 << Usart0_RECVFlag1)) == 0);
}
/***************USART01��ʼ������ start*************************/
void init_usart0(void)
{
    UCSR0B = 0x00;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //�첽��8λ���ݣ�����żУ�飬һ��ֹͣλ���ޱ���
    UBRR0L = BAUD_L;                        //�趨������
    UBRR0H = BAUD_H;
    UCSR0A = 0x00;
    UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0); //0XD8  ���ա�����ʹ�ܣ� ���ж�ʹ��
}
/***************USART0��ʼ������ end***************************/
/***************ϵͳ��ʼ������ start ***********/
void init_cpu(void)
{
    EIMSK = 0x00; //����INT0~INT1�������ⲿ�ж�
    clr_bit(SFIOR, PUD); //������������������Ч

    DDRA = 0xff; //1�������0������
    PORTA = 0x00; //����PINC��PINFͨ·�л�

    DDRB = 0xff; //PINB���Ƽ̵���
    PORTB = 0x00;

    DDRC = 0xff;//PINC���Ƽ̵���
    PORTC = 0x00;

    DDRD  = 0x00; //PIND0/1Ϊ��Ե��Fail/Pass������,PIND2/3Ϊ��Ʒ��λ΢�����ؼ��,PIND4/5Ϊ��������,PIND6/7û�м��
    PORTD = 0x30;  //�������������������

    DDRE = 0xfe; //RXD0���룬��������Ч
    PORTE = 0x01; //TXD0���

    DDRF = 0xff; //PINFȫ�����
    PORTF = 0x00;

    DDRG = 0x00;
    PORTG = 0x00; //PING0,1,2Ϊ���׵�λ���

    init_usart0();
    init_TIMER0_OVF();
    SEI();
    clr_bit(flag2, is_NGFlag2);

    flag1 = 0;
    flag2 = 0;
    flag3 = 0;
    flagerr = 0;

}
/***************ϵͳ��ʼ������ end ***********/
//�Զ����Ժ���
void v362_auto_test(void)
{
    if(Has_Production() && isStarted == "false") //�в�Ʒ���ҹ�դû���ڵ�
    {
        isStarted = "true";
        while((flag1 && (1 << Usart0_RECVFlag1)) == 0)
        {
            send(start);
            delay_nms(200);
        }
        clr_bit(flag1, Usart0_RECVFlag1);
        if(rx_buffer[9] == 0x52) //-->��λ���������¿�ʼ
        {
            isStarted = "false";
            return;
        }
        else if(rx_buffer[9] == 0x53) //-->��λ����ȷ������ʼ
        {
            open_limit_valve();//����λ����
            delay_nms(500);
            if((PING & (1 << 0)) != 0)//��λ���׵�λ�Ÿп���
            {

                open_terminal_valve();//�����Ӷ��ӵ�����
                delay_nms(200);
                if((PING & (1 << 1)) != 0)//���Ӷ������׵�λ�Ÿп���
                {
                    open_insu_test_left12();//��1,2�ž�Ե����
                    delay_nms(1000);
                    open_test();
                    if(Insulation_Pass())//PD0-->Fail,PD1-->Pass
                    {
                        close_insu_test_left12();//�ر�1,2�Ų���
                        send_step(1, 0x10, 0x10);

                    }
                    else if(Insulation_Fail())
                    {
                        close_insu_test_left12();//�ر�1,2�Ų���
                        send_step(1, 0x10, 0x11);
                        set_bit(flag2, is_NGFlag2);
                        goto error;
                    }
                    delay_nms(20);

                    open_insu_test_left23();//��2,3�ž�Ե����
                    delay_nms(200);
                    open_test();
                    if(Insulation_Pass())
                    {
                        close_insu_test_left23();
                        send_step(2, 0x10, 0x10);
                    }
                    else if(Insulation_Fail())
                    {
                        close_insu_test_left23();
                        send_step(2, 0x10, 0x11);
                        set_bit(flag2, is_NGFlag2);
                        goto error;
                    }
                    delay_nms(20);

                    open_insu_test_right12();//��1,2�ž�Ե����
                    delay_nms(200);
                    open_test();
                    if(Insulation_Pass())
                    {
                        close_insu_test_right12();
                        send_step(3, 0x10, 0x10);
                    }
                    else if(Insulation_Fail())
                    {
                        close_insu_test_right12();
                        send_step(3, 0x10, 0x11);
                        set_bit(flag2, is_NGFlag2);
                        goto error;
                    }
                    delay_nms(20);

                    open_insu_test_right23();//��2,3�ž�Ե����
                    delay_nms(200);
                    open_test();
                    if(Insulation_Pass())
                    {
                        close_insu_test_right23();
                        send_step(4, 0x10, 0x10);
                    }
                    else if(Insulation_Fail())
                    {
                        close_insu_test_right23();
                        send_step(4, 0x10, 0x11);
                        set_bit(flag2, is_NGFlag2);
                        goto error;
                    }
                    delay_nms(20);
					open_reset();

                    open_13_8V();//�򿪲��Ե�ѹ13.8V,5A������һ�ε�Դ��ѹ
                    delay_nms(50);
                    opentestprd_one();//��Ʒδѹ��ʱ����1�ŵ�ѹ��Ϊ��Դ��ѹ
                    delay_nms(20);
					
					steps(5, 0x11, 0x10);
                    
                    delay_nms(1000);
                    if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
                    {
                        clr_bit(flag1, Usart0_RECVFlag1);
                        if(rx_buffer[9] == 0x4f)
                        {
                            closetestprd_one();
                        }
                        else if(rx_buffer[9] == 0x4e)
                        {
                            set_bit(flag2, is_NGFlag2);
                            goto error;
                        }
                    }
                    else
                    {
                        set_bit(flag2, is_NGFlag2);
                        //send(reset);
                        goto error;
                    }
					
                    open_product_valve();//��ѹ�ϲ�Ʒ������
                    delay_nms(1000);
                    if((PING & (1 << 2)) != 0)//ѹ�ϲ�Ʒ���׵�λ�Ÿп���
                    {
                        opentestprd_one();//����1�ŵĵ�ѹ��
                        delay_nms(200);
						
						steps(6, 0x11, 0x10);
                        
                        delay_nms(1000);
                        if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
                        {
                            clr_bit(flag1, Usart0_RECVFlag1);
                            if(rx_buffer[9] == 0x4f)
                            {
                                closetestprd_one();
                            }
                            else if(rx_buffer[9] == 0x4e)
                            {
                                set_bit(flag2, is_NGFlag2);
                                goto error;
                            }
                        }
                        else
                        {
                            set_bit(flag2, is_NGFlag2);
                            goto error;
                        }

                        opentestprd_two();//����2�ŵĵ�ѹ��
                        delay_nms(200);
						
						steps(7, 0x11, 0x10);
                        
                        delay_nms(1000);
                        if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
                        {
                            clr_bit(flag1, Usart0_RECVFlag1);
                            if(rx_buffer[9] == 0x4f)
                            {
                                closetestprd_two();
                            }
                            else if(rx_buffer[9] == 0x4e)
                            {
                                set_bit(flag2, is_NGFlag2);
                                goto error;
                            }
                        }
                        else
                        {
                            set_bit(flag2, is_NGFlag2);
                            goto error;
                        }


                        opentestprd_three();//����3�ŵĵ�ѹ��
                        delay_nms(200);
						
						steps(8, 0x11, 0x10);
                        
                        delay_nms(1000);
                        if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
                        {
                            clr_bit(flag1, Usart0_RECVFlag1);
                            if(rx_buffer[9] == 0x4f)
                            {
                                closetestprd_three();
                            }
                            else if(rx_buffer[9] == 0x4e)
                            {
                                set_bit(flag2, is_NGFlag2);
                                goto error;
                            }
                        }
                        else
                        {
                            set_bit(flag2, is_NGFlag2);
                            goto error;
                        }

                        close_13_8V();//�رճ̿ص�Դ�����
                        close_product_valve();//��Ʒѹ�����׻���
                        delay_nms(500);
						//while((PING & (1 << 2)) != 0);
                        //if((PING & (1 << 2)) == 0)
                        //{
                            opentestprd_one();//���Զϵ��ĵ�Դ��ѹ
                            delay_nms(20);
							
							steps(9, 0x11, 0x10);
                            
                            delay_nms(1000);
                            if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
                            {
                                clr_bit(flag1, Usart0_RECVFlag1);
                                if(rx_buffer[9] == 0x4f)
                                {
                                    close_terminal_valve();//������϶������׻���
									delay_nms(500);
									close_limit_valve();//���������λ���׻���
                                }
                                else if(rx_buffer[9] == 0x4e)
                                {
                                    set_bit(flag2, is_NGFlag2);
                                    goto error;
                                }
                            }
                            else
                            {
                                set_bit(flag2, is_NGFlag2);
                                goto error;
                            }

error:
                            isStarted = "false";
							clr_bit(flag1, Usart0_RECVFlag1);
							if((flag2 & (1 << is_NGFlag2)) != 0)
                            {
                                open_speaker();
                                open_reset();
                            }
                        //}
                    }
                }
            }
        }
    }
}
//����������===============================================
void key_scan(void)
{
    if((flag1 && (1 << keyprq_flag1)) == 0)  //���û�а�������
    {
        if((PIND & (1 << key1)) == 0) //�������԰���
        {
            key_now = 1;
        }
        /*
        else if((PIND & (1 << key2)) == 0) //��ͣ����
        {
        key_now = 2;
        }//*/
        else
        {
            key_now = 0;
            key_old = 0;
            key_code = 0;
        }
        if(key_now != 0)
        {
            if(key_now != key_code)
            {
                key_code = key_now;
                key_cnt = 0;
            }
            else
            {
                key_cnt++;
                if(key_cnt >= KEY_COUNTER)
                {
                    set_bit(flag1, keyprq_flag1);
                }
            }
        }
    }
}
//����������===============================================
void key_process(void)
{
    if((flag1 & (1 << keyprq_flag1)) != 0)
    {
        clr_bit(flag1, keyprq_flag1);
        if(key_code == key_old)
        {
            ; //do nothing~
        }
        else
        {
            key_old = key_code;
            set_bit(flag1, keyeff_flag1);  //������Ч
        }
        if((flag1 & (1 << keyeff_flag1)) != 0)
        {
            clr_bit(flag1, keyeff_flag1);
            switch(key_old)
            {
            case 1:  //�������԰�������
            {
                v362_auto_test();
            }
            break;
            /*
            case 2:  //��ͣ��������
            {

            } break;//*/
            default:
                break;
            }
        }
    }
}
/***************������ start *******************/
void main(void)
{
    init_cpu();    //��ʼ��CPU
	open_reset();
    send(reset);
    while(1)
    {
        key_scan();
        key_process();
        delay_nms(10);
    }
}