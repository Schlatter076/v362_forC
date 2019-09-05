/******************************************/
/*           广州旭东阪田电子有限公司     */
/*Project:     V362侧拉门开关             */
/*Guest:                                  */
/*Name:             main.c                */
/*Mcu chip:         Atmega64              */
/*Main clock:       外部晶体11.0592MHz    */
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
/****************************继电器1***************************/
void open_13_8V(void)
{
    set_bit(PORTB, 0);
}

void close_13_8V(void)//产品电源13.8V,5A
{
    clr_bit(PORTB, 0);
}

void opentestprd_one(void)//1脚电压降测试
{
    set_bit(PORTB, 1);
}

void closetestprd_one(void)
{
    clr_bit(PORTB, 1);
}

void opentestprd_two(void)//2脚电压降测试
{
    set_bit(PORTB, 2);
}

void closetestprd_two(void)
{
    clr_bit(PORTB, 2);
}

void opentestprd_three(void)//3脚电压降测试
{
    set_bit(PORTB, 3);
}

void closetestprd_three(void)
{
    clr_bit(PORTB, 3);
}

void open_insu_test_left12(void)//产品左1-2脚绝缘电阻测试
{
    set_bit(PORTB, 4); //第一行为绝缘仪正极
    set_bit(PORTB, 5); //第二行为绝缘仪负极
}

void close_insu_test_left12(void)
{
    clr_bit(PORTB, 4);
    clr_bit(PORTB, 5);
}

void open_insu_test_left23(void)//产品左2-3脚绝缘电阻测试
{
    set_bit(PORTB, 5);
    set_bit(PORTB, 6);
}

void close_insu_test_left23(void)
{
    clr_bit(PORTB, 5);
    clr_bit(PORTB, 6);
}

void open_insu_test_right12(void)//产品右1-2脚绝缘电阻测试
{
    set_bit(PORTC, 0);
    set_bit(PORTC, 1);
}

void close_insu_test_right12(void)
{
    clr_bit(PORTC, 0);
    clr_bit(PORTC, 1);
}

void open_insu_test_right23(void)//产品右2-3脚绝缘电阻测试
{
    set_bit(PORTC, 1);
    set_bit(PORTC, 2);
}

void close_insu_test_right23(void)
{
    clr_bit(PORTC, 1);
    clr_bit(PORTC, 2);
}

void open_test(void)//绝缘仪开始测试
{
    delay_nms(200);
	set_bit(PORTC, 4);
    delay_nms(200);
    clr_bit(PORTC, 4);
    while(!Insulation_Pass() && !Insulation_Fail());
}

void open_reset(void)//绝缘仪复位
{
    set_bit(PORTC, 5);
    delay_nms(200);
    clr_bit(PORTC, 5);
}

void open_speaker(void)//报警
{
    set_bit(PORTC, 7);
}

void close_speaker(void)
{
    clr_bit(PORTC, 7);
}

void open_limit_valve(void)//限位电磁阀
{
    set_bit(PORTB, 7);
}

void close_limit_valve(void)
{
    clr_bit(PORTB, 7);
}

void open_terminal_valve(void)//端子电磁阀
{
    set_bit(PORTC, 3);
}

void close_terminal_valve(void)
{
    clr_bit(PORTC, 3);
}

void open_product_valve(void)//产品闭合电磁阀
{
    set_bit(PORTC, 6);
}

void close_product_valve(void)
{
    clr_bit(PORTC, 6);
}
/****************************继电器0***************************/
/*============================================================*/

/*============================================================*/
/***********USART0接收中断服务函数 start**********************/
//USART接收缓冲区
#define RX_BUFFER_SIZE 11                  //接收缓冲区大小，可根据需要修改。
unsigned char rx_buffer[RX_BUFFER_SIZE];   //定义接收缓冲区
unsigned char rx_counter = 0;              //定义rx_counter为存放在队列中的已接收到字符个数。

//定义一个标志位Usart0_RECVFlag1:=1表示串口0接收到了一个完整的数据包
//在port.h中定义

#pragma interrupt_handler usart0_rxc_isr:19  //接收中断服务程序
void usart0_rxc_isr(void)
{
    uchar status, data;
    status = UCSR0A;
    data = UDR0;
    if((flag1 & (1 << Usart0_RECVFlag1)) == 0) //判断是否允许接收一个新的数据包
    {
        if ((status & (USART0_FRAMING_ERROR | USART0_PARITY_ERROR | USART0_DATA_OVERRUN)) == 0)
        {
            rx_buffer[rx_counter] = data;
            rx_counter++;
            switch (rx_counter)
            {
            case 1:       // 检验起始字符
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
            case 11:      // 检验结束字符
            {
                rx_counter = 0;
                if (data == END_TEXT) set_bit(flag1, Usart0_RECVFlag1); // Usart0_RecvFlag=1，表示正确接收到一个数据包
            }
            break;
            default:
                break;
            }
        }
    }
}
/***************USART0接收中断服务函数 end**********************/
/*============================================================*/
/*============================================================*/
/***************USART0发送中断服务函数 start********************/
#define TX_BUFFER_SIZE 11
unsigned char tx_buffer[TX_BUFFER_SIZE];
unsigned char tx_wr_index = 0, tx_rd_index = 0, tx_counter = 0;

#pragma interrupt_handler usart0_txc_isr:21  //发送中断服务程序
void usart0_txc_isr(void)
{
    if (tx_counter)//队列不为空
    {
        --tx_counter;//出队列
        UDR0 = tx_buffer[tx_rd_index];
        if (++tx_rd_index == TX_BUFFER_SIZE) tx_rd_index = 0;
    }
}
/***********USART0发送中断服务函数 end**********************/

/*============================================================*/
/***********USART0发送一个字符函数 start**********************/
void USART0_putchar(unsigned char c)
{
    while (tx_counter == TX_BUFFER_SIZE);
    CLI();//#asm("cli")关闭全局中断允许
    if (tx_counter || ((UCSR0A & USART0_DATA_REGISTER_EMPTY) == 0)) //发送缓冲器不为空
    {
        tx_buffer[tx_wr_index] = c; //数据进入队列
        if (++tx_wr_index == TX_BUFFER_SIZE) tx_wr_index = 0; //队列已满
        ++tx_counter;
    }
    else
        UDR0 = c;
    SEI(); //#asm("sei")打开全局中断允许
}
/***********USART0发送服务函数 end**********************/
/**
 * 发送多字节函数
 * @arrs 字符数组
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
 * 发送字符串函数(保留)
 * @arrs 字符串
 */
void trans(const unsigned char *arrs)
{
    while(*(arrs++) != '\0')
    {
        USART0_putchar(*(arrs - 1));
    }
}
//发送步骤函数
void send_step(unsigned char step_counter, unsigned char test_mode, unsigned char insulation_result)
{
    step[2] = step_counter; //设置测试步数
    step[3] = test_mode; //选择绝缘测试或电压降测试
    step[4] = insulation_result; //填充绝缘测试结果
    send(step);
}
//定时器0初始化
void init_TIMER0_OVF(void)
{
    TCCR0 = 0x06; //256分频
    TCNT0 = 256 - CRYSTAL / 256 / 2 * 0.5; //0.5s定时
    TIMSK |= (1 << TOIE0); //定时器0中断使能
    SREG = 0x80;
}
#pragma interrupt_handler timer0_isr:17
void timer0_isr(void)
{
    TCNT0 = 256 - CRYSTAL / 256 / 2 * 0.05; //重装0.5s定时
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


/************USART0发送字符数组函数end****************/
//封装指令发送函数
void function(void (*fuc)())
{
    uchar cnt = 0;
    do
    {
        if(cnt++ >= RETYR_TIMES)
        {
            cnt = 0;
            break; //如果重试次数大于RETYR_TIMES,结束循环
        }
        (*fuc)();    //发送测试指令
        delay_nms(300); //等待被测板数据返回
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
            break; //如果重试次数大于RETYR_TIMES,结束循环
        }
        send_step(step, statu1, statu2);    //发送测试指令
        delay_nms(300); //等待上位机数据返回
    }
    while((flag1 & (1 << Usart0_RECVFlag1)) == 0);
}
/***************USART01初始化函数 start*************************/
void init_usart0(void)
{
    UCSR0B = 0x00;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  //异步，8位数据，无奇偶校验，一个停止位，无倍速
    UBRR0L = BAUD_L;                        //设定波特率
    UBRR0H = BAUD_H;
    UCSR0A = 0x00;
    UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0); //0XD8  接收、发送使能， 开中断使能
}
/***************USART0初始化函数 end***************************/
/***************系统初始化函数 start ***********/
void init_cpu(void)
{
    EIMSK = 0x00; //屏蔽INT0~INT1的所有外部中断
    clr_bit(SFIOR, PUD); //设置输入上拉电阻有效

    DDRA = 0xff; //1是输出，0是输入
    PORTA = 0x00; //控制PINC与PINF通路切换

    DDRB = 0xff; //PINB控制继电器
    PORTB = 0x00;

    DDRC = 0xff;//PINC控制继电器
    PORTC = 0x00;

    DDRD  = 0x00; //PIND0/1为绝缘仪Fail/Pass输入检测,PIND2/3为产品到位微动开关检测,PIND4/5为按键输入,PIND6/7没有检测
    PORTD = 0x30;  //外接上拉，按键带上拉

    DDRE = 0xfe; //RXD0输入，且上拉有效
    PORTE = 0x01; //TXD0输出

    DDRF = 0xff; //PINF全是输出
    PORTF = 0x00;

    DDRG = 0x00;
    PORTG = 0x00; //PING0,1,2为气缸到位检测

    init_usart0();
    init_TIMER0_OVF();
    SEI();
    clr_bit(flag2, is_NGFlag2);

    flag1 = 0;
    flag2 = 0;
    flag3 = 0;
    flagerr = 0;

}
/***************系统初始化函数 end ***********/
//自动测试函数
void v362_auto_test(void)
{
    if(Has_Production() && isStarted == "false") //有产品，且光栅没有遮挡
    {
        isStarted = "true";
        while((flag1 && (1 << Usart0_RECVFlag1)) == 0)
        {
            send(start);
            delay_nms(200);
        }
        clr_bit(flag1, Usart0_RECVFlag1);
        if(rx_buffer[9] == 0x52) //-->上位机请求重新开始
        {
            isStarted = "false";
            return;
        }
        else if(rx_buffer[9] == 0x53) //-->上位机正确启动开始
        {
            open_limit_valve();//打开限位气缸
            delay_nms(500);
            if((PING & (1 << 0)) != 0)//限位气缸到位磁感开关
            {

                open_terminal_valve();//打开连接端子的气缸
                delay_nms(200);
                if((PING & (1 << 1)) != 0)//连接端子气缸到位磁感开关
                {
                    open_insu_test_left12();//左1,2脚绝缘测试
                    delay_nms(1000);
                    open_test();
                    if(Insulation_Pass())//PD0-->Fail,PD1-->Pass
                    {
                        close_insu_test_left12();//关闭1,2脚测试
                        send_step(1, 0x10, 0x10);

                    }
                    else if(Insulation_Fail())
                    {
                        close_insu_test_left12();//关闭1,2脚测试
                        send_step(1, 0x10, 0x11);
                        set_bit(flag2, is_NGFlag2);
                        goto error;
                    }
                    delay_nms(20);

                    open_insu_test_left23();//左2,3脚绝缘测试
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

                    open_insu_test_right12();//右1,2脚绝缘测试
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

                    open_insu_test_right23();//右2,3脚绝缘测试
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

                    open_13_8V();//打开测试电压13.8V,5A，测试一次电源电压
                    delay_nms(50);
                    opentestprd_one();//产品未压合时，测1脚电压即为电源电压
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
					
                    open_product_valve();//打开压合产品的气缸
                    delay_nms(1000);
                    if((PING & (1 << 2)) != 0)//压合产品气缸到位磁感开关
                    {
                        opentestprd_one();//测试1脚的电压降
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

                        opentestprd_two();//测试2脚的电压降
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


                        opentestprd_three();//测试3脚的电压降
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

                        close_13_8V();//关闭程控电源的输出
                        close_product_valve();//产品压合气缸回缩
                        delay_nms(500);
						//while((PING & (1 << 2)) != 0);
                        //if((PING & (1 << 2)) == 0)
                        //{
                            opentestprd_one();//测试断电后的电源电压
                            delay_nms(20);
							
							steps(9, 0x11, 0x10);
                            
                            delay_nms(1000);
                            if((flag1 & (1 << Usart0_RECVFlag1)) != 0)
                            {
                                clr_bit(flag1, Usart0_RECVFlag1);
                                if(rx_buffer[9] == 0x4f)
                                {
                                    close_terminal_valve();//测试完毕端子气缸回缩
									delay_nms(500);
									close_limit_valve();//测试完毕限位气缸回缩
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
//按键处理函数===============================================
void key_scan(void)
{
    if((flag1 && (1 << keyprq_flag1)) == 0)  //如果没有按键按下
    {
        if((PIND & (1 << key1)) == 0) //启动测试按键
        {
            key_now = 1;
        }
        /*
        else if((PIND & (1 << key2)) == 0) //急停按键
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
//按键处理函数===============================================
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
            set_bit(flag1, keyeff_flag1);  //按键有效
        }
        if((flag1 & (1 << keyeff_flag1)) != 0)
        {
            clr_bit(flag1, keyeff_flag1);
            switch(key_old)
            {
            case 1:  //启动测试按键按下
            {
                v362_auto_test();
            }
            break;
            /*
            case 2:  //急停按键按下
            {

            } break;//*/
            default:
                break;
            }
        }
    }
}
/***************主函数 start *******************/
void main(void)
{
    init_cpu();    //初始化CPU
	open_reset();
    send(reset);
    while(1)
    {
        key_scan();
        key_process();
        delay_nms(10);
    }
}