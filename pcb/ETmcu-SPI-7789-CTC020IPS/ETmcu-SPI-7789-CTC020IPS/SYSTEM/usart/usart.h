/**
 ****************************************************************************************************
 * @file        usart.h
 * @author      EYA-DISPLAY
 * @version     V2.0
 * @date        2022-04-28
 * @brief       液晶屏驱动Demo
 * @license     Copyright (c) 2022-2032, 亦亞徽科技集团(广东)
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:EYA-ETmcu开发板
 * 公司网址:www.eya-display.com
 *
 ****************************************************************************************************
 **/
#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define Baudrate            256000
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			   1		//使能（1）/禁止（0）串口1接收

#define DMA_SIZE 2
extern u8 DMA_Flag;
extern u8 DMA_Buf_Flag1;
extern u8 DMA_Buf_Flag2;
extern u8 DMA_Buf1[] ;
extern u8 DMA_Buf2[];

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
#endif


