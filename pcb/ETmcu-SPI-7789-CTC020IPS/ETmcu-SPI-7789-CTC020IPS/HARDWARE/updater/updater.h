/**
 ****************************************************************************************************
 * @file        lcd.h
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
 #ifndef __UPDATER_H
#define  __UPDATER_H		
#include "sys.h"	 
#include "stdlib.h"

//标识符
#define UART_IDentifier      0x40  //标识符
void TIM3_Int_Init(u16 arr,u16 psc);
void NVIC_PriorityGroupConfigSet(void);
void Update(void);

extern u8  Contflag;//计数标志，用于测算刷图时间
extern u16 Counter; //用于计数，测算刷图时间

extern u32 ADD_BASE;

#endif  
	 
	 



