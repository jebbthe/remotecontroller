/**
 ****************************************************************************************************
 * @file        delay.h
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
#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"

void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























