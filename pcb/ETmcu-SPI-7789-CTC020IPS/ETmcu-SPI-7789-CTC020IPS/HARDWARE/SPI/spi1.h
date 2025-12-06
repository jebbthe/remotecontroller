#ifndef __spi1_H
#define __spi1_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//亦亞徽科技ETmcu开发板
//SPI相关函数	   
//官网:www.eya-display.com
//修改日期:2016/3/26
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广东省亦亞徽科技有限公司 2016-2026
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define SPI1_DMA 1 //0  关闭  1开启   (usart3 dma 冲突)

void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8   SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
	
#if SPI1_DMA==1
u8 DMA1_Spi1_Init(void);
u8 DMA1_Spi1_RX(u8 *buffer,u32 len);
u8 DMA1_Spi1_TX(u8 *buffer,u32 len);
#endif

#endif

