/**
 ****************************************************************************************************
 * @file        cst3xx.h
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
#ifndef __CST3XX_H
#define __CST3XX_H	
#include "sys.h"							  

//IO操作函数	 
#define CST_RST    		PCout(14)	  // 复位引脚
#define CST_INT    		PCin(15)		// 中断引脚	
   	
//cst328和3240差异在于IIC地址及ID号，其他寄存器地址完全一致
//I2C读写命令	
#define CST3xxIIC_AD  0x1A                        //设备地址
#define CST328_CMD_WR 		(CST3xxIIC_AD<<1)     	//写命令
#define CST328_CMD_RD 		(CST3xxIIC_AD<<1)+1		  //读命令

//I2C读写命令	
#define CST3240IIC_AD     0x5A                    //设备地址
#define CST3240_CMD_WR 		(CST3240IIC_AD<<1)     	//写命令
#define CST3240_CMD_RD 		(CST3240IIC_AD<<1)+1		//读命令
  
//CST3XX 部分寄存器定义 
#define CST_PID_REG     0xD204
#define CST_GSTID_REG   0xD005

#define CST_TP1_REG 		0XD001  	//第一个触摸点数据地址
#define CST_TP2_REG 		0XD008		//第二个触摸点数据地址
#define CST_TP3_REG 		0XD00D		//第三个触摸点数据地址
#define CST_TP4_REG 		0XD012		//第四个触摸点数据地址
#define CST_TP5_REG 		0XD017		//第五个触摸点数据地址  
 
extern u8 CST_IIC_WR,CST_IIC_RD;

u8 CST3XX_WR_Reg(u16 reg,u8 *buf,u8 len);
void CST3XX_RD_Reg(u16 reg,u8 *buf,u8 len); 
u8 CST3XX_Init(void);
u8 CST3XX_Scan(u8 mode); 
u8 Null_Scan(u8 i);
#endif













