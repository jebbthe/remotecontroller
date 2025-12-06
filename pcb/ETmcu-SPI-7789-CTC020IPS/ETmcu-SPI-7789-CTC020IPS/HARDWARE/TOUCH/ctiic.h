#ifndef __MYCT_IIC_H
#define __MYCT_IIC_H
#include "sys.h"	    
 //////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//亦亞徽科技ETmcu开发板
//IIC电容触摸功能   
//官网:www.eya-display.com
//修改日期:2016/3/26
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广东省亦亞徽科技有限公司 2016-2026
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
   	   		   
//IO方向设置
#define CT_SDA_IN()  {GPIOA->CRL&=0XFFFFF0FF;GPIOA->CRL|=8<<4*2;}
#define CT_SDA_OUT() {GPIOA->CRL&=0XFFFFF0FF;GPIOA->CRL|=3<<4*2;}

//IO操作函数	 
#define CT_IIC_SCL    PAout(0) 			//SCL     
#define CT_IIC_SDA    PAout(2) 			//SDA	 
#define CT_READ_SDA   PAin(2)  			//输入SDA 

//IIC所有操作函数
void CT_IIC_Init(void);                	//初始化IIC的IO口				 
void CT_IIC_Start(void);				//发送IIC开始信号
void CT_IIC_Stop(void);	  				//发送IIC停止信号
void CT_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 CT_IIC_Read_Byte(unsigned char ack);	//IIC读取一个字节
u8 CT_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void CT_IIC_Ack(void);					//IIC发送ACK信号
void CT_IIC_NAck(void);					//IIC不发送ACK信号

#endif







