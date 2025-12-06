/**
 ****************************************************************************************************
 * @file        cst3xx.c
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
#include "cst3xx.h"
#include "touch.h"
#include "ctiic.h"
#include "usart.h"
#include "delay.h" 
#include "string.h" 
#include "lcd.h" 

u8 CST_IIC_WR=CST328_CMD_WR,CST_IIC_RD=CST328_CMD_RD;//默认IIC地址是CST328

//向CST3XX写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
u8 CST3XX_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(CST_IIC_WR);   	//发送写命令 	 
	CT_IIC_Wait_Ack();
	CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	CT_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}
//从CST3XX读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void CST3XX_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(CST_IIC_WR);   //发送写命令 	 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(CST_IIC_RD);   //发送读命令		   
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    CT_IIC_Stop();//产生一个停止条件    
} 
//初始化CST3XX触摸屏
//返回值:0,初始化成功;1,初始化失败 
u8 CST3XX_Init(void)
{
	u8 temp[5]; 
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//使能PC端口时钟

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14|GPIO_Pin_15;// PC14和PC15端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);//输出1

 	CT_IIC_Init();  //初始化电容屏的I2C总线  
	CST_RST=0;				//复位
	delay_ms(10);
 	CST_RST=1;				//释放复位 
 	delay_ms(10);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;			//PC15端口配置
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;		//下拉输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);			  //PC15下拉输入
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);				    //下拉	

 	delay_ms(100);
	CST3XX_WR_Reg(0xD101,temp,4);
	CST3XX_RD_Reg(CST_PID_REG,temp,4);//读取产品ID
	if((temp[3]==0x01)&&(temp[2]==0x48)) return 0;
	
	CST_IIC_WR=CST3240_CMD_WR;
	CST_IIC_RD=CST3240_CMD_RD;
	CST3XX_WR_Reg(0xD101,temp,4);
	CST3XX_RD_Reg(CST_PID_REG,temp,4);//读取产品ID  读取到3240
	if((temp[3]==0x32)&&(temp[2]==0x40)) return 0;
	//printf("CTP ID:0x%x\r\n",temp[0]);	//打印ID 
	//printf("CTP ID:0x%x\r\n",temp[1]);
	//printf("CTP ID:0x%x\r\n",temp[2]);
	//printf("CTP ID:0x%x\r\n",temp[3]);
	return 1;
}
const u16 CST3XX_TPX_TBL[5]={CST_TP1_REG,CST_TP2_REG,CST_TP3_REG,CST_TP4_REG,CST_TP5_REG};
//扫描触摸屏(采用查询方式)
//mode:0,正常扫描.
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
u8 CST3XX_Scan(u8 mode)
{
	u8 buf[4];
	u8 i=0;
	u8 res=0;
	u8 temp;
	static u8 t=0;//控制查询间隔,从而降低CPU占用率   
	t++;
	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
	{
		CST3XX_RD_Reg(CST_GSTID_REG,&mode,1);//读取触摸点的状态 
		//printf("mode:0x%x\r\n",mode);
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);//将点的个数转换为1的位数,匹配tp_dev.sta定义 
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//触摸有效?
				{
					CST3XX_RD_Reg(CST3XX_TPX_TBL[i],buf,4);	//读取XY坐标值
					if(lcddev.dir==0)	//竖屏
					{
						tp_dev.x[i]=((u16)buf[0]<<4)+(((u16)buf[2]>>4)&0x0F);
						tp_dev.y[i]=(((u16)buf[1]<<4)+((u16)buf[2]&0x0f));
					}else //横屏
					{
						tp_dev.y[i]=((u16)buf[0]<<4)+(((u16)buf[2]>>4)&0x0F);
						tp_dev.x[i]=lcddev.width-(((u16)buf[3]<<8)+buf[2]);
					}  
					printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);//串口打印坐标，用于调试
				}			
			} 
			res=1;
			if(tp_dev.x[0]==0 && tp_dev.y[0]==0)mode=0;	//读到的数据都是0,则忽略此次数据
			t=0;		//触发一次,则会最少连续监测10次,从而提高命中率
		}
 		if((mode&0XF)<6)
		{
			temp=0;
			CST3XX_WR_Reg(CST_GSTID_REG,&temp,1);//清标志 		
		}
	}
	if((mode&0X0F)==0X00)//无触摸点按下
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)	//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);	//标记按键松开
		}else						//之前就没有被按下
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//清除点有效标记	
		}	 
	} 	
	if(t>240)t=10;//重新从10开始计数
	return res;
}
 

























