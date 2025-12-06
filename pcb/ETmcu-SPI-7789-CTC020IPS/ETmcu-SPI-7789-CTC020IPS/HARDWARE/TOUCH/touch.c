/**
 ****************************************************************************************************
 * @file        touch.c
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
#include "touch.h" 
#include "lcd.h"
#include "delay.h"
#include "stdlib.h"
#include "math.h"
#include "GT911.h" 
#include "CST3XX.h" 
#include "usart.h"
#include "text.h"	
#include "w25qxx.h"

extern u8 AUTO;
u16 x0temp,y0temp;
//触摸相关参数
_m_tp_dev tp_dev=
{
	TP_Init,
	TP_Scan,
	TP_Adjust,
	0,
	0, 
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 		
};					
//默认为touchtype=0的数据.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
 	 			    					   
//SPI写数据
//向触摸屏IC写入1byte数据    
//num:要写入的数据
void TP_Write_Byte(u8 num)    
{  
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)TDIN=1;  
		else TDIN=0;   
		num<<=1;    
		TCLK=0; 
		delay_us(1);
		TCLK=1;		//上升沿有效	        
	}		 			    
} 		 
//SPI读数据 
//从触摸屏IC读取adc值
//CMD:指令
//返回值:读到的数据	   
u16 TP_Read_AD(u8 CMD)	  
{ 	 
	u8 count=0; 	  
	u16 Num=0; 
	TCLK=0;		//先拉低时钟 	 
	TDIN=0; 	//拉低数据线
	TCS=0; 		//选中触摸屏IC
	TP_Write_Byte(CMD);//发送命令字
	delay_us(6);//ADS7846的转换时间最长为6us
	TCLK=0; 	     	    
	delay_us(1);    	   
	TCLK=1;		//给1个时钟，清除BUSY
	delay_us(1);    
	TCLK=0; 	     	    
	for(count=0;count<16;count++)//读出16位数据,只有高12位有效 
	{ 				  
		Num<<=1; 	 
		TCLK=0;	//下降沿有效  	    	   
		delay_us(1);    
 		TCLK=1;
 		if(DOUT)Num++; 		 
	}  	
	Num>>=4;   	//只有高12位有效.
	TCS=1;		//释放片选	 
	return(Num);   
}
//读取一个坐标值(x或者y)
//连续读取READ_TIMES次数据,对这些数据升序排列,
//然后去掉最低和最高LOST_VAL个数,取平均值 
//xy:指令（CMD_RDX/CMD_RDY）
//返回值:读到的数据
#define READ_TIMES 5 	//读取次数
#define LOST_VAL 1	  	//丢弃值
u16 TP_Read_XOY(u8 xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)//排序
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//升序排列
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 
//读取x,y坐标
//最小值不能少于100.
//x,y:读取到的坐标值
//返回值:0,失败;1,成功。
u8 TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  												   
	//if(xtemp<100||ytemp<100)return 0;//读数失败
	x0temp=xtemp;
	y0temp=ytemp;//把值传递到前端用于判定是否有带触摸
	
	*x=xtemp;
	*y=ytemp;
	return 1;//读数成功
}
//连续2次读取触摸屏IC,且这两次的偏差不能超过
//ERR_RANGE,满足条件,则认为读数正确,否则读数错误.	   
//该函数能大大提高准确度
//x,y:读取到的坐标值
//返回值:0,失败;1,成功。
#define ERR_RANGE 50 //误差范围 
u8 TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//前后两次采样在+-50内
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
}  
//////////////////////////////////////////////////////////////////////////////////		  
//与LCD部分有关的函数  
//画一个触摸点
//用来校准用的
//x,y:坐标
//color:颜色
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y,color);//横线
	LCD_DrawLine(x,y-12,x,y+13,color);//竖线
	LCD_DrawPoint(x+1,y+1,color);
	LCD_DrawPoint(x-1,y+1,color);
	LCD_DrawPoint(x+1,y-1,color);
	LCD_DrawPoint(x-1,y-1,color);
	Draw_Circle(x,y,6,color);//画中心圈
}	  
//画一个大点(2*2的点)		   
//x,y:坐标
//color:颜色
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y,color);//中心点 
	LCD_DrawPoint(x+1,y,color);
	LCD_DrawPoint(x,y+1,color);
	LCD_DrawPoint(x+1,y+1,color);	 	  	
}						  
//////////////////////////////////////////////////////////////////////////////////		  
//触摸按键扫描
//tp:0,屏幕坐标;1,物理坐标(校准等特殊场合用)
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
u8 TP_Scan(u8 tp)
{			   
	if(PEN==0)//有按键按下
	{
		if(tp)TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]);//读取物理坐标
		else if(TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]))//读取屏幕坐标
		{
	 		tp_dev.x[0]=tp_dev.xfac*tp_dev.x[0]+tp_dev.xoff;//将结果转换为屏幕坐标
			tp_dev.y[0]=tp_dev.yfac*tp_dev.y[0]+tp_dev.yoff;  
	 	} 
		if((tp_dev.sta&TP_PRES_DOWN)==0)//之前没有被按下
		{		 
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//按键按下  
			tp_dev.x[4]=tp_dev.x[0];//记录第一次按下时的坐标
			tp_dev.y[4]=tp_dev.y[0];  	   			 
		}
		
		printf("x:%d\r\n",tp_dev.x[0]);	
		printf("y:%d\r\n",tp_dev.y[0]);	  //调试TP坐标时使用
				   
	}else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);//标记按键松开	
		}else//之前就没有被按下
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}	    
	}
	return tp_dev.sta&TP_PRES_DOWN;//返回当前的触屏状态
}	  
//////////////////////////////////////////////////////////////////////////	 
//保存在EEPROM里面的地址区间基址,占用13个字节(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE (uint32_t )0xA10000 //Flash 触摸校准数据的存放 地址不要动，否者会影响到其他字库或者图片
//保存校准参数										    
void TP_Save_Adjdata(void)
{
	u8 tbuff[4];
	s32 temp;			 
	//保存校正结果!		   							  
	temp=tp_dev.xfac*100000000;//保存x校正因素  
	//存储temp在外部flash中
  tbuff[0]=temp>>24;
	tbuff[1]=temp>>16;	
	tbuff[2]=temp>>8;	
	tbuff[3]=temp;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE,4);
  
	temp=tp_dev.yfac*100000000;//保存y校正因素  
  //存储temp在外部flash中  
  tbuff[0]=temp>>24;
	tbuff[1]=temp>>16;	
	tbuff[2]=temp>>8;	
	tbuff[3]=temp;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+4,4);
	//保存x偏移量
	tbuff[0]=tp_dev.xoff>>8;	
	tbuff[1]=tp_dev.xoff;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+8,2);	    
	//保存y偏移量
	tbuff[0]=tp_dev.yoff>>8;	
	tbuff[1]=tp_dev.yoff;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+10,2);
	//保存触屏类型
	tbuff[0]=tp_dev.touchtype;	
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+12,1);
	
	tbuff[0]=0X0A;	//标记校准过了
  W25QXX_Write((u8*)tbuff,SAVE_ADDR_BASE+13,1);
}
//得到保存在EEPROM里面的校准值
//返回值：1，成功获取数据
//        0，获取失败，要重新校准
u8 TP_Get_Adjdata(void)
{		
  u8 tbuff[4];	
	s32 tempfac;
  W25QXX_Read(tbuff,SAVE_ADDR_BASE+13,1);//读取标记字,看是否校准过！ 
  tempfac=tbuff[0];
	printf("tempfac=%x \r\n",tempfac);
	
	if(tempfac==0X0A)//触摸屏已经校准过了			   
	{ 
    W25QXX_Read(tbuff,SAVE_ADDR_BASE,4);
    tempfac =tbuff[0]; tempfac<<=8;
		tempfac|=tbuff[1]; tempfac<<=8;
		tempfac|=tbuff[2]; tempfac<<=8;
		tempfac|=tbuff[3];		   
		tp_dev.xfac=(float)tempfac/100000000;//得到x校准参数
		
		W25QXX_Read(tbuff,SAVE_ADDR_BASE+4,4);
    tempfac =tbuff[0]; tempfac<<=8;
		tempfac|=tbuff[1]; tempfac<<=8;
		tempfac|=tbuff[2]; tempfac<<=8;
		tempfac|=tbuff[3];			          
		tp_dev.yfac=(float)tempfac/100000000;//得到y校准参数
	    //得到x偏移量
		
		W25QXX_Read(tbuff,SAVE_ADDR_BASE+8,2);
    tp_dev.xoff =tbuff[0]; tp_dev.xoff<<=8;
		tp_dev.xoff|=tbuff[1];  		   	  
 	    //得到y偏移量
		
		W25QXX_Read(tbuff,SAVE_ADDR_BASE+10,2);
    tp_dev.yoff =tbuff[0]; tp_dev.yoff<<=8;
		tp_dev.yoff|=tbuff[1];
			
    W25QXX_Read(tbuff,SAVE_ADDR_BASE+12,1);//读取触屏类型标记
    tp_dev.touchtype =tbuff[0];
	
		if(tp_dev.touchtype)//X,Y方向与屏幕相反
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	 
		}else				   //X,Y方向与屏幕相同
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
}	 
//提示字符串
u8* const TP_REMIND_MSG_TBL="Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.";
 					  
//提示校准结果(各个参数)
void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac,u16 color)
{	  
	POINT_COLOR=RED;
	LCD_ShowString(40,160,lcddev.width,lcddev.height,16,"x1:",color);
 	LCD_ShowString(40+80,160,lcddev.width,lcddev.height,16,"y1:",color);
 	LCD_ShowString(40,180,lcddev.width,lcddev.height,16,"x2:",color);
 	LCD_ShowString(40+80,180,lcddev.width,lcddev.height,16,"y2:",color);
	LCD_ShowString(40,200,lcddev.width,lcddev.height,16,"x3:",color);
 	LCD_ShowString(40+80,200,lcddev.width,lcddev.height,16,"y3:",color);
	LCD_ShowString(40,220,lcddev.width,lcddev.height,16,"x4:",color);
 	LCD_ShowString(40+80,220,lcddev.width,lcddev.height,16,"y4:",color);  
 	LCD_ShowString(40,240,lcddev.width,lcddev.height,16,"fac is:",color);     
	LCD_ShowNum(40+24,160,x0,4,16,color);		//显示数值
	LCD_ShowNum(40+24+80,160,y0,4,16,color);	//显示数值
	LCD_ShowNum(40+24,180,x1,4,16,color);		//显示数值
	LCD_ShowNum(40+24+80,180,y1,4,16,color);	//显示数值
	LCD_ShowNum(40+24,200,x2,4,16,color);		//显示数值
	LCD_ShowNum(40+24+80,200,y2,4,16,color);	//显示数值
	LCD_ShowNum(40+24,220,x3,4,16,color);		//显示数值
	LCD_ShowNum(40+24+80,220,y3,4,16,color);	//显示数值
 	LCD_ShowNum(40+56,240,fac,3,16,color); 	//显示数值,该数值必须在95~105范围之内.

}
		 
//触摸屏校准代码
//得到四个校准参数
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//坐标缓存值
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	double fac; 	
	u16 outtime=0;
 	cnt=0;				
	POINT_COLOR=BLUE;
	BACK_COLOR =WHITE;
	LCD_Clear(WHITE);//清屏   
	POINT_COLOR=RED;//红色 
	LCD_Clear(WHITE);//清屏 	   
	POINT_COLOR=BLACK;
	LCD_ShowString(40,40,160,100,16,(u8*)TP_REMIND_MSG_TBL,POINT_COLOR);//显示提示信息
	TP_Drow_Touch_Point(20,20,RED);//画点1 
	tp_dev.sta=0;//消除触发信号 
	tp_dev.xfac=0;//xfac用来标记是否校准过,所以校准之前必须清掉!以免错误	 
	while(1)//如果连续10秒钟没有按下,则自动退出
	{
		tp_dev.scan(1);//扫描物理坐标
		if((tp_dev.sta&0xc0)==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{	
			outtime=0;		
			tp_dev.sta&=~(1<<6);//标记按键已经被处理过了.
						   			   
			pos_temp[cnt][0]=tp_dev.x[0];
			pos_temp[cnt][1]=tp_dev.y[0];
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//清除点1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//画点2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//清除点2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//画点3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//清除点3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//画点4
					break;
				case 4:	 //全部四个点已经得到
	    		    //对边相等
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,2的距离
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到3,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//不合格
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100,RED);//显示数据   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,3的距离
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100,RED);//显示数据   
						continue;
					}//正确了
								   
					//对角线相等
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,4的距离
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,3的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100,RED);//显示数据   
						continue;
					}//正确了
					//计算结果
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//得到xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//得到xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//得到yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//得到yoff  
					if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//触屏和预设的相反了.
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
						LCD_ShowString(40,26,lcddev.width,lcddev.height,16,"TP Need readjust!",POINT_COLOR);
						tp_dev.touchtype=!tp_dev.touchtype;//修改触屏类型.
						if(tp_dev.touchtype)//X,Y方向与屏幕相反
						{
							CMD_RDX=0X90;
							CMD_RDY=0XD0;	 
						}else				   //X,Y方向与屏幕相同
						{
							CMD_RDX=0XD0;
							CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					POINT_COLOR=BLUE;
					LCD_Clear(WHITE);//清屏
					LCD_ShowString(35,110,lcddev.width,lcddev.height,16,"Touch Screen Adjust OK!",POINT_COLOR);//校正完成
					
					printf("tp_dev.xfac:%f\r\n",tp_dev.xfac);
					printf("tp_dev.yfac:%f\r\n",tp_dev.yfac);
					printf("tp_dev.xoff:%d\r\n",tp_dev.xoff);
					printf("tp_dev.yoff:%d\r\n",tp_dev.yoff);
					
					delay_ms(1000);
					TP_Save_Adjdata();  
 					LCD_Clear(WHITE);//清屏   
					return;//校正完成				 
			}
		}
		delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			TP_Get_Adjdata();
			break;
	 	} 
 	}
}	 
//触摸屏初始化  		    
//返回值:1,没有进行校准
//       1,进行过校准
u8 TP_Init(void)
{	
	if(CST3XX_Init()==0)
	{
		tp_dev.scan=CST3XX_Scan;  
		tp_dev.touchtype |= 0X80;       /* 标记电容屏 */
		return 1;
	}
	else if(GT911_Init()==0)
	{
		tp_dev.scan=GT911_Scan;       	//扫描函数指向GT911触摸屏扫描
		tp_dev.touchtype |= 0X80;       /* 标记电容屏 */
		return 1;
	}else
	{ 
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA C
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;// PC1端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_SetBits(GPIOC,GPIO_Pin_14);//输出1
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;// PA0/1/2 端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);//输出1
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;			  //PA1端口配置
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;		//上拉输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);			  //PA1上拉输入
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);				      //上拉
		

		TP_Read_XY(&tp_dev.x[0],&tp_dev.y[0]);//第一次读取初始化	 
		//printf("xtemp:%d\r\n",x0temp);	
	  //printf("ytemp:%d\r\n",y0temp);	
	  if(x0temp==0&&y0temp==0)
    {
		   AUTO=1;
			 tp_dev.scan=Null_Scan; 
			 return 1;	 
		 }//不存在电阻触摸芯片，自动刷图
		
		W25QXX_Init();//初始化25QXX
		if(TP_Get_Adjdata())return 1;//已经校准
		else			        //未校准?
		{ 
			LCD_Clear(WHITE);//清屏
			TP_Adjust();  	 //屏幕校准 
			TP_Save_Adjdata();	 
		}			
		TP_Get_Adjdata();	
		
		return 0; 									 
	}
}

//判断触点是不是在指定区域之内
//(x,y):起始坐标
//xlen,ylen:在x,y方向上的偏移长度
//返回值 :1,在该区域内.0,不在该区域内.
u8 Is_In_Area(u16 x,u16 y,u16 xlen,u16 ylen)
{
	tp_dev.scan(0);//扫描
	if(tp_dev.sta&TP_PRES_DOWN)//有按键被按下
		{
			delay_ms(20);//必要的延时,否则老认为有按键按下.
			if(tp_dev.x[0]<=(x+xlen)&&tp_dev.x[0]>=x&&tp_dev.y[0]<=(y+ylen)&&tp_dev.y[0]>=y)
			{
				return 1;
			}
			else return 0;
		}
	 return 0;
} 


//坐标地址坐标
u16 PositionH[4*2]={
0,LCD_WIDTH-40,40,40,     //第一个按键位置： 起始位置X，起始位置Y，宽度，长度
LCD_HEIGHT-40,LCD_WIDTH-40,40,40,    //第二个按键位置： 起始位置X，起始位置Y，宽度，长度
};
u16 PositionV[4*2]={
0,LCD_HEIGHT-40,40,40,    //第一个按键位置： 起始位置X，起始位置Y，宽度，长度
LCD_WIDTH-40,LCD_HEIGHT-40,40,40,    //第二个按键位置： 起始位置X，起始位置Y，宽度，长度
};

u16 touchcount=0;
u8 lock=0;
//触摸位置扫描
u8 py_get_keynum(void)
{
	u8 KeyNO;
	u16 X,Y,WIDTH,HEIGHT;
	u8 i;
	static u8 key_x=0;//0,没有任何按键按下;1~9,1~9号按键按下
	u8 key=0;
	u16 *Pos;//按键坐标
	tp_dev.scan(0); 		 
	if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
	{	
		 if(lcddev.dir==0) Pos = PositionV; //竖屏
		 else              Pos = PositionH; //横屏 
		 KeyNO=2;//只有两个按键
		//扫描按键位置
		for (i = 0; i < KeyNO; i++)
		{
				X = Pos[i*4 + 0];
				Y = Pos[i*4 + 1];
			  WIDTH = Pos[i*4 + 2]; 
			  HEIGHT = Pos[i*4 + 3]; //获取长宽坐标
			
				if(tp_dev.x[0]<(X+WIDTH)&&tp_dev.x[0]>(X)&&tp_dev.y[0]<(Y+HEIGHT)&&tp_dev.y[0]>(Y)) { key=i+1; break;}//找到按键位置，跳出循环
		 }	
			if(key) //检查值的正确性
			{	  
          if((touchcount++>500)&&(key_x!=0))//长按了	
					{
						key_x=key;
					}else//短按
          {					
						if(key_x==key)key=0;
						else          key_x=key;
					}
			}
		}else if(key_x) //弹起 改用LOCK作为判断弹起的坐标，防止移动改变坐标
	  {
			  touchcount=0;
		    key_x=0;
		    lock=0;//释放按键锁
 
	  } 
	 return key;
}


//清空屏幕并在右上角显示"RST"
void Load_Drow_Dialog(void)
{
	LCD_Clear(WHITE);	//清屏   
	LCD_ShowString(lcddev.width-24,0,200,16,16,"RST",BLUE);//显示清屏区域
	Show_Str(0,lcddev.height-20,50,16,"返回",16,1,BLUE);
}

//显示返回和继续
void ShowUI(u8 i)
{
	if(i>1) Show_Str(0,lcddev.height-20,50,16,"返回",16,1,POINT_COLOR); //不在第一个画面才需要返回
	        Show_Str(lcddev.width-40,lcddev.height-20,50,16,"继续",16,1,POINT_COLOR);
}

////////////////////////////////////////////////////////////////////////////////
//电容触摸屏专有部分
//画水平线
//x0,y0:坐标
//len:线长度
//color:颜色
void gui_draw_hline(u16 x0,u16 y0,u16 len,u16 color)
{
	if(len==0)return;
	LCD_Fill(x0,y0,x0+len-1,y0,color);	
}
//画实心圆
//x0,y0:坐标
//r:半径
//color:颜色
void gui_fill_circle(u16 x0,u16 y0,u16 r,u16 color)
{											  
	u32 i;
	u32 imax = ((u32)r*707)/1000+1;
	u32 sqmax = (u32)r*(u32)r+(u32)r/2;
	u32 x=r;
	gui_draw_hline(x0-r,y0,2*r,color);
	for (i=1;i<=imax;i++) 
	{
		if ((i*i+x*x)>sqmax)// draw lines from outside  
		{
 			if (x>imax) 
			{
				gui_draw_hline (x0-i+1,y0+x,2*(i-1),color);
				gui_draw_hline (x0-i+1,y0-x,2*(i-1),color);
			}
			x--;
		}
		// draw lines from inside (center)  
		gui_draw_hline(x0-x,y0+i,2*x,color);
		gui_draw_hline(x0-x,y0-i,2*x,color);
	}
}  
//两个数之差的绝对值 
//x1,x2：需取差值的两个数
//返回值：|x1-x2|
u16 my_abs(u16 x1,u16 x2)
{			 
	if(x1>x2)return x1-x2;
	else return x2-x1;
}  
//画一条粗线
//(x1,y1),(x2,y2):线条的起始坐标
//size：线条的粗细程度
//color：线条的颜色
void lcd_draw_bline(u16 x1, u16 y1, u16 x2, u16 y2,u8 size,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	if(x1<size|| x2<size||y1<size|| y2<size)return; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		gui_fill_circle(uRow,uCol,size,color);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
} 
//5个触控点的颜色												 
const u16 POINT_COLOR_TBL[CT_MAX_TOUCH]={RED,GREEN,BLUE,BROWN,GRED};  
//电容触摸屏测试函数
void ctp_test(void)
{
	u8 t=0;
	u8 i=0;	  	    
 	u16 lastpos[5][2];		//记录最后一次的数据 
	lastpos[0][0]=0;
	lastpos[0][1]=0;
	while(1)
	{
		tp_dev.scan(0);    //从扫描函数取X Y坐标 
		for(t=0;t<CT_MAX_TOUCH;t++)
		{
			if((tp_dev.sta)&(1<<t))
			{
				if(tp_dev.x[t]<lcddev.width&&tp_dev.y[t]<lcddev.height)
				{
					printf("x:%d\r\n",tp_dev.x[0]);	
					printf("y:%d\r\n",tp_dev.y[0]);	 
					
					if(lastpos[t][0]==0XFFFF)
					{
						lastpos[t][0] = tp_dev.x[t];
						lastpos[t][1] = tp_dev.y[t];
					}
					lcd_draw_bline(lastpos[t][0],lastpos[t][1],tp_dev.x[t],tp_dev.y[t],2,POINT_COLOR_TBL[t]);//画线
					lastpos[t][0]=tp_dev.x[t];
					lastpos[t][1]=tp_dev.y[t];
					if(tp_dev.x[t]>(lcddev.width-40)&&tp_dev.y[t]<40)
					{
						Load_Drow_Dialog();//清除
					}else if(tp_dev.x[t]<(0+24)&&tp_dev.x[t]>(0)&&tp_dev.y[t]<(lcddev.height)&&tp_dev.y[t]>(lcddev.height-24))//左下角
					{
						LCD_Clear(WHITE);
						ShowUI(1);
						return; //返回
					}
				}
			}else lastpos[t][0]=0XFFFF;	
		}
		delay_ms(5);i++;
	}	
}

//电阻触摸屏测试函数
void rtp_test(void)
{
	while(1)
	{
		tp_dev.scan(0); 		 
		if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
		{	
		 	if(tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{	
				if(tp_dev.x[0]>(lcddev.width-24)&&tp_dev.y[0]<16)Load_Drow_Dialog();//清除
				else if(tp_dev.x[0]<(0+24)&&tp_dev.x[0]>(0)&&tp_dev.y[0]<(lcddev.height)&&tp_dev.y[0]>(lcddev.height-24))//左下角
				{
						LCD_Clear(WHITE);
						ShowUI(1);
						return; //返回
				}
				else TP_Draw_Big_Point(tp_dev.x[0],tp_dev.y[0],RED);		//画图	  			   
			}
		}else delay_ms(10);	//没有按键按下的时候 	    
	}
}
