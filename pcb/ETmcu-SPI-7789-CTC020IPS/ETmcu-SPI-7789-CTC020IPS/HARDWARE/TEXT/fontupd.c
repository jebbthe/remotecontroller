/**
 ****************************************************************************************************
 * @file        fontupd.c
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
#include "fontupd.h"
//#include "ff.h"	  
#include "w25qxx.h"
#include "lcd.h"  
//#include "malloc.h"
					   
u32 FONTINFOADDR=(1024*6+500)*1024;//默认是6M的地址
//字库信息结构体. 
//用来保存字库基本信息，地址，大小等
_font_info ftinfo;

//在sd卡中的路径
const u8 *GBK16_SDPATH="0:/SYSTEM/FONT/GBK16.FON";		//GBK16的存放位置
const u8 *GBK12_SDPATH="0:/SYSTEM/FONT/GBK12.FON";		//GBK12的存放位置
const u8 *UNIGBK_SDPATH="0:/SYSTEM/FONT/UNIGBK.BIN";	//UNIGBK.BIN的存放位置
//在25Qxx中的路径
const u8 *GBK16_25QPATH="1:/SYSTEM/FONT/GBK16.FON";		//GBK16的存放位置
const u8 *GBK12_25QPATH="1:/SYSTEM/FONT/GBK12.FON";		//GBK12的存放位置
const u8 *UNIGBK_25QPATH="1:/SYSTEM/FONT/UNIGBK.BIN";	//UNIGBK.BIN的存放位置

//显示当前字体更新进度
//x,y:坐标
//size:字体大小
//fsize:整个文件大小
//pos:当前文件指针位置
u32 fupd_prog(u16 x,u16 y,u8 size,u32 fsize,u32 pos)
{
	float prog;
	u8 t=0XFF;
	prog=(float)pos/fsize;
	prog*=10000;
	if(t!=prog)
	{
		t=prog/100;
		if(t>100)	t=100;
		LCD_ShowNum(x,y,t,2,size);//显示数值
		t=0;		//清空數據
		t=(u16)prog%100;
		if(t != 0)
		{
			if(t<10)
			{
				LCD_ShowNum(x+3*size/2,y,0,1,size);//显示数值
				LCD_ShowNum(x+4*size/2,y,t,1,size);//显示数值
			}else
			{
			LCD_ShowNum(x+3*size/2,y,t,2,size);//显示数值
			}
			LCD_ShowString(x+2*size/2,y,240,320,size,".");		
			LCD_ShowString(x+5*size/2,y,240,320,size,"%");		
		}
	}
	return 0;					    
} 

//更新字体文件,UNIGBK,GBK12,GBK16一起更新
//x,y:提示信息的显示地址
//size:字体大小
//提示信息字体大小
//src:0,从SD卡更新.
//	  1,从25QXX更新											  
//返回值:0,更新成功;
//		 其他,错误代码.	  
//初始化字体
//返回值:0,字库完好.
//		 其他,字库丢失
u8 font_init(void)
{			  												 
	W25QXX_Init();
	FONTINFOADDR=0X00;			//W25Q64,6M以后	 
	ftinfo.f16addr=FONTINFOADDR;		//UNICODEGBK 表存放首地址固定地址
	W25QXX_Read((u8*)&ftinfo,FONTINFOADDR,sizeof(ftinfo));//读出ftinfo结构体数据
	//if(ftinfo.fontok!=0XAA)return 1;		//字库错误. 
	return 0;		    
}





























