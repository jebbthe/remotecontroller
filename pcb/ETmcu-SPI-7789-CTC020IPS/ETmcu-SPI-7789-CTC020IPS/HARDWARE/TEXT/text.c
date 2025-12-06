/**
 ****************************************************************************************************
 * @file        text.c
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
#include "sys.h" 
#include "fontupd.h"
#include "w25qxx.h"
#include "lcd.h"
#include "text.h"	
#include "string.h"			
#include "malloc.h"	
#include "updater.h"


#define F16addr ADD_BASE+0x00000000
#define F24addr ADD_BASE+0x000BB080
#define F32addr ADD_BASE+0x0025FDA0
/**
//获取汉字点阵数据
//code  : 当前汉字编码(GBK码)
//mat   : 当前汉字点阵数据存放地址
//size  : 字体大小
//size大小的字体,其点阵数据大小为: (size / 8 + ((size % 8) ? 1 : 0)) * (size)  字节
 */
static void text_get_hz_mat(unsigned char *code, unsigned char *mat, uint8_t size)
{
    unsigned char qh, ql;
    unsigned char i;
    unsigned long foffset;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size); /* 得到字体一个字符对应点阵集所占的字节数 */
    qh = *code;
    ql = *(++code);

    if (qh < 0x81 || ql < 0x40 || ql == 0xff || qh == 0xff)   /* 非 常用汉字 */
    {
        for (i = 0; i < csize; i++)
        {
            *mat++ = 0x00;  /* 填充满格 */
        }

        return;     /* 结束访问 */
    }

    if (ql < 0x7f)
    {
        ql -= 0x40; /* 注意! */
    }
    else
    {
        ql -= 0x41;
    }

    qh -= 0x81;
    foffset = ((unsigned long)190 * qh + ql) * csize;   /* 得到字库中的字节偏移量 */

    switch (size)
    {
        case 32:
            W25QXX_Read(mat, foffset + F32addr, csize);
            break;

        case 16:
            W25QXX_Read(mat, foffset + F16addr, csize);
            break;

        case 24:
            W25QXX_Read(mat, foffset + F24addr, csize);
            break;
    }
}
//显示一个指定大小的汉字
//x,y   : 汉字的坐标
//font  : 汉字GBK码
//size  : 字体大小
//mode  : 显示模式
//0, 正常显示(不需要显示的点,用LCD背景色填充,即g_back_color)
//1, 叠加显示(仅显示需要显示的点, 不需要显示的点, 不做处理)
//color : 字体颜色
//该函数会对内存进行分配管理，由于演示的demo CPU RAM不足，可能会存在bug
void text_show_font(uint16_t x, uint16_t y, uint8_t *font, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t temp, t, t1;
    uint16_t y0 = y;
    uint8_t *dzk;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size);     /* 得到字体一个字符对应点阵集所占的字节数 */

    if (size != 16 && size != 24 && size != 32)//需要其他字体自己添加
    {
        return;     /* 不支持的size */
    }

    dzk = malloc(size);       /* 申请内存 */

    if (dzk == 0) return;               /* 内存不够了 */

    text_get_hz_mat(font, dzk, size);   /* 得到相应大小的点阵数据 */

    for (t = 0; t < csize; t++)
    {
        temp = dzk[t];                  /* 得到点阵数据 */

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
            {
                LCD_DrawPoint(x, y, color);        /* 画需要显示的点 */
            }
            else if (mode == 0)     /* 如果非叠加模式, 不需要显示的点,用背景色填充 */
            {
                LCD_DrawPoint(x, y, BACK_COLOR);  /* 填充背景色 */
            }

            temp <<= 1;
            y++;

            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
    free(dzk);    /* 释放内存 */
}





//在指定位置开始显示一个字符串	    
//支持自动换行
//(x,y):起始坐标
//width,height:区域
//str  :字符串
//size :字体大小
//mode:0,非叠加方式;1,叠加方式   
//color : 字体颜色
void Show_Str(u16 x,u16 y,u16 width,u16 height,char*str,u8 size,uint8_t mode, uint16_t color)
{					
    u8 x0 = x;
    u8 y0 = y;
    uint8_t bHz = 0;                /* 字符或者中文 */
    uint8_t *pstr = (uint8_t *)str; /* 指向char*型字符串首地址 */
		width+=x;
	  height+=y;

    while (*pstr != 0)   /* 数据未结束 */
    {
        if (!bHz)
        {
            if (*pstr > 0x80)   /* 中文 */
            {
                bHz = 1;    /* 标记是中文 */
            }
            else            /* 字符 */
            {
							while((*pstr<='~')&&(*pstr>=' '))//判断是不是非法字符!
							{       
									if(x>=width){x=x0;y+=size;}
									if(y>=height)break;//退出
									LCD_ShowChar(x,y,*pstr,size,0,color);
									x+=size/2;
									pstr++;
							} 
            }
        }
        else     /* 中文 */
        {
            bHz = 0; /* 有汉字库 */

            if (x > (x0 + width - size))   /* 换行 */
            {
                y += size;
                x = x0;
            }

            if (y > (y0 + height - size))break; /* 越界返回 */

            text_show_font(x, y, pstr, size, mode, color); /* 显示这个汉字,空心显示 */
            pstr += 2;
            x += size; /* 下一个汉字偏移 */
        }
    }
} 			 		 
//在指定宽度的中间显示字符串
//如果字符长度超过了len,则用Show_Str显示
//len:指定要显示的宽度			  
void Show_Str_Mid(u16 x,u16 y,char*str,u8 size,u8 len,u16 color)
{
	u16 strlenth=0;
   	strlenth=strlen((const char*)str);
	strlenth*=size/2;
	if(strlenth>len)Show_Str(x,y,lcddev.width,lcddev.height,str,size,1,color);
	else
	{
		strlenth=(len-strlenth)/2;
	    Show_Str(strlenth+x,y,lcddev.width,lcddev.height,str,size,1,color);
	}
}   





















		  






