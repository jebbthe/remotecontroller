/**
 ****************************************************************************************************
 * @file        main.c
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
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "touch.h" 
#include "text.h" 
#include "w25qxx.h"
#include "updater.h"
/*  如果需要更换成别的CPU，只需要定义好IO口，其他程序可以照搬使用
*/

u8 AUTO=0;//如果不带触摸，想要一直刷屏，把这个参数设置为1
u8 Olddir;//用于记录屏幕原始的方向
u8  Contflag=0;//计数标志，用于测算刷图时间
u16 Counter=0; //用于计数，测算刷图时间

//外部flash地址定义
// 16*16汉字GBK 从0x00000000 -0x000BB080
// 24*24汉字GBK 从0x000BB080 -0x001A4D20
// 32*32汉字GBK 从0x0025FDA00-0x002EC200
// 128x128图片  从 0x0054BFA0           开始存放  每张图片大小 128*128*2字节  有5张图片
// 160x128图片  从 0x0054BFA0+0x00028000开始存放  每张图片大小 160*128*2字节  有5张图片   即偏移地址0x00028000
// 320x240图片  从 0x0054BFA0+0x0005A000开始存放  每张图片大小 320*240*2字节  有5张图片   即偏移地址0x0005A000
// 480x272图片  从 0x0054BFA0+0x00115800开始存放  每张图片大小 480*272*2字节  有5张图片   即偏移地址0x00115800
// 480x320图片  从 0x0054BFA0+0x00253C80开始存放  每张图片大小 480*320*2字节  有5张图片   即偏移地址0x00253C80

//图片存放地址 
#define pic_ad  ADD_BASE+0x0054BFA0 //这个地址不需要改动
//根据lcd.h中配置的分辨率选择对应的图片起始地址
#if((LCD_WIDTH==128)&&(LCD_HEIGHT==128))
#define off_ad  0x0
#elif((LCD_WIDTH==128)&&(LCD_HEIGHT==160))
#define off_ad  0x00028000
#elif((LCD_WIDTH==240)&&(LCD_HEIGHT==320))
#define off_ad  0x0005A000
#elif((LCD_WIDTH==480)&&(LCD_HEIGHT==272))
#define off_ad  0x00115800
#elif((LCD_WIDTH==320)&&(LCD_HEIGHT==480))
#define off_ad  0x00253C80
#endif
//图片取模方式为横屏取模，如果要正常显示图片，需要切换成横屏显示!!!

//定义按键
#define KEYup   1
#define KEYdown 2

#define DelayTime 1000000
//以下为主函数入口
 int main(void)
 {
	u32 loop=0;               //用作不带触摸的屏幕，自动刷屏跑画面
	u8 ix=0;                  //循环使用的变量
	u8 Keypad;                //触摸位置缓存
  u8 buffpic[LCD_WIDTH*2];  //图片数组
	delay_init();	    	      //延时函数初始化	  
	NVIC_Configuration(); 	  //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(Baudrate);	 	  //串口初始化为115200  可以通过串口打印一些数据作为调试参考
	W25QXX_Init();            //初始化外部NorFLASH
	LCD_Init();               //液晶屏初始化  并显示logo  LOGO文件通过取模放在 IMAGE1.C中，在LCD初始化完毕之后显示LOGO，LOGO显示完毕之后开背光
	TIM3_Int_Init(49,7199);   //10Khz的计数频率，计数到50为5ms  
	TIM2_PWM_Init(899,16);    //初始化背光控制PWM  //8分频。PWM频率=72000/(899+1)/16=2Khz 
	//Update();               //如果需要更新FALSH，则放开这个函数，目前只支持全部更新
	//背光由暗到亮变化
	for(ix=0;ix<10;ix++)
	{
		 SendPWMval(ix);         //亮度设置0-10
		 delay_ms(250);         //延时观看亮度变化，可以参考这个功能设置背光亮度
	}
	delay_ms(2000); 
	tp_dev.init();		        //触摸屏初始化
	//TP_Adjust();            //如果跳过电阻触摸校准画面，则放开这个函数，编译下载校准即可。  电阻触摸进行横竖屏切换之后，也要放开这个函数，重新校准触摸
//////////////////////////////////////////以下开始LCD功能测试 开始///////////////////////////////////////	
	ix=0;
	LCD_Clear(WHITE);//清屏
	Show_Str((lcddev.width-32*7)/2,10,260,32,"亦亞徽科技集团",32,1,RED); //X方向居中显示32*32汉字 
	Show_Str(30,60,600,24,"串口屏",24,1,BLACK);                          //显示24*24汉字
	Show_Str(30,60+24*1,600,24,"液晶显示屏",24,1,BLACK);
	Show_Str(30,60+24*2,600,24,"液晶显示模块",24,1,BLACK);
	Show_Str(30,60+24*3,600,24,"智能显示终端",24,1,BLACK);
	Show_Str(30,60+24*4,600,24,"显示解决方案",24,1,BLACK);
	Show_Str(lcddev.width-40,lcddev.height-20,50,16,"继续",16,1,BLUE);  //显示16*16汉字
	while(1)
	{
		if(AUTO==1)loop++; //如果没有触摸，则用这个参数实现循环播放图片
		if(((Keypad=py_get_keynum())>0)||(loop>DelayTime))//按键位置扫描，并处理功能
		{
			if(loop>DelayTime) { Keypad=KEYdown;loop=0;}
	
			     if(Keypad==KEYup)   {if(ix>0)  ix--;} //按了向上的按键
			else if(Keypad==KEYdown) {if(ix<12) ix++;}//按了向下的按键
			
			if(ix>12)ix=0;
			
			Contflag=1; //开始计时
			switch(ix)//方向转换
			{
				case 1:LCD_Clear(RED);   POINT_COLOR=WHITE;	Show_Str(10,10,200,16,"三基色显示（红色）",16,1,GREEN);                  break;
				case 2:LCD_Clear(GREEN); POINT_COLOR=BLACK;	Show_Str(10,10,200,16,"三基色显示（绿色）",16,1,RED);                    break;
				case 3:LCD_Clear(BLUE);  POINT_COLOR=RED;   Show_Str(10,10,200,16,"三基色显示（蓝色）",16,1,RED);                    break;
				case 4:LCD_Clear(BLACK); POINT_COLOR=WHITE; Show_Str(10,10,200,16,"黑色",16,1,RED);                								   break;
				case 5:LCD_Clear(WHITE); POINT_COLOR=BLACK; Show_Str(10,10,200,16,"白色",16,1,RED);      Olddir=lcddev.dir;          break;
				case 6:sendicon(0,0,lcddev.width,lcddev.height,(pic_ad+off_ad)+(lcddev.width*lcddev.height)*2*0, lcddev.width,lcddev.height*2, buffpic);
				       break;
				case 7:sendicon(0,0,lcddev.width,lcddev.height,(pic_ad+off_ad)+(lcddev.width*lcddev.height)*2*1, lcddev.width,lcddev.height*2, buffpic);
				       break;
				case 8:sendicon(0,0,lcddev.width,lcddev.height,(pic_ad+off_ad)+(lcddev.width*lcddev.height)*2*2, lcddev.width,lcddev.height*2, buffpic);
				       break;
				case 9:sendicon(0,0,lcddev.width,lcddev.height,(pic_ad+off_ad)+(lcddev.width*lcddev.height)*2*3, lcddev.width,lcddev.height*2, buffpic);
				       break;
				case 10:sendicon(0,0,lcddev.width,lcddev.height,(pic_ad+off_ad)+(lcddev.width*lcddev.height)*2*4, lcddev.width,lcddev.height*2, buffpic);
		           free(buffpic);
			         break;
			  			  			  			  		
			  case 11:LCD_Clear(WHITE); 
				        Show_Str(0,0,lcddev.width,lcddev.height,"    广东省亦亚徽科技集团是一家集研发、生产、销售为一体的科技型企业。" 
				                                                 "公司专业制造0.68英寸-15.6英寸液晶显示屏、串口屏、智能显示屏，提供显示解决方案。"
				                                                 "产品广泛应用于工业、车载、医疗、家电、物联网、智能家居等各行各业。"
				                                                 "工厂面积5000多M2,拥有5条全自动生产线，每月饱和产能2KK."
				                                                 " 公司拥有行业顶尖的研发团队10多人，核心人员行业经验15年以上，能快速提供各种显示方案,大大缩短客户的开发周期。"
				                                                 "公司制造部的设备工程师团队，行业资历15年以上。"
				                                                 "公司拥有15人的品质团队，在各个环节严格把关，为优良品质保驾护航。"
				                                                 "公司还拥有30多人的销售团队，分国内市场部和国际市场部两个部门。"
				                                                 "公司通过了ISO9001质量管理体系认证，拥有完善的管理制度。"
				                                                 "“利用雄厚的技术实力，强大的制造能力，优质的服务能力”"
				                                                 "为客户提供高性价比的产品和服务是公司最大的竞争力。"
				                                                 "感谢老客户一直以来的支持，也欢迎越来越多新客户的加入。"			
				                                                 "亦亚徽期待与志同道合的客户一起成长，共荣共赢。"
				                                                 "公司网址:wwww.eya-display.com",16,1,BLACK);
			
				        if(AUTO==1) ix=0; //如果没有触摸，就不需要测试触摸的画面了
				        break;
				case 12:Load_Drow_Dialog();Show_Str(10,10,100,16,"触摸测试",16,1,RED);
							  if (tp_dev.touchtype & 0X80)
							  {
								  ctp_test(); ix=0; /* 电容屏测试 */
							  }
							  else
							  {
								  rtp_test(); ix=0; /* 电阻屏测试 */
							  }
								break;
		   }
			Contflag=0;//停止计时
			LCD_ShowNum(lcddev.width-35,5,Counter*5,3,12,BLACK);
			LCD_ShowString(lcddev.width-17,5,24,12,12,"MS",BLACK); //显示刷图时间
			printf("刷图时间是：%d ms\r\n",Counter*5);
			printf("ix===%d\r\n",ix);
			Counter=0;//清除计时器
			ShowUI(ix);
		 }
	//////////////////////////////////////////以下开始LCD功能测试 结束///////////////////////////////////////
  }
}
