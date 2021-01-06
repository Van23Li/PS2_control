/******************************************************************************************************
说明：
红灯模式、绿灯模式两种模式下处理。
按下对应按钮，串口1返回对应按钮名称。
红灯模式下摇杆模拟数值有效，绿灯模式下无效。			
******************************************************************************************************/


#include "stc12c5a60s2.h"	//stc12头文件
#include <stdio.h>			//输入输出函数头文件
#include <string.h>			//字符头文件


/***********************************宏定义*******************************************/
#define interrupt_open()	{EA = 1;}	//中断开
#define interrupt_close()	{EA = 0;}	//中断关
#define UART_BUF_SIZE 	 64			//字符接收数组大小
#define START_CMD			0x01	  //手柄起始指令
#define ASK_DAT_CMD			0x42	  //手柄应答指令
#define PSX_GREEN_MODE      0x41	  //手柄绿灯模式对应码
#define PSX_RED_MODE        0x73	  //手柄红灯模式对应码								 
#define PS2_CMD_NUM 16				   //手柄按钮数
#define PSX_BUF_SIZE 9				   //手柄数组大小
#define CMD_RETURN_SIZE 50			   //串口打印数组大小

#define       L1_RELEASE       0x01				 //设置L1键弹起标志
#define       L2_RELEASE       0x02				 //设置L2键弹起标志
#define       R1_RELEASE       0x03				 //设置R1键弹起标志
#define       R2_RELEASE       0x04				 //设置R2键弹起标志
#define       LU_RELEASE       0x05				 //设置LU键弹起标志
#define       LD_RELEASE       0x06				 //设置LD键弹起标志
#define       LL_RELEASE       0x07				 //设置LL键弹起标志
#define       LR_RELEASE       0x08				 //设置LR键弹起标志
#define       RU_RELEASE       0x09				 //设置RU键弹起标志
#define       RD_RELEASE       0x10				 //设置RD键弹起标志
#define       RL_RELEASE       0x11				 //设置RL键弹起标志
#define       RR_RELEASE       0x12				 //设置RR键弹起标志
#define       SE_RELEASE       0x13				 //设置SE键弹起标志
#define       ST_RELEASE       0x14  			 //设置ST键弹起标志
#define       AL_RELEASE       0x15  			 //设置AL键弹起标志
#define       AR_RELEASE       0x16  			 //设置AR键弹起标志


/***********************************IO位定义*******************************************/
sbit PS2_DAT = P2^0; //pin1
sbit PS2_CMD = P2^1; //pin2
sbit PS2_ATT = P2^2; //pin6
sbit PS2_CLK = P2^3; //pin7

sbit en=P2^6;		//lcd1602  6管脚
sbit rs=P2^4;	 //lcd1602端口	4管脚
sbit rw=P2^5;//lcd1602控制端口 5管脚

sbit motor_A=P1^5;
sbit motor_A_IN1=P1^4;
sbit motor_A_IN2=P1^3;

sbit motor_B=P1^0;
sbit motor_B_IN1=P1^2;
sbit motor_B_IN2=P1^1;

sbit motor_C=P1^6;
sbit motor_C_IN1=P3^0;
sbit motor_C_IN2=P1^7;

sbit motor_D=P3^4;
sbit motor_D_IN1=P3^2;
sbit motor_D_IN2=P3^1;				//两个L298N驱动模块输入管脚

unsigned int flag;							//2s更新转速循环计数标志
unsigned int PWM = 0;						//占空比，PWM越大，转速越快
unsigned int zhuansu = 0;					//电机转速

unsigned char power = 0;							//电机转动标志
unsigned char uart_get_ok;							//中断标志位 无符号类型
unsigned char psx_buf[PSX_BUF_SIZE];		//通讯字符
unsigned char deal_flag = 0;							//处理标志
unsigned char release_flag = 0;						    //弹起标志


/***********************************函数声明*******************************************/
void system_init(void);			 //系统初始化
void io_init(void);				 //IO初始化函数
void init_lcd(void);				 //初始化显示屏
void init_timer(void);				 //初始化中断


void North(unsigned int PWM_IN);				
void South(unsigned int PWM_IN);		
void West(unsigned int PWM_IN);			
void East(unsigned int PWM_IN);				
void TurnLeft(unsigned int PWM_IN);			
void Northwest(unsigned int PWM_IN);			
void Northeast(unsigned int PWM_IN);			
void Southwest(unsigned int PWM_IN);			
void Southeast(unsigned int PWM_IN);			
void TurnRight(unsigned int PWM_IN);				 //电机转动函数

void show_PWM(void);				 //显示占空比
void show_speed(void);				 //显示转速
void write_data(unsigned char date);				 //向1602写一字节（数据）
void write_com(unsigned char com);				 //向1602写一字节（控制指令）

void uart1_init(void);			 //串口1初始化函数
void uart1_send_str(unsigned char *s);		//串口1字符发送函数
void delay_ms(unsigned int t);	 //ms级别延时
void delay_us(unsigned int t);		 //us级别延时
void psx_init(void);			//手柄初始化
unsigned char psx_transfer(unsigned char dat);			 //手柄指令解析
void psx_write_read(unsigned char *get_buf);			 //手柄读写
void handle_psx_green(void);									 //手柄处理函数


/*=========================================主函数================================================
函数名：main
功能介绍：程序运行入口
函数参数：无
返回值：无
===============================================================================================*/				  
void main(void) {							 //主函数
	motor_A = 0;
	motor_B = 0;
	motor_C = 0;
	motor_D = 0;					//初始保证电机停转
	
	system_init();	  						 //系统初始化
	while(1) {
		handle_psx_green(); 				 //处理绿灯模式
		delay_ms(50);
	}	
}


/*=========================================系统初始化============================================
函数名：system_init
功能介绍：各个功能函数的初始化设置
函数参数：无
返回值：无
===============================================================================================*/
void system_init(void) {
	init_timer();						//初始化中断
  init_lcd();              //初始化液晶显示屏
	io_init();	  			  //IO初始化
	uart1_init();			  //串口1初始化
  uart1_send_str("uart1 init ok!\r\n");	 //串口打印信息
  psx_init();				 //手柄初始化
}


/*=========================================手柄初始化============================================
函数名：psx_init
功能介绍：PS2引脚拉高
函数参数：无
返回值：无
===============================================================================================*/
void psx_init(void) {
	PS2_ATT = 1;
	PS2_CMD = 1;
	PS2_CLK = 1;
	return;
}


/*=========================================绿灯模式处理函数======================================
函数名：handle_psx_green
功能介绍：绿灯模式下，按钮处理
函数参数：无
返回值：无
===============================================================================================*/
void handle_psx_green(void) {
	psx_write_read(psx_buf);//读取手柄
	if(psx_buf[1] == PSX_GREEN_MODE) {					 //判断是否为绿灯模式
		switch(psx_buf[3]) {			   
			case 0xef: {if(deal_flag < 1) {North(PWM);uart1_send_str("GREEN_MODE_PUSH:LU    ");release_flag = LU_RELEASE;deal_flag++;}break;} //LU
			case 0xbf: {if(deal_flag < 1) {South(PWM);uart1_send_str("GREEN_MODE_PUSH:LD    ");release_flag = LD_RELEASE;deal_flag++;}break;} //LD
			case 0x7f: {if(deal_flag < 1) {West(PWM);uart1_send_str("GREEN_MODE_PUSH:LR    ");release_flag = LR_RELEASE;deal_flag++;}break;} //LL 
			case 0xdf: {if(deal_flag < 1) {East(PWM);uart1_send_str("GREEN_MODE_PUSH:LL    ");release_flag = LL_RELEASE;deal_flag++;}break;} //LR
			default: {
				 switch(release_flag) {
					 case LU_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:LU\r\n");break;		//弹起后，指示灯灭，串口发送对应按钮弹起信息
					 case LD_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:LD\r\n");break;
					 case LL_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:LL\r\n");break;
					 case LR_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:LR\r\n");break;
					 default:break;
				 }			 			
				 break;
			}
		} 	
		switch(psx_buf[4]) {	 	  	   	 		  	   	   
				case 0xfb: {if(deal_flag < 1) {TurnLeft(PWM);uart1_send_str("GREEN_MODE_PUSH:L1    ");release_flag = L1_RELEASE;deal_flag++;}break;}	//L1
		    case 0xfe: {if(deal_flag < 1) {PWM=PWM+10;if(PWM >100){PWM=0;}show_PWM();uart1_send_str("GREEN_MODE_PUSH:L2    ");release_flag = L2_RELEASE;deal_flag++;}break;}	//L2
		    case 0xf7: {if(deal_flag < 1) {TurnRight(PWM);uart1_send_str("GREEN_MODE_PUSH:R1    ");release_flag = R1_RELEASE;deal_flag++;}break;}	//R1
		    case 0xfd: {if(deal_flag < 1) {PWM=PWM-10;if(PWM >100){PWM=100;}show_PWM();uart1_send_str("GREEN_MODE_PUSH:R2    ");release_flag = R2_RELEASE;deal_flag++;}break;}	//R2
		    case 0xef: {if(deal_flag < 1) {Northeast(PWM);uart1_send_str("GREEN_MODE_PUSH:RU    ");release_flag = RU_RELEASE;deal_flag++;}break;}	//RU	
		    case 0xbf: {if(deal_flag < 1) {Southwest(PWM);uart1_send_str("GREEN_MODE_PUSH:RD    ");release_flag = RD_RELEASE;deal_flag++;}break;}	//RD
				case 0x7f: {if(deal_flag < 1) {Northwest(PWM);uart1_send_str("GREEN_MODE_PUSH:RL    ");release_flag = RL_RELEASE;deal_flag++;}break;}	//RL
		    case 0xdf: {if(deal_flag < 1) {Southeast(PWM);uart1_send_str("GREEN_MODE_PUSH:RR    ");release_flag = RR_RELEASE;deal_flag++;}break;}	//RR
			default:{
			switch(release_flag) {
				case L1_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:L1\r\n");break;		//弹起后，指示灯灭，串口发送对应按钮弹起信息
				case L2_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:L2\r\n");break;
				case R1_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:R1\r\n");break;
				case R2_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:R2\r\n");break;
				case RU_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:RU\r\n");break;
				case RD_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:RD\r\n");break;
				case RL_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:RL\r\n");break;
				case RR_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:RR\r\n");break;
				default:break;
			 }			 			
			 break;
		   }
	   }
	}	 	
}
					 

/*=========================================手柄原码获取函数=======================================
函数名：psx_write_read
功能介绍：解析并保存手柄传输的数据
函数参数：*get_buf 获取数据
返回值：无
===============================================================================================*/
void psx_write_read(unsigned char *get_buf) {
	PS2_ATT = 0;							//各组发送接收的字节前，必须被拉低，再重新置位
	get_buf[0] = psx_transfer(START_CMD);				//主机将0x01传给手柄 手柄起始指令
	get_buf[1] = psx_transfer(ASK_DAT_CMD);			//主机将0x42传给手柄 手柄应答指令 0x41=绿灯,0x73=红灯
	get_buf[2] = psx_transfer(get_buf[0]);			//手柄将0x5A传给主机 标志数据来了 后面主机都没有再穿数据给手柄
	get_buf[3] = psx_transfer(get_buf[0]);			//左边按键
	get_buf[4] = psx_transfer(get_buf[0]);			//右边按键
	get_buf[5] = psx_transfer(get_buf[0]);			//模拟量
	get_buf[6] = psx_transfer(get_buf[0]);			//模拟量
	get_buf[7] = psx_transfer(get_buf[0]);			//模拟量
	get_buf[8] = psx_transfer(get_buf[0]);			//模拟量
	PS2_ATT = 1;
}


/*=========================================手柄码解析函数=======================================
函数名：psx_transfer
功能介绍：解析手柄传输的数据
函数参数：dat（手柄源码）
返回值：rd_data（接）
===============================================================================================*/
unsigned char psx_transfer(unsigned char dat) {
	unsigned char rd_data ,wt_data, i;
	wt_data = dat;
	rd_data = 0;
	for(i = 0;i < 8;i++){
		PS2_CMD = (bit) (wt_data & (0x01 << i));	//把0x01左移i位后取并，也就是从右到左提取wt_data，共8位，转换为bit类型
																							//赋值给PS2_CMD（主机流向手柄的信息）
		PS2_CLK = 1;
		PS2_CLK = 0;
		delay_us(3);	//延时3ms
		PS2_CLK = 1;
		if(PS2_DAT) {
			rd_data |= 0x01<<i;		//最后结果是rd_data = PS2_DAT（手柄流向主机的信息）
		}
	}
	return rd_data;
}


/*=========================================串口1初始化函数=======================================
函数名：uart1_init
功能介绍：串口的相关设置
函数参数：无
返回值：无
===============================================================================================*/
void uart1_init(void) {  //4800bps@11.0592MHz
	PCON &= 0x7F;		//01111111 波特率不倍速
	SCON = 0x50;		//8位数据,可变波特率
	BRT = 0xB8;			//设定独立波特率发生器重装值
	AUXR |= 0x04;		//独立波特率发生器时钟为Fosc,即1T 按位或后赋值
	AUXR |= 0x01;		//串口1选择独立波特率发生器为波特率发生器
	AUXR |= 0x10;		//启动独立波特率发生器
}


/*=========================================串口1字符串发送函数=====================================
函数名：uart1_send_str
功能介绍：发送字符串
函数参数：*s :发送的字符串首地址
返回值：无
===============================================================================================*/
void uart1_send_str(unsigned char *s) {   
    while(*s!='\0') {
		SBUF=*s;
		while(!TI);
		TI=0;
		s++;
	} 
	return;
} 


/*=========================================IO初始化============================================
函数名：io_init
功能介绍：IO口设置
函数参数：无
返回值：无
===============================================================================================*/
void io_init(void) {
	P2M0 = 0x00;		  //设置P2端口为准双向
	P2M1 = 0;
	P3M0 = 0xff;		  //设置P3端口为推挽输出
	P3M1 = 0;
	return;
}


/*=========================================延时函数==============================================
函数名：delay_ms
功能介绍：ms级延时
函数参数：t :延时时间
返回值：无
===============================================================================================*/
void delay_ms(unsigned int z) {
	unsigned int x,y;
	for(x=z; x>0; x--)
		for(y=110; y>0; y--);
}


/*=========================================延时函数==============================================
函数名：delay_us
功能介绍：us级延时
函数参数：t :延时时间
返回值：无
===============================================================================================*/
void delay_us(unsigned int zz) {
	while(zz--);
}


/*=========================================前进函数==============================================
函数名：North
功能介绍：小车前进
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void North(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//前
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//前
	motor_C_IN1 = 0; motor_C_IN2 = 1;		//前
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//前
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[3]) {
									case 0xef:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================后退函数==============================================
函数名：South
功能介绍：小车后退
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void South(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//后
	motor_B_IN1 = 1; motor_B_IN2 = 0;		//后
	motor_C_IN1 = 1; motor_C_IN2 = 0;		//后
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//后
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[3]) {
									case 0xbf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================左向平移函数==============================================
函数名：West
功能介绍：左向平移
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void West(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//后
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//前
	motor_C_IN1 = 1; motor_C_IN2 = 0;		//后
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//前
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[3]) {
									case 0x7f:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================右向平移函数==============================================
函数名：East
功能介绍：右向平移
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void East(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//前
	motor_B_IN1 = 1; motor_B_IN2 = 0;		//后
	motor_C_IN1 = 0; motor_C_IN2 = 1;		//前
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//后
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[3]) {
									case 0xdf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================逆时针旋转函数==============================================
函数名：TurnLeft
功能介绍：逆时针旋转
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void TurnLeft(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//后
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//前
	motor_C_IN1 = 0; motor_C_IN2 = 1;		//前
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//后
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[4]) {
									case 0xfb:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================顺时针函数==============================================
函数名：TurnRight
功能介绍：逆时针旋转
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void TurnRight(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//前
	motor_B_IN1 = 1; motor_B_IN2 = 0;		//后
	motor_C_IN1 = 1; motor_C_IN2 = 0;		//后
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//前
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[4]) {
									case 0xf7:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================左前方平移函数==============================================
函数名：Northwest
功能介绍：左前方平移
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void Northwest(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 0;	
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//前
	motor_C_IN1 = 0; motor_C_IN2 = 0;
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//前
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[4]) {
									case 0x7f:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================右前方平移函数==============================================
函数名：Northeast
功能介绍：右前方平移
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void Northeast(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//前
	motor_B_IN1 = 0; motor_B_IN2 = 0;	
	motor_C_IN1 = 0; motor_C_IN2 = 1;	//前
	motor_D_IN1 = 0; motor_D_IN2 = 0;	
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[4]) {
									case 0xef:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================左后方平移函数==============================================
函数名：Southwest
功能介绍：右前方平移
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void Southwest(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//后
	motor_B_IN1 = 0; motor_B_IN2 = 0;	
	motor_C_IN1 = 1; motor_C_IN2 = 0;	//后
	motor_D_IN1 = 0; motor_D_IN2 = 0;	
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[4]) {
									case 0xbf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================右后方平移函数==============================================
函数名：Southeast
功能介绍：右后方平移
函数参数：PWM_IN：调节占空比，越高车轮转速越快
返回值：无
===============================================================================================*/
void Southeast(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 0;
	motor_B_IN1 = 1; motor_B_IN2 = 0;			//后
	motor_C_IN1 = 0; motor_C_IN2 = 0;
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//后
							power = 1;
							while(power){
								motor_A = 1;
								motor_B = 1;
								motor_C = 1;
								motor_D = 1;
								delay_ms(PWM_IN);
								motor_A = 0;
								motor_B = 0;
								motor_C = 0;
								motor_D = 0;
								delay_ms(100-PWM_IN);
								psx_write_read(psx_buf);//读取手柄
								switch(psx_buf[4]) {
									case 0xdf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================向液晶显示屏写指令函数==============================================
函数名：write_com
功能介绍：向液晶显示屏写指令函数
函数参数：com：将写入的命令
返回值：无
===============================================================================================*/
void write_com(unsigned char com)//向1602写一字节（控制指令）
{
  rs=0;
  P0=com;
  delay_ms(5);
  en=1;
  delay_ms(10);
  en=0;
}


/*=========================================向液晶显示屏写数据函数==============================================
函数名：write_data
功能介绍：向液晶显示屏写数据函数
函数参数：data：将写入的数据
返回值：无
===============================================================================================*/
void write_data(unsigned char date)//向1602写一字节（数据）
{
  rs=1;
  P0=date;
  delay_ms(5);
  en=1;
  delay_ms(5);
  en=0;
}


/*=========================================定时器初始化函数==============================================
函数名：init_timer
功能介绍：定时器初始化函数
函数参数：
返回值：无
===============================================================================================*/
void init_timer()
{
	TMOD=0x01;               //定时器0为工作方式1 ，定时器1为工作方式0
  
	TH0=(65536-9216)/256;
  TL0=(65536-9216)%256;           //定时器装入初值 10ms
  EA=1;                    //开总中断 
  ET0=1;                   //定时器0开中断
  TR0=1; 										//定时器0运行控制位
  
	EX1=1;									//打开外部中断1中断
  IT1=1;                 //外部中断1触发方式为跳变沿触发，INT1又高到低有效 
}


/*=========================================液晶显示屏初始化函数==============================================
函数名：init_lcd
功能介绍：液晶显示屏初始化函数
函数参数：
返回值：无
===============================================================================================*/
void init_lcd()//初始化函数
{ 
  en=0;
  rw=0;
  write_com(0x38);					   //5X7显示
  write_com(0x01);         //lcd初始化
  write_com(0x0c);			 //关闭光标
	write_com(0x01);         //lcd初始化
	
  write_com(0x80);
  write_data('V');
  write_data(':');
  write_com(0x80+7);		 //第一行显示转速
  write_data('r');
  write_data('p');
  write_data('m');
  write_com(0x80+0x40); 
  write_data('z');
  write_data('h');
  write_data('a');
  write_data('n');
  write_data('k');
  write_data('o');
  write_data('n');
  write_data('g');
  write_data('b');
  write_data('i');	  //在第二行显示zhankongbi:
  write_data(':');
  show_PWM();
}


/*=========================================显示转速函数==============================================
函数名：show_speed
功能介绍：显示转速
函数参数：
返回值：无
===============================================================================================*/
void show_speed()
{
	unsigned zhuansu_x;
  write_com(0x82);
  zhuansu_x=zhuansu/4;	  //将两秒内的计数乘以30得到转每分 2个脉冲/圈 zhuansu/4

  if(zhuansu_x/10000!=0)
   write_data(zhuansu_x/10000+0x30);						  //如果转速的万位不为0	正常显示否则显示空格
   else
   write_data(' ');

  if(zhuansu_x/1000==0)
  write_data(' ');
  else 
  write_data(zhuansu_x%10000/1000+0x30);								//如果转速小于1000 千位为空格 否则正常显示
  
  if(zhuansu_x/100==0)
  write_data(' ');
  else
  write_data(zhuansu_x%10000%1000/100+0x30);					   //如果转速小于100 百位为空格 否则正常显示

    if(zhuansu_x/10==0)
  write_data(' ');
  else
  write_data(zhuansu_x%10000%1000%100/10+0x30);				  //如果转速小于10 十位为空格 否则正常显示

  write_data(zhuansu_x%10000%1000%100%10+0x30);
  write_com(0xd0);
}


/*=========================================显示占空比函数==============================================
函数名：show_PWM
功能介绍：显示占空比数值
函数参数：
返回值：无
===============================================================================================*/
void show_PWM()			//显示占空比数值
{
   write_com(0xcd);
	 write_data(PWM/100+0x30);
	
	 write_com(0xce);
	 write_data(PWM%100/10+0x30);	

	write_com(0xcf);
	 write_data(PWM%10+0x30);		
}


/*=========================================外部中断==============================================
函数名：int1
功能介绍：记录霍尔元件脉冲数，用于计算电机转速
函数参数：
返回值：无
===============================================================================================*/
void int1()interrupt 2				//外部中断1 脉冲技术记录电机的转速 电机转一圈zhuansu加1
{
  zhuansu++;
}


/*=========================================定时器中断==============================================
函数名：int2
功能介绍：每隔1ms中断一次，累计2s时更新一次速度显示
函数参数：
返回值：无
===============================================================================================*/
void int2()interrupt 1				  //定时器0 显示转速
{
  TH0=(65536-9216)/256;
  TL0=(65536-9216)%256;//定时10ms
  flag++;
  if(flag==200)				   //计时到达2s，即每隔2s更新一此速度显示
    {
	show_speed();					//显示转速
	zhuansu=0;					  //转速置0
	flag=0;
	}
}