/******************************************************************************************************
˵����
���ģʽ���̵�ģʽ����ģʽ�´���
���¶�Ӧ��ť������1���ض�Ӧ��ť���ơ�
���ģʽ��ҡ��ģ����ֵ��Ч���̵�ģʽ����Ч��			
******************************************************************************************************/


#include "stc12c5a60s2.h"	//stc12ͷ�ļ�
#include <stdio.h>			//�����������ͷ�ļ�
#include <string.h>			//�ַ�ͷ�ļ�


/***********************************�궨��*******************************************/
#define interrupt_open()	{EA = 1;}	//�жϿ�
#define interrupt_close()	{EA = 0;}	//�жϹ�
#define UART_BUF_SIZE 	 64			//�ַ����������С
#define START_CMD			0x01	  //�ֱ���ʼָ��
#define ASK_DAT_CMD			0x42	  //�ֱ�Ӧ��ָ��
#define PSX_GREEN_MODE      0x41	  //�ֱ��̵�ģʽ��Ӧ��
#define PSX_RED_MODE        0x73	  //�ֱ����ģʽ��Ӧ��								 
#define PS2_CMD_NUM 16				   //�ֱ���ť��
#define PSX_BUF_SIZE 9				   //�ֱ������С
#define CMD_RETURN_SIZE 50			   //���ڴ�ӡ�����С

#define       L1_RELEASE       0x01				 //����L1�������־
#define       L2_RELEASE       0x02				 //����L2�������־
#define       R1_RELEASE       0x03				 //����R1�������־
#define       R2_RELEASE       0x04				 //����R2�������־
#define       LU_RELEASE       0x05				 //����LU�������־
#define       LD_RELEASE       0x06				 //����LD�������־
#define       LL_RELEASE       0x07				 //����LL�������־
#define       LR_RELEASE       0x08				 //����LR�������־
#define       RU_RELEASE       0x09				 //����RU�������־
#define       RD_RELEASE       0x10				 //����RD�������־
#define       RL_RELEASE       0x11				 //����RL�������־
#define       RR_RELEASE       0x12				 //����RR�������־
#define       SE_RELEASE       0x13				 //����SE�������־
#define       ST_RELEASE       0x14  			 //����ST�������־
#define       AL_RELEASE       0x15  			 //����AL�������־
#define       AR_RELEASE       0x16  			 //����AR�������־


/***********************************IOλ����*******************************************/
sbit PS2_DAT = P2^0; //pin1
sbit PS2_CMD = P2^1; //pin2
sbit PS2_ATT = P2^2; //pin6
sbit PS2_CLK = P2^3; //pin7

sbit en=P2^6;		//lcd1602  6�ܽ�
sbit rs=P2^4;	 //lcd1602�˿�	4�ܽ�
sbit rw=P2^5;//lcd1602���ƶ˿� 5�ܽ�

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
sbit motor_D_IN2=P3^1;				//����L298N����ģ������ܽ�

unsigned int flag;							//2s����ת��ѭ��������־
unsigned int PWM = 0;						//ռ�ձȣ�PWMԽ��ת��Խ��
unsigned int zhuansu = 0;					//���ת��

unsigned char power = 0;							//���ת����־
unsigned char uart_get_ok;							//�жϱ�־λ �޷�������
unsigned char psx_buf[PSX_BUF_SIZE];		//ͨѶ�ַ�
unsigned char deal_flag = 0;							//�����־
unsigned char release_flag = 0;						    //�����־


/***********************************��������*******************************************/
void system_init(void);			 //ϵͳ��ʼ��
void io_init(void);				 //IO��ʼ������
void init_lcd(void);				 //��ʼ����ʾ��
void init_timer(void);				 //��ʼ���ж�


void North(unsigned int PWM_IN);				
void South(unsigned int PWM_IN);		
void West(unsigned int PWM_IN);			
void East(unsigned int PWM_IN);				
void TurnLeft(unsigned int PWM_IN);			
void Northwest(unsigned int PWM_IN);			
void Northeast(unsigned int PWM_IN);			
void Southwest(unsigned int PWM_IN);			
void Southeast(unsigned int PWM_IN);			
void TurnRight(unsigned int PWM_IN);				 //���ת������

void show_PWM(void);				 //��ʾռ�ձ�
void show_speed(void);				 //��ʾת��
void write_data(unsigned char date);				 //��1602дһ�ֽڣ����ݣ�
void write_com(unsigned char com);				 //��1602дһ�ֽڣ�����ָ�

void uart1_init(void);			 //����1��ʼ������
void uart1_send_str(unsigned char *s);		//����1�ַ����ͺ���
void delay_ms(unsigned int t);	 //ms������ʱ
void delay_us(unsigned int t);		 //us������ʱ
void psx_init(void);			//�ֱ���ʼ��
unsigned char psx_transfer(unsigned char dat);			 //�ֱ�ָ�����
void psx_write_read(unsigned char *get_buf);			 //�ֱ���д
void handle_psx_green(void);									 //�ֱ�������


/*=========================================������================================================
��������main
���ܽ��ܣ������������
������������
����ֵ����
===============================================================================================*/				  
void main(void) {							 //������
	motor_A = 0;
	motor_B = 0;
	motor_C = 0;
	motor_D = 0;					//��ʼ��֤���ͣת
	
	system_init();	  						 //ϵͳ��ʼ��
	while(1) {
		handle_psx_green(); 				 //�����̵�ģʽ
		delay_ms(50);
	}	
}


/*=========================================ϵͳ��ʼ��============================================
��������system_init
���ܽ��ܣ��������ܺ����ĳ�ʼ������
������������
����ֵ����
===============================================================================================*/
void system_init(void) {
	init_timer();						//��ʼ���ж�
  init_lcd();              //��ʼ��Һ����ʾ��
	io_init();	  			  //IO��ʼ��
	uart1_init();			  //����1��ʼ��
  uart1_send_str("uart1 init ok!\r\n");	 //���ڴ�ӡ��Ϣ
  psx_init();				 //�ֱ���ʼ��
}


/*=========================================�ֱ���ʼ��============================================
��������psx_init
���ܽ��ܣ�PS2��������
������������
����ֵ����
===============================================================================================*/
void psx_init(void) {
	PS2_ATT = 1;
	PS2_CMD = 1;
	PS2_CLK = 1;
	return;
}


/*=========================================�̵�ģʽ������======================================
��������handle_psx_green
���ܽ��ܣ��̵�ģʽ�£���ť����
������������
����ֵ����
===============================================================================================*/
void handle_psx_green(void) {
	psx_write_read(psx_buf);//��ȡ�ֱ�
	if(psx_buf[1] == PSX_GREEN_MODE) {					 //�ж��Ƿ�Ϊ�̵�ģʽ
		switch(psx_buf[3]) {			   
			case 0xef: {if(deal_flag < 1) {North(PWM);uart1_send_str("GREEN_MODE_PUSH:LU    ");release_flag = LU_RELEASE;deal_flag++;}break;} //LU
			case 0xbf: {if(deal_flag < 1) {South(PWM);uart1_send_str("GREEN_MODE_PUSH:LD    ");release_flag = LD_RELEASE;deal_flag++;}break;} //LD
			case 0x7f: {if(deal_flag < 1) {West(PWM);uart1_send_str("GREEN_MODE_PUSH:LR    ");release_flag = LR_RELEASE;deal_flag++;}break;} //LL 
			case 0xdf: {if(deal_flag < 1) {East(PWM);uart1_send_str("GREEN_MODE_PUSH:LL    ");release_flag = LL_RELEASE;deal_flag++;}break;} //LR
			default: {
				 switch(release_flag) {
					 case LU_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:LU\r\n");break;		//�����ָʾ���𣬴��ڷ��Ͷ�Ӧ��ť������Ϣ
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
				case L1_RELEASE: motor_A = 0;motor_B = 0;motor_C = 0;motor_D = 0;deal_flag = 0;release_flag = 0;uart1_send_str("RELEASE:L1\r\n");break;		//�����ָʾ���𣬴��ڷ��Ͷ�Ӧ��ť������Ϣ
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
					 

/*=========================================�ֱ�ԭ���ȡ����=======================================
��������psx_write_read
���ܽ��ܣ������������ֱ����������
����������*get_buf ��ȡ����
����ֵ����
===============================================================================================*/
void psx_write_read(unsigned char *get_buf) {
	PS2_ATT = 0;							//���鷢�ͽ��յ��ֽ�ǰ�����뱻���ͣ���������λ
	get_buf[0] = psx_transfer(START_CMD);				//������0x01�����ֱ� �ֱ���ʼָ��
	get_buf[1] = psx_transfer(ASK_DAT_CMD);			//������0x42�����ֱ� �ֱ�Ӧ��ָ�� 0x41=�̵�,0x73=���
	get_buf[2] = psx_transfer(get_buf[0]);			//�ֱ���0x5A�������� ��־�������� ����������û���ٴ����ݸ��ֱ�
	get_buf[3] = psx_transfer(get_buf[0]);			//��߰���
	get_buf[4] = psx_transfer(get_buf[0]);			//�ұ߰���
	get_buf[5] = psx_transfer(get_buf[0]);			//ģ����
	get_buf[6] = psx_transfer(get_buf[0]);			//ģ����
	get_buf[7] = psx_transfer(get_buf[0]);			//ģ����
	get_buf[8] = psx_transfer(get_buf[0]);			//ģ����
	PS2_ATT = 1;
}


/*=========================================�ֱ����������=======================================
��������psx_transfer
���ܽ��ܣ������ֱ����������
����������dat���ֱ�Դ�룩
����ֵ��rd_data���ӣ�
===============================================================================================*/
unsigned char psx_transfer(unsigned char dat) {
	unsigned char rd_data ,wt_data, i;
	wt_data = dat;
	rd_data = 0;
	for(i = 0;i < 8;i++){
		PS2_CMD = (bit) (wt_data & (0x01 << i));	//��0x01����iλ��ȡ����Ҳ���Ǵ��ҵ�����ȡwt_data����8λ��ת��Ϊbit����
																							//��ֵ��PS2_CMD�����������ֱ�����Ϣ��
		PS2_CLK = 1;
		PS2_CLK = 0;
		delay_us(3);	//��ʱ3ms
		PS2_CLK = 1;
		if(PS2_DAT) {
			rd_data |= 0x01<<i;		//�������rd_data = PS2_DAT���ֱ�������������Ϣ��
		}
	}
	return rd_data;
}


/*=========================================����1��ʼ������=======================================
��������uart1_init
���ܽ��ܣ����ڵ��������
������������
����ֵ����
===============================================================================================*/
void uart1_init(void) {  //4800bps@11.0592MHz
	PCON &= 0x7F;		//01111111 �����ʲ�����
	SCON = 0x50;		//8λ����,�ɱ䲨����
	BRT = 0xB8;			//�趨���������ʷ�������װֵ
	AUXR |= 0x04;		//���������ʷ�����ʱ��ΪFosc,��1T ��λ���ֵ
	AUXR |= 0x01;		//����1ѡ����������ʷ�����Ϊ�����ʷ�����
	AUXR |= 0x10;		//�������������ʷ�����
}


/*=========================================����1�ַ������ͺ���=====================================
��������uart1_send_str
���ܽ��ܣ������ַ���
����������*s :���͵��ַ����׵�ַ
����ֵ����
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


/*=========================================IO��ʼ��============================================
��������io_init
���ܽ��ܣ�IO������
������������
����ֵ����
===============================================================================================*/
void io_init(void) {
	P2M0 = 0x00;		  //����P2�˿�Ϊ׼˫��
	P2M1 = 0;
	P3M0 = 0xff;		  //����P3�˿�Ϊ�������
	P3M1 = 0;
	return;
}


/*=========================================��ʱ����==============================================
��������delay_ms
���ܽ��ܣ�ms����ʱ
����������t :��ʱʱ��
����ֵ����
===============================================================================================*/
void delay_ms(unsigned int z) {
	unsigned int x,y;
	for(x=z; x>0; x--)
		for(y=110; y>0; y--);
}


/*=========================================��ʱ����==============================================
��������delay_us
���ܽ��ܣ�us����ʱ
����������t :��ʱʱ��
����ֵ����
===============================================================================================*/
void delay_us(unsigned int zz) {
	while(zz--);
}


/*=========================================ǰ������==============================================
��������North
���ܽ��ܣ�С��ǰ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void North(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//ǰ
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//ǰ
	motor_C_IN1 = 0; motor_C_IN2 = 1;		//ǰ
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//ǰ
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[3]) {
									case 0xef:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================���˺���==============================================
��������South
���ܽ��ܣ�С������
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void South(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//��
	motor_B_IN1 = 1; motor_B_IN2 = 0;		//��
	motor_C_IN1 = 1; motor_C_IN2 = 0;		//��
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//��
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[3]) {
									case 0xbf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================����ƽ�ƺ���==============================================
��������West
���ܽ��ܣ�����ƽ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void West(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//��
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//ǰ
	motor_C_IN1 = 1; motor_C_IN2 = 0;		//��
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//ǰ
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[3]) {
									case 0x7f:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================����ƽ�ƺ���==============================================
��������East
���ܽ��ܣ�����ƽ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void East(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//ǰ
	motor_B_IN1 = 1; motor_B_IN2 = 0;		//��
	motor_C_IN1 = 0; motor_C_IN2 = 1;		//ǰ
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//��
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[3]) {
									case 0xdf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================��ʱ����ת����==============================================
��������TurnLeft
���ܽ��ܣ���ʱ����ת
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void TurnLeft(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//��
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//ǰ
	motor_C_IN1 = 0; motor_C_IN2 = 1;		//ǰ
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//��
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[4]) {
									case 0xfb:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================˳ʱ�뺯��==============================================
��������TurnRight
���ܽ��ܣ���ʱ����ת
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void TurnRight(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//ǰ
	motor_B_IN1 = 1; motor_B_IN2 = 0;		//��
	motor_C_IN1 = 1; motor_C_IN2 = 0;		//��
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//ǰ
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[4]) {
									case 0xf7:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================��ǰ��ƽ�ƺ���==============================================
��������Northwest
���ܽ��ܣ���ǰ��ƽ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void Northwest(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 0;	
	motor_B_IN1 = 0; motor_B_IN2 = 1;		//ǰ
	motor_C_IN1 = 0; motor_C_IN2 = 0;
	motor_D_IN1 = 0; motor_D_IN2 = 1;		//ǰ
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[4]) {
									case 0x7f:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================��ǰ��ƽ�ƺ���==============================================
��������Northeast
���ܽ��ܣ���ǰ��ƽ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void Northeast(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 1;		//ǰ
	motor_B_IN1 = 0; motor_B_IN2 = 0;	
	motor_C_IN1 = 0; motor_C_IN2 = 1;	//ǰ
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[4]) {
									case 0xef:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================���ƽ�ƺ���==============================================
��������Southwest
���ܽ��ܣ���ǰ��ƽ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void Southwest(unsigned int PWM_IN) {
	motor_A_IN1 = 1; motor_A_IN2 = 0;		//��
	motor_B_IN1 = 0; motor_B_IN2 = 0;	
	motor_C_IN1 = 1; motor_C_IN2 = 0;	//��
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[4]) {
									case 0xbf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================�Һ�ƽ�ƺ���==============================================
��������Southeast
���ܽ��ܣ��Һ�ƽ��
����������PWM_IN������ռ�ձȣ�Խ�߳���ת��Խ��
����ֵ����
===============================================================================================*/
void Southeast(unsigned int PWM_IN) {
	motor_A_IN1 = 0; motor_A_IN2 = 0;
	motor_B_IN1 = 1; motor_B_IN2 = 0;			//��
	motor_C_IN1 = 0; motor_C_IN2 = 0;
	motor_D_IN1 = 1; motor_D_IN2 = 0;		//��
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
								psx_write_read(psx_buf);//��ȡ�ֱ�
								switch(psx_buf[4]) {
									case 0xdf:{break;}
									default: {power = 0;break;}}
								}
}


/*=========================================��Һ����ʾ��дָ���==============================================
��������write_com
���ܽ��ܣ���Һ����ʾ��дָ���
����������com����д�������
����ֵ����
===============================================================================================*/
void write_com(unsigned char com)//��1602дһ�ֽڣ�����ָ�
{
  rs=0;
  P0=com;
  delay_ms(5);
  en=1;
  delay_ms(10);
  en=0;
}


/*=========================================��Һ����ʾ��д���ݺ���==============================================
��������write_data
���ܽ��ܣ���Һ����ʾ��д���ݺ���
����������data����д�������
����ֵ����
===============================================================================================*/
void write_data(unsigned char date)//��1602дһ�ֽڣ����ݣ�
{
  rs=1;
  P0=date;
  delay_ms(5);
  en=1;
  delay_ms(5);
  en=0;
}


/*=========================================��ʱ����ʼ������==============================================
��������init_timer
���ܽ��ܣ���ʱ����ʼ������
����������
����ֵ����
===============================================================================================*/
void init_timer()
{
	TMOD=0x01;               //��ʱ��0Ϊ������ʽ1 ����ʱ��1Ϊ������ʽ0
  
	TH0=(65536-9216)/256;
  TL0=(65536-9216)%256;           //��ʱ��װ���ֵ 10ms
  EA=1;                    //�����ж� 
  ET0=1;                   //��ʱ��0���ж�
  TR0=1; 										//��ʱ��0���п���λ
  
	EX1=1;									//���ⲿ�ж�1�ж�
  IT1=1;                 //�ⲿ�ж�1������ʽΪ�����ش�����INT1�ָߵ�����Ч 
}


/*=========================================Һ����ʾ����ʼ������==============================================
��������init_lcd
���ܽ��ܣ�Һ����ʾ����ʼ������
����������
����ֵ����
===============================================================================================*/
void init_lcd()//��ʼ������
{ 
  en=0;
  rw=0;
  write_com(0x38);					   //5X7��ʾ
  write_com(0x01);         //lcd��ʼ��
  write_com(0x0c);			 //�رչ��
	write_com(0x01);         //lcd��ʼ��
	
  write_com(0x80);
  write_data('V');
  write_data(':');
  write_com(0x80+7);		 //��һ����ʾת��
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
  write_data('i');	  //�ڵڶ�����ʾzhankongbi:
  write_data(':');
  show_PWM();
}


/*=========================================��ʾת�ٺ���==============================================
��������show_speed
���ܽ��ܣ���ʾת��
����������
����ֵ����
===============================================================================================*/
void show_speed()
{
	unsigned zhuansu_x;
  write_com(0x82);
  zhuansu_x=zhuansu/4;	  //�������ڵļ�������30�õ�תÿ�� 2������/Ȧ zhuansu/4

  if(zhuansu_x/10000!=0)
   write_data(zhuansu_x/10000+0x30);						  //���ת�ٵ���λ��Ϊ0	������ʾ������ʾ�ո�
   else
   write_data(' ');

  if(zhuansu_x/1000==0)
  write_data(' ');
  else 
  write_data(zhuansu_x%10000/1000+0x30);								//���ת��С��1000 ǧλΪ�ո� ����������ʾ
  
  if(zhuansu_x/100==0)
  write_data(' ');
  else
  write_data(zhuansu_x%10000%1000/100+0x30);					   //���ת��С��100 ��λΪ�ո� ����������ʾ

    if(zhuansu_x/10==0)
  write_data(' ');
  else
  write_data(zhuansu_x%10000%1000%100/10+0x30);				  //���ת��С��10 ʮλΪ�ո� ����������ʾ

  write_data(zhuansu_x%10000%1000%100%10+0x30);
  write_com(0xd0);
}


/*=========================================��ʾռ�ձȺ���==============================================
��������show_PWM
���ܽ��ܣ���ʾռ�ձ���ֵ
����������
����ֵ����
===============================================================================================*/
void show_PWM()			//��ʾռ�ձ���ֵ
{
   write_com(0xcd);
	 write_data(PWM/100+0x30);
	
	 write_com(0xce);
	 write_data(PWM%100/10+0x30);	

	write_com(0xcf);
	 write_data(PWM%10+0x30);		
}


/*=========================================�ⲿ�ж�==============================================
��������int1
���ܽ��ܣ���¼����Ԫ�������������ڼ�����ת��
����������
����ֵ����
===============================================================================================*/
void int1()interrupt 2				//�ⲿ�ж�1 ���弼����¼�����ת�� ���תһȦzhuansu��1
{
  zhuansu++;
}


/*=========================================��ʱ���ж�==============================================
��������int2
���ܽ��ܣ�ÿ��1ms�ж�һ�Σ��ۼ�2sʱ����һ���ٶ���ʾ
����������
����ֵ����
===============================================================================================*/
void int2()interrupt 1				  //��ʱ��0 ��ʾת��
{
  TH0=(65536-9216)/256;
  TL0=(65536-9216)%256;//��ʱ10ms
  flag++;
  if(flag==200)				   //��ʱ����2s����ÿ��2s����һ���ٶ���ʾ
    {
	show_speed();					//��ʾת��
	zhuansu=0;					  //ת����0
	flag=0;
	}
}