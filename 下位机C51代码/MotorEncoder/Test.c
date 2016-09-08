#include <reg52.h>
#include <intrins.h>
/****************************/
typedef char Int8;
typedef int  Int16;
typedef long Int32;
/****************************/
void delay10ms(void)   //?? -0.000000000002us
{
    unsigned char a,b;
    for(b=151;b>0;b--)
        for(a=29;a>0;a--);
}


void delay100ms(void)   //?? -0.000000000021us
{
    unsigned char a,b,c;
    for(c=35;c>0;c--)
        for(b=10;b>0;b--)
            for(a=130;a>0;a--);
}

void delay200ms(void)   //?? -0.000000000058us
{
    unsigned char a,b,c;
    for(c=193;c>0;c--)
        for(b=136;b>0;b--)
            for(a=2;a>0;a--);
}

void delay500ms(void)   //?? -0.000000000127us
{
    unsigned char a,b,c;
    for(c=122;c>0;c--)
        for(b=222;b>0;b--)
            for(a=7;a>0;a--);
    _nop_();  //if Keil,require use intrins.h
}
/****************************/
void InitUART(void)
{
    SCON  = 0x50;		        // SCON: 模式 1, 8-bit UART, 使能接收  
    TMOD |= 0x20;               // TMOD: timer 1, mode 2, 8-bit 重装
    TH1   = 0xFD;               // TH1:  重装值 9600 波特率 晶振 11.0592MHz  
    TR1   = 1;                  // TR1:  timer 1 打开                         
    EA    = 1;                  //打开总中断
    //ES    = 1;                  //打开串口中断
}                            

void SendByte(unsigned char dat)
{
	SBUF = dat;
	while(!TI);
	
	TI = 0;
}

void SendData(unsigned char *s,unsigned int len)
{
	unsigned char crc = 0xAA;
	while(len--)
  {
		SendByte(*s);
		crc += *s;
		s++;
  }
	SendByte(crc);
}

void SendStr(unsigned char *s)
{
	while(*s != '\0')
  {
		SendByte(*s);
		s++;
  }
}
/****************************/

sbit Right_Encoder_B=P1^2;
sbit Left_Encoder_B=P1^3;

Int32 Encoder_Counter[3] = {0,0,0xFFFF};
//Int32 Right_Encoder_Counter = 0;
//Int32 Left_Encoder_Counter = 0;

void InitEncoder(void)
{
	EA=1;          //全局中断使能
  EX0=1;         //
  IT0=1;				 //边沿触发
	EX1=1;         //
  IT1=1;				 //边沿触发
}

void Right_Encoder_A(void) interrupt 0 //P3^2
{
	Encoder_Counter[0] += Right_Encoder_B ? 1 : -1;
}

void Left_Encoder_A(void) interrupt 2 //P3^3
{
	Encoder_Counter[1] -= Left_Encoder_B ? 1 : -1;
}
/****************************/
//为了后期写PID算法，还是把驱动的控制器接在51上
sbit Right_Motor_IN1 = P2^0;
sbit Right_Motor_IN2 = P2^1;
sbit Left_Motor_IN1 = P2^2;
sbit Left_Motor_IN2 = P2^3;

//愚蠢的做法
sbit Pi_Control_OUT1 = P2^4;
sbit Pi_Control_OUT2 = P2^5;
sbit Pi_Control_OUT3 = P2^6;
sbit Pi_Control_OUT4 = P2^7;

void InitMotor(void)
{
	Right_Motor_IN1 = 0;
	Right_Motor_IN2 = 0;
	Left_Motor_IN1 = 0;
	Left_Motor_IN2 = 0;
	
	Pi_Control_OUT1 = 0;
	Pi_Control_OUT2 = 0;
	Pi_Control_OUT3 = 0;
	Pi_Control_OUT4 = 0;
}

/****************************/
void InitTimer0(void) //1 ms
{
		TMOD |= 0x01;
    TH0 = 0x0FC;
    TL0 = 0x66;
    EA = 1;
    ET0 = 1;
    TR0 = 1;
}

//预留PID调速 控制小车走直线
sbit Right_Motor_En = P1^4;
sbit Left_Motor_En = P1^5;
bit PWM_Val = 0;

void Timer0Interrupt(void) interrupt 1
{
	TH0 = 0x0FC;
  TL0 = 0x66;
  //add your code here!
	//delay10ms(); //10毫毫秒以内指令不会导致串口丢包
	if(Right_Motor_IN1 != Left_Motor_IN1)
	{
		EX0=0;
		EX1=0;
	}
	else
	{
		EX0=1;
		EX1=1;
	}
	Right_Motor_IN1 = Pi_Control_OUT1;
	Right_Motor_IN2 = Pi_Control_OUT2;
	Left_Motor_IN1 = Pi_Control_OUT3;
	Left_Motor_IN2 = Pi_Control_OUT4;
	//
	PWM_Val = ~PWM_Val;
	Right_Motor_En = PWM_Val;
	Left_Motor_En = PWM_Val;
}
/****************************/
sfr IPH = 0xB7;

void InitPriority()
{
	IPH = 0x15;
	PX0 = 1;
	PX1 = 1;
	PS = 0;
	PT0 = 1;
}

void main (void)
{
	InitUART();
	InitMotor();
	InitEncoder();
	InitTimer0(); //From Raspberry Pi to L298N
	//InitTimer1(); //PWM ,UART used timer1
	
	//InitPriority();

	while (1)                       
	{
		SendData((unsigned char *)Encoder_Counter,8);
		delay500ms();
  }
}