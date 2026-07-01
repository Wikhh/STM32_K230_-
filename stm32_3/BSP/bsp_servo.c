/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         bsp_servo.c	
* @version      V1.0
* @brief        6个舵机控制驱动源文件
* @details      
* @par History  见如下说明
*                 
* 
*/

#include "bsp_servo.h"
#include "delay.h"


ServTypdef Xserv = {
    .pwm = 1500,
    .MAXPWM = 2400,
    .MINPWM = 600,
    .MIDPWM = 1500
};

ServTypdef Yserv = {
    .pwm = 1500,
    .MAXPWM = 2300,
    .MINPWM = 700,
    .MIDPWM = 1840//中点
};
//红色激光笔
//ServTypdef Xserv = {
//    .pwm = 1425,
//	.MAXPWM = 2500,
//	.MINPWM = 500,
//	.MIDPWM = 1300//712中间点
//};

///* y???? - */
//ServTypdef Yserv = {
//    .pwm = 1425,
//	.MAXPWM = 2300,
//	.MINPWM = 1000,
//	.MIDPWM = 1670//1170中间点
//};//红色激光笔


void servo_ctr(ServTypdef *servo, int PidInput)
{
    uint16_t CompSet;

    CompSet = servo->MIDPWM + PidInput;

    if (CompSet > servo->MAXPWM)
    {
        CompSet = servo->MAXPWM;
    }
    else if (CompSet < servo->MINPWM)
    {
        CompSet = servo->MINPWM;
    }

    servo->pwm = CompSet;

    if (servo == &Xserv)
    {
        GPIO_SetBits(Servo_J1_PORT, Servo_J1_PIN);
        delay_us(CompSet);
        GPIO_ResetBits(Servo_J1_PORT, Servo_J1_PIN);
        delay_ms(20 - CompSet / 1000);
    }
    else if (servo == &Yserv)
    {
        GPIO_SetBits(Servo_J2_PORT, Servo_J2_PIN);
        delay_us(CompSet);
        GPIO_ResetBits(Servo_J2_PORT, Servo_J2_PIN);
        delay_ms(20 - CompSet / 1000);
    }
}
//void servo_ctr(ServTypdef *servo, int PidInput)
//{
//	uint16_t CompSet = servo->MIDPWM + PidInput;

//    if( CompSet > servo->MAXPWM ){
//		CompSet = servo->MAXPWM;
//	} else if ( CompSet < servo->MINPWM  ){
//		CompSet = servo->MINPWM;
//	}
//    if(servo->pwm == CompSet){/* 避免重复更改比较值 */
//        //printf("Same pwm compare num \r\n");
//        return;
//    } 
//	servo->pwm = CompSet;    
//	if( servo == &Xserv ){
//		TIM_SetCompare1(TIM1, servo->pwm);
//	}
//	else if( servo == &Yserv ){
//		TIM_SetCompare2(TIM1, servo->pwm);
//		//printf("comp:%d \t y:%.2f \t pid:%d \r\n", servo->pwm, after_kalman_y-120, PidInput);
//	}
//}

//优化成数组吧！(2维数组)
/*角度范围 0~180*/
int Angle_J[GROUP_NUM][DUOJI_NUM];

/**
* Function       Servo_J1
* @author        
* @date          
* @brief         舵机1控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J1(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J1_PORT, Servo_J1_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J1_PORT, Servo_J1_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J2
* @author        
* @date            
* @brief         舵机2控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J2(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J2_PORT, Servo_J2_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J2_PORT, Servo_J2_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J3
* @author        
* @date          
* @brief         舵机3控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J3(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J3_PORT, Servo_J3_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J3_PORT, Servo_J3_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J4
* @author        
* @date              
* @brief         舵机4控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J4(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J4_PORT, Servo_J4_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J4_PORT, Servo_J4_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J5
* @author        
* @date             
* @brief         舵机5控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J5(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J5_PORT, Servo_J5_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J5_PORT, Servo_J5_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J6
* @author        
* @date             
* @brief         舵机6控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J6(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J6_PORT, Servo_J6_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J6_PORT, Servo_J6_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J7
* @author        
* @date             
* @brief         舵机7控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J7(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J7_PORT, Servo_J7_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J7_PORT, Servo_J7_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       Servo_J8
* @author        
* @date             
* @brief         舵机8控制函数
* @param[in]     v_iAngle 角度：0~180°
* @param[out]    void
* @retval        void
* @par History   无
*/
void Servo_J8(int v_iAngle)/*定义一个脉冲函数，用来模拟方式产生PWM值*/
{
	int pulsewidth;    						//定义脉宽变量

	pulsewidth = (v_iAngle * 11) + 500;			//将角度转化为500-2480 的脉宽值

	GPIO_SetBits(Servo_J8_PORT, Servo_J8_PIN );		//将舵机接口电平置高
	delay_us(pulsewidth);					//延时脉宽值的微秒数

	GPIO_ResetBits(Servo_J8_PORT, Servo_J8_PIN );	//将舵机接口电平置低
	delay_ms(20 - pulsewidth/1000);			//延时周期内剩余时间
}

/**
* Function       front_detection
* @author        
* @date             
* @brief         云台舵机向前
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void front_detection()
{
	int i = 0;
  	//此处循环次数减少，为了增加小车遇到障碍物的反应速度
  	for(i=0; i <= 15; i++) 						//产生PWM个数，等效延时以保证能转到响应角度
  	{
    	Servo_J1(90);						//模拟产生PWM
  	}
}

/**
* Function       left_detection
* @author        
* @date             
* @brief         云台舵机向左
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void left_detection()
{
	int i = 0;
	for(i = 0; i <= 15; i++) 						//产生PWM个数，等效延时以保证能转到响应角度
	{
		Servo_J1(175);					//模拟产生PWM
	}
}

/**
* Function       right_detection
* @author        
* @date             
* @brief         云台舵机向右
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   无
*/
void right_detection()
{
	int i = 0;
	for(i = 0; i <= 15; i++) 						//产生PWM个数，等效延时以保证能转到响应角度
	{
		Servo_J1(5);						//模拟产生PWM
	}
}

