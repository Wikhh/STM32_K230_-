#include "pid.h"
#include "AllHeader.h"
#include "kalman.h"
#include "control.h"
PID_TypeDef x_pid, y_pid;

/**
 * @brief       PID???
 * @param       ???
 * @retval      ?
 */
void pid_param_init(PID_TypeDef *pid)
{
	pid->Target = 0;//???
	pid->PID_out = 0;
    pid->Kp = 0;
    pid->Ki= 0;
	pid->Kd = 0;
    pid->Err = 0.0f;
    pid->LastErr = 0.0f;
	pid->PenultErr = 0.0f;
    pid->Integral = 0.0f;//???
	
	pid->KP_polarity = 1;
	pid->KI_polarity = 1;
	pid->KD_polarity = 1;
}

/**
 * @brief       pid??????位置式
 * @param       *PID:PID???????
 * @param       CurrentValue:?????
 * @retval      ?????
 */
float pid_calculate(PID_TypeDef *PID,float CurrentValue)
{
  PID->Err =  PID->Target - CurrentValue;
	PID->Integral += PID->Err;  
/*	????
    if( (PID->Err > 36) || (PID->Err < -36) ){
		PID->Integral = 0;
		//PID->PID_out += PID->IntegralConstant * PID->Integral10
	} 
*/    
	/*????*/
	if(PID->Integral > 7000){
		PID->Integral = 7000;
	}
	if(PID->Integral < -7000){
		PID->Integral = -7000;
	}    
    PID->PID_out = PID->Kp * PID->Err 										/*??*/
				 + PID->Ki * PID->Integral  								/*??*/
			     + PID->Kd * (PID->Err - PID->LastErr);						/*??*/
	
	PID->LastErr = PID->Err;
    return PID->PID_out;
}

/**
 * @brief       pid??????(???)增量式
 * @param       *PID:PID???????
 * @param       CurrentValue:?????
 * @retval      ?????
 */
float pid_calculate_inc(PID_TypeDef *PID,float CurrentValue)
{
	float increment_val;
    PID->Err =  PID->Target - CurrentValue;
	
    increment_val =  PID->Kp * (PID->Err - PID->LastErr) 						/*??*/
				   + PID->Ki *  PID->Err  										/*??*/
			       + PID->Kd * (PID->Err - 2*PID->LastErr + PID->PenultErr);	/*??*/

	PID->PID_out += increment_val;
	
	PID->PenultErr = PID->LastErr;
	PID->LastErr = PID->Err;
    return PID->PID_out;
}


void pid_timer_init(void) //定时器时间 (99+1)*(720+1)/72M = 1ms 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructer;
	NVIC_InitTypeDef NVIC_InitStructer;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	/*定时器TIM2初始化*/
//	TIM_DeInit(TIM4);
	TIM_TimeBaseInitStructer.TIM_Period = 10000-1;//定时周期
	TIM_TimeBaseInitStructer.TIM_Prescaler = 36-1; //分频系数
	TIM_TimeBaseInitStructer.TIM_ClockDivision = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseInitStructer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructer);
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//开启更新中断

	/*定时器中断初始化*/

	
	NVIC_InitStructer.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructer.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructer.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructer.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructer);
	TIM_Cmd(TIM4, ENABLE);//定时器使能
}

void TIM4_IRQHandler(void)/* 10ms??pid?? */
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 
	{	
      pid_calculate(&x_pid, kfp_x.out);
      pid_calculate(&y_pid, kfp_y.out);

	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}

/**
  * @brief  ?????
  * @param  PID_TypeDef *pid
  *	@note 	?
  * @retval pid->Target
  */
float get_pid_target(PID_TypeDef *pid)
{
  return pid->Target;    // ????????
}

void set_pid_target(PID_TypeDef *pid, float target)
{
	pid->Target = target;
}

/**
  * @brief  ????????????
  * @param  p:???? P
  * @param  i:???? i
  * @param  d:???? d
  *	@note 	?
  * @retval ?
  */
void set_pid_param(PID_TypeDef *pid, float p, float i, float d)
{
  pid->Kp = p * (pid->KP_polarity);    // ?????? P
	pid->Ki = i * (pid->KI_polarity);    // ?????? I
	pid->Kd = d * (pid->KD_polarity);    // ?????? D
}

void set_pid_polarity(PID_TypeDef *pid, int8_t p_polarity, int8_t i_polarity, int8_t d_polarity)
{
	pid->KP_polarity = p_polarity;
	pid->KI_polarity = i_polarity;
	pid->KD_polarity = d_polarity;
}
