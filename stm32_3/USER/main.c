#include "AllHeader.h"
#include "pid.h"
#include "kalman.h"
#include "control.h"

u8 grop = 0;
float version = 1.1;

extern volatile uint32_t usart3_rx_count;
extern volatile uint8_t usart3_last_byte;

int main(void)
{
    bsp_init();

    Kalman_Init(&kfp_x);
    Kalman_Init(&kfp_y);

    pid_param_init(&x_pid);
    pid_param_init(&y_pid);

    Laser_Track_Init();

    USART_Cmd(USART3, ENABLE);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /*
     * 激光视觉 PID 不使用 TIM4。
     * 防止旧 PID 覆盖 x_pid.PID_out / y_pid.PID_out。
     */
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM4, DISABLE);

    while (1)
    {
        Laser_Track_Task();
    }
}
