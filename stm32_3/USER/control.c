#include "control.h"
#include "kalman.h"
#include "AllHeader.h"
#include "bsp_usart.h"
#include "pid.h"
#include "kalman.h"

#include <stdint.h>

#define LASER_FRAME_HEAD      0xFF
#define LASER_FRAME_END       0xFE

#define LASER_CMD_VALID       0x01
#define LASER_CMD_LOST        0x02

// 误差死区，单位：像素
#define LASER_DEAD_ZONE_X     3.0f
#define LASER_DEAD_ZONE_Y     3.0f



/*
 * 像素 -> PWM 微秒的映射系数
 *
 * 你现在 2.0 太小。
 * 建议先从 10 开始试。
 */
#define SERVO_US_PER_PIXEL_X   4.0f
#define SERVO_US_PER_PIXEL_Y   4.0f

/*
 * 方向控制：
 * 如果方向已经对，就保持 1。
 * 如果某个方向反了，就改成 -1。
 */
#define SERVO_X_DIR            -1.0f
#define SERVO_Y_DIR            -1.0f

/*
 * 最大舵机偏移，单位 us。
 * 这个值要根据你的 PWM 安全范围调整。
 */
#define SERVO_X_OFFSET_MIN      -700.0f
#define SERVO_X_OFFSET_MAX       700.0f

#define SERVO_Y_OFFSET_MIN      -600.0f
#define SERVO_Y_OFFSET_MAX       600.0f

/*
 * 每收到一帧，舵机最多变化多少 us。
 * 48fps 下，8us/frame 大约是 384us/s。
 */
#define SERVO_MAX_STEP_X         8.0f
#define SERVO_MAX_STEP_Y         8.0f

/*
 * 误差滤波系数。
 * 越小越稳，越大越跟手。
 */
#define LASER_ERR_ALPHA          0.25f




// 丢失多少次后停止输出
#define LASER_LOST_LIMIT      10

typedef struct
{
    int16_t error_x;
    int16_t error_y;
    uint8_t cmd;
} LaserPacket_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;

    float output;

    float output_min;
    float output_max;

    float integral_min;
    float integral_max;

    float max_step;

    float dead_zone;

    uint8_t inited;
} LaserPid_t;

uint8_t mode_flag = 0;
uint16_t rec_step_point[4][2] = {0};
uint16_t rec_step_p[1][2] = {0};
uint8_t step_flag = 0;


static float laser_err_x_f = 0.0f;
static float laser_err_y_f = 0.0f;

static float servo_offset_x = 0.0f;
static float servo_offset_y = 0.0f;

static uint8_t laser_filter_inited = 0;
static uint8_t laser_lost_count = 0;


static LaserPid_t laser_pid_x;
static LaserPid_t laser_pid_y;



static void LaserPid_Init(
    LaserPid_t *pid,
    float kp,
    float ki,
    float kd,
    float output_min,
    float output_max,
    float integral_min,
    float integral_max,
    float max_step,
    float dead_zone
)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->last_error = 0.0f;

    pid->output = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;

    pid->max_step = max_step;
    pid->dead_zone = dead_zone;

    pid->inited = 0;
}


static float LaserPid_Update(LaserPid_t *pid, float error, float dir)
{
    float derivative;
    float step;

    /*
     * 死区内认为已经重合。
     * 注意：不要让舵机回中，而是保持当前 output。
     */
    if (abs_float(error) <= pid->dead_zone)
    {
        pid->integral = 0.0f;
        pid->last_error = 0.0f;
        pid->inited = 0;
        return pid->output;
    }

    if (pid->inited == 0)
    {
        pid->last_error = error;
        pid->inited = 1;
    }

    pid->integral += error;

    pid->integral = limit_float(
        pid->integral,
        pid->integral_min,
        pid->integral_max
    );

    derivative = error - pid->last_error;

    /*
     * PID 输出的是“本帧 PWM 增量”
     */
    step = pid->kp * error
         + pid->ki * pid->integral
         + pid->kd * derivative;

    step = dir * step;

    step = limit_float(
        step,
        -pid->max_step,
        pid->max_step
    );

    pid->output += step;

    pid->output = limit_float(
        pid->output,
        pid->output_min,
        pid->output_max
    );

    pid->last_error = error;

    return pid->output;
}


static void LaserPid_Reset_Dynamic(LaserPid_t *pid)
{
    /*
     * 目标丢失时，保留 output，避免云台乱跳。
     * 只清动态项。
     */
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->inited = 0;
}

void Laser_Track_Init(void)
{
    /*
     * 初始参数，保守优先。
     * 单位大致是 us / pixel / frame。
     */
    LaserPid_Init(
        &laser_pid_x,
        0.25f,      // Kp
        0.003f,     // Ki
        0.08f,      // Kd
        SERVO_X_OFFSET_MIN,
        SERVO_X_OFFSET_MAX,
        -80.0f,
        80.0f,
        SERVO_MAX_STEP_X,
        LASER_DEAD_ZONE_X
    );

    LaserPid_Init(
        &laser_pid_y,
        0.25f,      // Kp
        0.003f,     // Ki
        0.08f,      // Kd
        SERVO_Y_OFFSET_MIN,
        SERVO_Y_OFFSET_MAX,
        -80.0f,
        80.0f,
        SERVO_MAX_STEP_Y,
        LASER_DEAD_ZONE_Y
    );

    laser_err_x_f = 0.0f;
    laser_err_y_f = 0.0f;
    laser_filter_inited = 0;
    laser_lost_count = 0;
}


static float limit_float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}



static float abs_float(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float approach_float(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step)
    {
        delta = max_step;
    }
    else if (delta < -max_step)
    {
        delta = -max_step;
    }

    return current + delta;
}

uint8_t LaserPacket_Decode(RingBuff_t *ringbuff, LaserPacket_t *packet)
{
    static uint8_t state = 0;
    static uint8_t buf[8];
    uint8_t data;

    while (Read_RingBuff(ringbuff, &data) == RINGBUFF_OK)
    {
        switch (state)
        {
            case 0:
                if (data == LASER_FRAME_HEAD)
                {
                    buf[0] = data;
                    state = 1;
                }
                break;

            case 1:
                if ((data == LASER_CMD_VALID) || (data == LASER_CMD_LOST))
                {
                    buf[1] = data;
                    state = 2;
                }
                else
                {
                    state = 0;
                }
                break;

            case 2:
                buf[2] = data;
                state = 3;
                break;

            case 3:
                buf[3] = data;
                state = 4;
                break;

            case 4:
                buf[4] = data;
                state = 5;
                break;

            case 5:
                buf[5] = data;
                state = 6;
                break;

            case 6:
                buf[6] = data;
                state = 7;
                break;

            case 7:
            {
                uint8_t checksum;

                buf[7] = data;
                state = 0;

                if (buf[7] != LASER_FRAME_END)
                {
                    return 0;
                }

                checksum = (uint8_t)((buf[1] + buf[2] + buf[3] + buf[4] + buf[5]) & 0xFF);

                if (checksum != buf[6])
                {
                    return 0;
                }

                packet->cmd = buf[1];

                packet->error_x = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
                packet->error_y = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

                return 1;
            }

            default:
                state = 0;
                break;
        }
    }

    return 0;
}

void Laser_Track_Task(void)
{
    LaserPacket_t packet;
    LaserPacket_t latest_packet;

    uint8_t has_packet = 0;

    float error_x;
    float error_y;

    float out_x;
    float out_y;

    /*
     * 只处理最新一帧，避免旧数据造成滞后。
     */
    while (LaserPacket_Decode(&Uart3_RingBuff, &packet))
    {
        latest_packet = packet;
        has_packet = 1;
    }

    if (has_packet == 0)
    {
        return;
    }

    if (latest_packet.cmd == LASER_CMD_LOST)
    {
        laser_lost_count++;

        if (laser_lost_count > LASER_LOST_LIMIT)
        {
            laser_filter_inited = 0;

            LaserPid_Reset_Dynamic(&laser_pid_x);
            LaserPid_Reset_Dynamic(&laser_pid_y);
        }

        /*
         * 丢失目标时保持当前输出，不乱回中。
         */
        return;
    }

    laser_lost_count = 0;

    /*
     * K230 现在发送的是：
     * error_x = red_x - green_x
     * error_y = red_y - green_y
     */
    error_x = (float)latest_packet.error_x;
    error_y = (float)latest_packet.error_y;

    /*
     * 视觉误差低通滤波
     */
    if (laser_filter_inited == 0)
    {
        laser_err_x_f = error_x;
        laser_err_y_f = error_y;
        laser_filter_inited = 1;
    }
    else
    {
        laser_err_x_f += LASER_ERR_ALPHA * (error_x - laser_err_x_f);
        laser_err_y_f += LASER_ERR_ALPHA * (error_y - laser_err_y_f);
    }

    /*
     * 视觉 PID。
     * 输出是 PWM 偏移量，单位 us。
     */
    out_x = LaserPid_Update(&laser_pid_x, laser_err_x_f, SERVO_X_DIR);
    out_y = LaserPid_Update(&laser_pid_y, laser_err_y_f, SERVO_Y_DIR);

    /*
     * 继续复用 x_pid.PID_out / y_pid.PID_out 作为舵机偏移量。
     * 注意：这里不是用旧 PID 算出来的，只是沿用输出变量。
     */
    x_pid.PID_out = (int)out_x;
    y_pid.PID_out = (int)out_y;

   /* printf(
        "ex=%.1f ey=%.1f fx=%.1f fy=%.1f ox=%d oy=%d\r\n",
        error_x,
        error_y,
        laser_err_x_f,
        laser_err_y_f,
        x_pid.PID_out,
        y_pid.PID_out
    );*/
}

void set_target(float x, float y)
{
    x_pid.Target = x;
    y_pid.Target = y;
}

void mode_2(void)
{

		
int a_Err=(int)x_pid.Err, b_Err=(int)y_pid.Err;
		
			get_point2_p();
		
		
	
			  if(abs(a_Err) + abs(b_Err) < 800){/* 声光提示 */
          BEEP_ON;
        } 
				else 
					{
          BEEP_OFF;
        }
			set_target(rec_step_p[0][0],rec_step_p[0][1]);	
//				printf("a_Err:%d,b_Err:%d \r\n",a_Err,b_Err);
				
		
}



void go_to_point(uint16_t now_x, uint16_t now_y, uint16_t targ_x, uint16_t targ_y)
{
    uint16_t i;
    float step_num = 1000;/* 1000步 */
    float step_x, step_y; /* 步进值  */
    float going_x = now_x, going_y = now_y; /* 当前目标值*/
    
    step_x = (targ_x - now_x) / step_num;
    step_y = (targ_y - now_y) / step_num;

    for(i = 0; i < step_num; i++){

        
			kfp_x.source = step_x;
			kfp_y.source = step_y;
			kfp_x.out = KalmanFilter(&kfp_x, kfp_x.source);
			kfp_y.out = KalmanFilter(&kfp_y, kfp_y.source);
	
		
			going_x += kfp_x.out;
      going_y += kfp_y.out;
				
      set_target(going_x, going_y); 


    }
}



uint8_t get_point2_p()   //处理坐标函数
{
	uint8_t lable, load_1, load_2 ,load_3 ,ret=0;
	if(DataDecode1(&Uart3_RingBuff,&lable, &load_1, &load_2,&load_3) == 0){
	 if(lable == 0x00){
			rec_step_p[0][0] = load_1 + load_2 + load_3 ;
//			ret=1;
//		 printf("x:%2x\r\n",x);
		} else if(lable == 0xff){
			
			rec_step_p[0][1] = load_1 + load_2 + load_3 ;
//			ret=1;
//		 printf("y:%2x\r\n",y);
		} 

	return ret;
}
}

uint8_t get_point1_p()
{
	uint8_t lable, load_1, load_2,load_3, ret = 0;
	if(DataDecode1(&Uart3_RingBuff,&lable, &load_1, &load_2, &load_3) == 0){
		
		  if(lable == 0x10){
			rec_step_point[3][0] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[3][0]: %d\r\n", rec_step_point[3][0]);
		} else if(lable == 0x11){
			rec_step_point[3][1] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[3][1]: %d\r\n", rec_step_point[3][1]);
		} else if(lable == 0x12){
			rec_step_point[0][0] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[0][0]: %d\r\n", rec_step_point[0][0]);
		} else if(lable == 0x13){
			rec_step_point[0][1] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[0][1]: %d\r\n", rec_step_point[0][1]);
		} else if(lable == 0x14){
			rec_step_point[1][0] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[1][0]: %d\r\n", rec_step_point[1][0]);
		} else if(lable == 0x15){
			rec_step_point[1][1] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[1][1]: %d\r\n", rec_step_point[1][1]);
		} else if(lable == 0x16){
			rec_step_point[2][0] = (load_1 + load_2 + load_3);
//            printf("rec_step_point[2][0]: %d\r\n", rec_step_point[2][0]);
		} else if(lable == 0x17){
			rec_step_point[2][1] = (load_1 + load_2 + load_3);
//           printf("rec_step_point[2][1]: %d\r\n", rec_step_point[2][1]);
            mode_flag = 4;			
		}
		
	}
	return ret;
}