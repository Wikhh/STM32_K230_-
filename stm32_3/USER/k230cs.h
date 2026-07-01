#ifndef __K230CS_H__
#define __K230CS_H__

#include "stm32f10x.h"

#define PTO_MAX_BUF_LEN           (20)


#define PTO_HEAD                  (0x55)
#define PTO_DEVICE_ID             (0xaa)

/* 功能名称定义 */
#define FUNC_AUTO_REPORT          (0x01)
#define FUNC_BEEP                 (0x02)
#define FUNC_PWM_SERVO            (0x03)
#define FUNC_PWM_SERVO_ALL        (0x04)
#define FUNC_RGB                  (0x05)
#define FUNC_RGB_EFFECT           (0x06)

#define FUNC_REPORT_SPEED         (0x0A)
#define FUNC_REPORT_MPU_RAW       (0x0B)
#define FUNC_REPORT_IMU_ATT       (0x0C)
#define FUNC_REPORT_ENCODER       (0x0D)
#define FUNC_REPORT_ICM_RAW       (0x0E)
#define FUNC_RESET_STATE          (0x0F)







/* 请求数据 */
#define FUNC_REQUEST_DATA         (0x50)
#define FUNC_VERSION              (0x51)
#define FUNC_NOW_YAW              (0x52)



/* 其他参数 */
#define SAVE_VERIFY                (0x5F)


void Upper_Data_Receive(uint8_t data);
void Upper_Data_Parse(uint8_t *data_buf, uint8_t num);
void Upper_Data_Parse_Low_Battery(uint8_t *data_buf, uint8_t num);


void Clear_CMD_Flag(void);
void Clear_RxBuffer(void);
uint8_t* Get_RxBuffer(void);
uint8_t Get_CMD_Length(void);
uint8_t Get_CMD_Flag(void);

uint8_t Get_Request_Flag(void);
void Request_Data(uint8_t request, uint8_t parm);



#endif
