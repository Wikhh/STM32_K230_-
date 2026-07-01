#include "k230cs.h"


#include "bsp.h"







/* 命令接收缓存 */
uint8_t RxBuffer[PTO_MAX_BUF_LEN];
/* 接收数据下标 */
uint8_t RxIndex = 0;
/* 接收状态机 */
uint8_t RxFlag = 0;
/* 新命令接收标志 */
uint8_t New_CMD_flag;
/* 新命令数据长度 */
uint8_t New_CMD_length;

/* 请求数据标志 */
uint8_t g_Request_Flag = 0;
uint8_t g_Request_ARM_ID = 0;
uint8_t g_Request_Parm = 0;




// 获取请求数据的标志
uint8_t Get_Request_Flag(void)
{
	return g_Request_Flag;
}

// 获取接收的数据
uint8_t* Get_RxBuffer(void)
{
	return (uint8_t*)RxBuffer;
}

// 获取命令长度
uint8_t Get_CMD_Length(void)
{
	return New_CMD_length;
}

// 获取命令标志
uint8_t Get_CMD_Flag(void)
{
	return New_CMD_flag;
}

// 清除命令数据和相关标志
void Clear_CMD_Flag(void)
{
	#if ENABLE_CLEAR_RXBUF
	for (uint8_t i = 0; i < PTO_MAX_BUF_LEN; i++)
	{
		RxBuffer[i] = 0;
	}
	#endif
	New_CMD_length = 0;
	New_CMD_flag = 0;
}

//清除RxBuffer中所有值为0
void Clear_RxBuffer(void)
{
	for (uint8_t i = 0; i < PTO_MAX_BUF_LEN; i++)
	{
		RxBuffer[i] = 0;
	}
}




void Upper_Data_Parse(uint8_t *data_buf, uint8_t num)
{
	#if ENABLE_CHECKSUM
	// 包头1 包头2 数量 功能 参数x 校验和
	/* 首先计算校验累加和 */
	int sum = 0;
	for (uint8_t i = 2; i < (num - 1); i++)
		sum += *(data_buf + i);
	sum = sum & 0xFF;
	/* 判断校验累加和 若不同则舍弃*/
	uint8_t recvSum = *(data_buf + num - 1);
	if (!(sum == recvSum))
	{
		DEBUG("Check sum error!, CalSum:%d, recvSum:%d\n", sum, recvSum);
		return;
	}
	#endif

	/* 判断帧尾 */
	if (!(*(data_buf) == PTO_HEAD && *(data_buf + 1) == PTO_DEVICE_ID) && *(data_buf + 5) == 0xfa)
	{
//		DEBUG("Check Head error!\n");
		return;
	}

	uint8_t func_id = *(data_buf + 2);
    uint8_t point_h = *(data_buf + 3);
    uint8_t point_l = *(data_buf + 4);
	// DEBUG("id=%02x, data_h:%02x, data_l:%02x\n", func_id, point_h, point_l);
	
	uint16_t point = (point_h << 8) | point_l;
//    DEBUG("id=%02x, data:%d\n", func_id, point);

}



/**
 * @Brief: 数据接收并保存
 * @Note: 
 * @Parm: 接收到的单字节数据
 * @Retval: 
 */
void Upper_Data_Receive(uint8_t Rx_Temp)
{
	switch (RxFlag)
	{
	case 0:
		if (Rx_Temp == PTO_HEAD)
		{
			RxBuffer[0] = PTO_HEAD;
			RxFlag = 1;
		}
		break;

	case 1:
		if (Rx_Temp == PTO_DEVICE_ID)
		{
			RxBuffer[1] = PTO_DEVICE_ID;
			RxFlag = 2;
			RxIndex = 2;
            New_CMD_length = 6;
		}
		else
		{
			RxFlag = 0;
			RxBuffer[0] = 0x0;
		}
		break;

	case 2:
		RxBuffer[RxIndex] = Rx_Temp;
		RxIndex++;
		if (RxIndex >= New_CMD_length)
		{
			New_CMD_flag = 1;
			RxIndex = 0;
			RxFlag = 0;
		}
		break;

	default:
		break;
	}
}


