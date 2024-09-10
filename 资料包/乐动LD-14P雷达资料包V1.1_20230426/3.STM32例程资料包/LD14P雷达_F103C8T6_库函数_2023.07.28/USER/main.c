/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-04-25

Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-4-24

All rights reserved
***********************************************/

/*************************************
使用串口3接收LD14雷达数据，使用串口1输出接收成功的次数
接线说明：
LD14雷达接口      单片机接口
-----------------------------------
   P5V----------------5V	
   GND----------------GND
   TX-----------------B11
   PWM----------------不接(不控速)
***************************************/

#include "sys.h"
extern LiDARFrameTypeDef Pack_Data;//雷达接收的数据储存在这个变量之中
extern LidarPointStructDef Dataprocess[800];//雷达转动一圈数据储存
u16 receive_cnt;//计算成功接收数据帧次数
extern u8 data_flag;
int i;
int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组，即优先级分级个数
	                                                //NVIC_PriorityGroup_2，代表抢占优先级位数位2，可以分[0, 1, 2, 3]四级优先级
	                                                //                          响应优先级位数位2，可以分[0, 1, 2, 3]四级优先级
	delay_init();//延时初始化
	
	uart_init(115200);//串口1初始化，用于输出接收数据帧成功的次数
	
	uart3_init(230400);//雷达串口接收
	while(1)
	{
		if(data_flag == 1)
		{
			printf("***********************************");
			printf("\r\n");
			printf("Successfully received a circle of data\r\n");
			printf("***********************************");
			printf("\r\n");
			for(i = 0;i<720;i++)//遍历打印
			{
					if(float_abs(Dataprocess[i].angle-0)<0.5||float_abs(Dataprocess[i].angle-90)<0.5||float_abs(Dataprocess[i].angle-180)<0.5||float_abs(Dataprocess[i].angle-270)<0.5)
					{//打印角度在0 90 180 270 附近的点	
							 printf("Angle is %3.3f,",Dataprocess[i].angle);
							 printf("Distant is %d mm\r\n",Dataprocess[i].distance);
					}
			}
			printf("***********************************");
			printf("\r\n");printf("\r\n"); printf("\r\n");
			data_flag = 0;//标志位清空
		}
		delay_ms(5);
	}
}
//接收一帧数据角度大概为6度
