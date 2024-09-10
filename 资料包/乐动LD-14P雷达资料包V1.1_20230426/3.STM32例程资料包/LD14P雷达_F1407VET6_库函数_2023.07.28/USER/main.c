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
使用串口5接收LD14雷达数据，使用串口1输出接收成功的次数
接线说明：
LD14雷达接口      单片机接口
-----------------------------------
   P5V----------------5V	
   GND----------------GND
   TX-----------------D2
   PWM----------------不接不控速)
***************************************/

#include "sys.h"
#include "system.h"

extern LiDARFrameTypeDef Pack_Data;//雷达接收的数据储存在这个变量之中
extern LidarPointStructDef Dataprocess[800];//雷达转动一圈数据储存
u16 receive_cnt;//计算成功接收数据帧次数
extern u8 data_flag;
int i;
int main(void)
{ 
	systemInit();					//Initialization function     初始化函数
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





