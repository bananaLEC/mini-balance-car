#include "mpu6050_test.h"
#include "app_usart1.h"
#include "app_MPU6050.h"
#include "delay.h"
#include <math.h>
#include "app_attitude.h"

// MPU6050 与姿态测试：手持小车翻转，串口看 roll,pitch,yaw，单位 0.01 度
void MPU6050_Test(void)
{
	App_USART1_Init();
	App_MPU6050_Init();
	
	while(1)
	{
		App_MPU6050_Update();
		
		float ax = App_MPU6050_GetAx();
		float ay = App_MPU6050_GetAy();
		float az = App_MPU6050_GetAz();
		
		float temperature = App_MPU6050_GetTemperature();

		float gx = App_MPU6050_GetGx();
		float gy = App_MPU6050_GetGy();
		float gz = App_MPU6050_GetGz();
		


		My_USART_Printf(USART1, "%d,%d,%d\r\n",
	              (int)(App_Attitude_GetRoll()  * 100.0f),
	              (int)(App_Attitude_GetPitch() * 100.0f),
	              (int)(App_Attitude_GetYaw()   * 100.0f));
		
		
		//My_USART_Printf(USART1,"%f,%f,%f,%f,%f,%f,%f\n",ax, ay, az, temperature, gx, gy, gz);
		
		Delay(10);
	}
	
}

