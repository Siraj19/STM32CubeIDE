/*
 * i2c_slave.c
 *
 *  Created on: Jun 2, 2024
 *      Author: Siraj
 */

#include "main.h"
#include "i2c_slave.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

#define RxSize 1
uint8_t RxData[RxSize];
uint8_t count = 0;
//Callback function when device completed listening
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//Again device waiting for listening
	HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if(TransferDirection == I2C_DIRECTION_TRANSMIT)
  {
	  HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, RxData, RxSize, I2C_FIRST_AND_LAST_FRAME);
  }
  else{
	  Error_Handler();
  }

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(count%=1){
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	}
	else if(count%=0){
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	}

	count++;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(&hi2c1);
}
