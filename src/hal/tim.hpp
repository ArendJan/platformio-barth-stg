#include <Arduino.h>


extern TIM_HandleTypeDef htim6;

void MX_TIM6_Init(void);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);


void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle);