#include "hal/tim.hpp"
TIM_HandleTypeDef htim6;

void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 159;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

// void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
// {

//   if(tim_baseHandle->Instance==TIM6)
//   {
//   /* USER CODE BEGIN TIM6_MspInit 0 */

//   /* USER CODE END TIM6_MspInit 0 */
//     /* TIM6 clock enable */
//     __HAL_RCC_TIM6_CLK_ENABLE();

//     /* TIM6 interrupt Init */
//     HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
//     HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
//   /* USER CODE BEGIN TIM6_MspInit 1 */

//   /* USER CODE END TIM6_MspInit 1 */
//   }
// }

// void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
// {

//   if(tim_baseHandle->Instance==TIM6)
//   {
//   /* USER CODE BEGIN TIM6_MspDeInit 0 */

//   /* USER CODE END TIM6_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_TIM6_CLK_DISABLE();

//     /* TIM6 interrupt Deinit */
//     HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
//   /* USER CODE BEGIN TIM6_MspDeInit 1 */

//   /* USER CODE END TIM6_MspDeInit 1 */
//   }
// }
