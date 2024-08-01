#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_RT_Pin GPIO_PIN_15
#define LED_RT_GPIO_Port GPIOC
#define SYS_RESETN_Pin GPIO_PIN_10
#define SYS_RESETN_GPIO_Port GPIOG
#define LED_GN_Pin GPIO_PIN_0
#define LED_GN_GPIO_Port GPIOA
#define MEM_NSS_Pin GPIO_PIN_4
#define MEM_NSS_GPIO_Port GPIOA
#define BIGMEM_NCS2_Pin GPIO_PIN_5
#define BIGMEM_NCS2_GPIO_Port GPIOA
#define CAN2_ON_Pin GPIO_PIN_14
#define CAN2_ON_GPIO_Port GPIOB
#define CAN1_ON_Pin GPIO_PIN_6
#define CAN1_ON_GPIO_Port GPIOC
#define EEPROM_SDA_Pin GPIO_PIN_8
#define EEPROM_SDA_GPIO_Port GPIOA
#define EEPROM_SCL_Pin GPIO_PIN_9
#define EEPROM_SCL_GPIO_Port GPIOA
#define OneWire_SCL_Pin GPIO_PIN_15
#define OneWire_SCL_GPIO_Port GPIOA
#define MEM_SCK_Pin GPIO_PIN_10
#define MEM_SCK_GPIO_Port GPIOC
#define MEM_MISO_Pin GPIO_PIN_11
#define MEM_MISO_GPIO_Port GPIOC
#define MEM_MOSI_Pin GPIO_PIN_5
#define MEM_MOSI_GPIO_Port GPIOB
#define OneWire_SDA_Pin GPIO_PIN_7
#define OneWire_SDA_GPIO_Port GPIOB

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_TxHeaderTypeDef CAN_TxHeader; // CAN header, used for transmission
uint8_t CAN_TxData[64]; // CAN data to send
FDCAN_RxHeaderTypeDef CAN_RxHeader; // CAN header, used for reception
uint8_t CAN_RxData[64]; // CAN data, after the reception
void WriteLED_Red ( GPIO_PinState State )
{
  HAL_GPIO_WritePin(LED_RT_GPIO_Port, LED_RT_Pin, State);
}

/**
 * @brief  Set duty cycle for LED
 * @param  DutyCycle_Percent: [0..100]
 * @retval none
 */
void WriteLED_Green ( GPIO_PinState State )
{
  HAL_GPIO_WritePin(LED_GN_GPIO_Port, LED_GN_Pin, State);
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
HAL_GPIO_WritePin(LED_GN_GPIO_Port, LED_GN_Pin, GPIO_PIN_SET);
  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}


/**
  * @brief This function handles FDCAN2 interrupt 0.
  */
void FDCAN2_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 0 */

  /* USER CODE END FDCAN2_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT0_IRQn 1 */

  /* USER CODE END FDCAN2_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 1.
  */
void FDCAN2_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 0 */

  /* USER CODE END FDCAN2_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 1 */

  /* USER CODE END FDCAN2_IT1_IRQn 1 */
}


/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_CLASSIC_CAN;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 20;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 12;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 4;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 7;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RT_Pin|CAN1_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_RESETN_GPIO_Port, SYS_RESETN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GN_GPIO_Port, LED_GN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MEM_NSS_Pin|BIGMEM_NCS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN2_ON_GPIO_Port, CAN2_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = LED_RT_Pin|CAN1_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = SYS_RESETN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_RESETN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_GN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA6
                           PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = MEM_NSS_Pin|BIGMEM_NCS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15
                           PB3 PB4 PB6 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CAN2_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN2_ON_GPIO_Port, &GPIO_InitStruct);

}

/**
 * @brief  start additional Init for CAN_CANFD
 * @param  none
 * @retval none
 */
void Additional_CAN_Init ( void )
{
  // Change CAN to Silent mode
  HAL_GPIO_WritePin(CAN1_ON_GPIO_Port, CAN1_ON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAN2_ON_GPIO_Port, CAN2_ON_Pin, GPIO_PIN_SET);

     /* Start the FDCAN module */
     auto x = HAL_FDCAN_Start(&hfdcan1) ;
  if ( x != HAL_OK )
  {
    Error_Handler();
  }

  if ( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK )
  {
    Error_Handler();
  }

 /* Start the FDCAN module */
 if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
 {
   Error_Handler();
 }

 if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
 {
   Error_Handler();
 }

  // Change CAN from Silent to Active mode
  HAL_GPIO_WritePin(CAN1_ON_GPIO_Port, CAN1_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAN2_ON_GPIO_Port, CAN2_ON_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Rx FIFO 0 callback.
 * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback ( FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs )
{
  if ( (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET )
  {
    if ( hfdcan->Instance == FDCAN1 )
    {
      static uint8_t LEDRed = 0;
      /*
       * LED will toggle every time a message is received
       */
      if ( LEDRed == 0 )
        WriteLED_Red(GPIO_PIN_SET);
      else
        WriteLED_Red(GPIO_PIN_RESET);
      LEDRed = !LEDRed;

      /* Retrieve Rx messages from RX FIFO0 */
      if ( HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData) == HAL_OK )
      {
        switch ( CAN_RxHeader.Identifier )
        {
          case 0x102:
            // Add code here when CAN-ID 0x100 was recieved
            break;
        }
      }
      else
      {
        Error_Handler();
      }
    }
    
    if ( hfdcan->Instance == FDCAN2 )
    {
      static uint8_t LEDGreen = 0;
      /*
       * LED will toggle every time a message is received
       */
      if ( LEDGreen == 0 )
        WriteLED_Green(GPIO_PIN_SET);
      else
        WriteLED_Green(GPIO_PIN_RESET);
      LEDGreen = !LEDGreen;

      /* Retrieve Rx messages from RX FIFO0 */
      if ( HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData) == HAL_OK )
      {
        switch ( CAN_RxHeader.Identifier )
        {
          case 0x202:
            // Add code here when CAN-ID 0x100 was recieved
            break;
        }
      }
      else
      {
        Error_Handler();
      }
    }
  }
}

/**
 * @brief  Send a CAN-message on CAN1-bus
 *         Is the CANID smaller than 0x800 the standard ID frame format is used, otherwise the extended ID frame format is used.
 * @param  CANID: specifies the CANID with which the message is to be sent
 *         DLC: specifies the number of bytes that will be sent
 *         Byte0: Data in Byte 0
 *         Byte1: Data in Byte 1
 *         Byte2: Data in Byte 2
 *         Byte3: Data in Byte 3
 *         Byte4: Data in Byte 4
 *         Byte5: Data in Byte 5
 *         Byte6: Data in Byte 6
 *         Byte7: Data in Byte 7
 * @retval none
 */
void SendCAN1_ClassicMessage ( uint32_t CANID, uint32_t DLC, uint8_t Byte0, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5,
    uint8_t Byte6, uint8_t Byte7 )
{
  /* Prepare Tx Header */
  CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  CAN_TxHeader.MessageMarker = 0;
  CAN_TxHeader.Identifier = CANID;
  CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
  if ( CANID >= 0x800 )
    CAN_TxHeader.IdType = FDCAN_EXTENDED_ID;
  CAN_TxHeader.DataLength = DLC;

  /* Fill Data */
  CAN_TxData[0] = Byte0;
  CAN_TxData[1] = Byte1;
  CAN_TxData[2] = Byte2;
  CAN_TxData[3] = Byte3;
  CAN_TxData[4] = Byte4;
  CAN_TxData[5] = Byte5;
  CAN_TxData[6] = Byte6;
  CAN_TxData[7] = Byte7;

  /* Start the Transmission process */
  if ( HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeader, CAN_TxData) != HAL_OK )
  {
    /* Transmission request Error */
    // Error_Handler();

    HAL_Delay (2); // request to fast, add a small delay
  }
}

/**
 * @brief  Send a CAN-message on CAN2-bus
 *         Is the CANID smaller than 0x800 the standard ID frame format is used, otherwise the extended ID frame format is used.
 * @param  CANID: specifies the CANID with which the message is to be sent
 *         DLC: specifies the number of bytes that will be sent
 *         Byte0: Data in Byte 0
 *         Byte1: Data in Byte 1
 *         Byte2: Data in Byte 2
 *         Byte3: Data in Byte 3
 *         Byte4: Data in Byte 4
 *         Byte5: Data in Byte 5
 *         Byte6: Data in Byte 6
 *         Byte7: Data in Byte 7
 * @retval none
 */
void SendCAN2_ClassicMessage ( uint32_t CANID, uint32_t DLC, uint8_t Byte0, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5,
    uint8_t Byte6, uint8_t Byte7 )
{
  /* Prepare Tx Header */
  CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  CAN_TxHeader.MessageMarker = 0;
  CAN_TxHeader.Identifier = CANID;
  CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
  if ( CANID >= 0x800 )
    CAN_TxHeader.IdType = FDCAN_EXTENDED_ID;
  CAN_TxHeader.DataLength = DLC;

  /* Fill Data */
  CAN_TxData[0] = Byte0;
  CAN_TxData[1] = Byte1;
  CAN_TxData[2] = Byte2;
  CAN_TxData[3] = Byte3;
  CAN_TxData[4] = Byte4;
  CAN_TxData[5] = Byte5;
  CAN_TxData[6] = Byte6;
  CAN_TxData[7] = Byte7;

  /* Start the Transmission process */
  if ( HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CAN_TxHeader, CAN_TxData) != HAL_OK )
  {
    /* Transmission request Error */
    // Error_Handler();

    HAL_Delay (2); // request to fast, add a small delay
  }
}

/**
 * @brief  Set duty cycle for LED
 * @param  DutyCycle_Percent: [0..100]
 * @retval none
 */
void Additional_Init ( void )
{
  // Additional_TIM6_Init(); // SW Tim
  Additional_CAN_Init();
}


void setup()
{  auto x = HAL_Init();
  if ( x != HAL_OK )
  {
    Error_Handler();
  }
// MX_TIM6_Init();
//   HAL_TIM_Base_Start_IT(&htim6);

  SystemClock_Config();

  MX_GPIO_Init();
    MX_FDCAN1_Init();
  MX_FDCAN2_Init();

    // can::fdcan2.init();

    // pinMode(LED_BUILTIN, OUTPUT);
      Additional_Init();

}



void loop()
{
    delay(100);
    static uint8_t i = 0;
    uint8_t data[] = {1, 2, 3, i++};
            SendCAN1_ClassicMessage(0x100, FDCAN_DLC_BYTES_8, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF);

    // can::fdcan2.send(102, data, 4); // can in
}

// void HAL_FDCAN_RxFifo0Callback ( FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs )
// {
//   WriteLED_Green(HIGH);
// }


#ifdef __cplusplus
}
#endif
