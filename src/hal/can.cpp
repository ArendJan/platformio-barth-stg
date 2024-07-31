#include <hal/can.hpp>
#include <hal/io.hpp>
namespace can {

FDCAN fdcan1(FDCAN1);
FDCAN fdcan2(FDCAN2);
extern "C" void HAL_CAN_MspInit(FDCAN_HandleTypeDef* can);
// FDCAN_HandleTypeDef hcan;
can::ReceiveCallback receive_callback = nullptr;
void MX_FDCAN1_Init(FDCAN_HandleTypeDef& hfdcan1)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
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
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}
/* FDCAN2 init function */
void MX_FDCAN2_Init(FDCAN_HandleTypeDef& hfdcan2)
{
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
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
    hfdcan2.Init.StdFiltersNbr = 1;
    hfdcan2.Init.ExtFiltersNbr = 0;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }
}

FDCAN::FDCAN(FDCAN_GlobalTypeDef* can_t, Baudrate baudrate)
{
    if (can_t == FDCAN1) {
        this->bus_num = 1;
    } else if (can_t == FDCAN2) {
        this->bus_num = 2;
    }
    // this->can_gpio_init();
}

void FDCAN::init()
{
    if (this->bus_num == 1) {
        MX_FDCAN1_Init(this->can_handle);
        HAL_GPIO_WritePin(CAN1_ON_GPIO_Port, CAN1_ON_Pin, GPIO_PIN_SET);
    } else {
        MX_FDCAN2_Init(this->can_handle);
        HAL_GPIO_WritePin(CAN2_ON_GPIO_Port, CAN2_ON_Pin, GPIO_PIN_SET);
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&can_handle) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&can_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    if (this->bus_num == 1) {
        HAL_GPIO_WritePin(CAN1_ON_GPIO_Port, CAN1_ON_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(CAN2_ON_GPIO_Port, CAN2_ON_Pin, GPIO_PIN_RESET);
    }
}

void FDCAN::can_gpio_init()
{
    // HAL_CAN_MspInit(&this->can_handle);
}

void FDCAN::can_init_filter()
{
    // FDCAN_FilterTypeDef filter_config;
    // filter_config.FilterIdHigh = 0;
    // filter_config.FilterIdLow = 0;
    // filter_config.FilterMaskIdHigh = 0;
    // filter_config.FilterMaskIdLow = 0;
    // filter_config.FilterFIFOAssignment = 0;
    // filter_config.FilterBank = 0;
    // filter_config.FilterMode = FDCAN_FILTERMODE_IDMASK;
    // filter_config.FilterScale = FDCAN_FILTERSCALE_32BIT;
    // filter_config.FilterActivation = ENABLE;
    // filter_config.SlaveStartFilterBank = 0;
    // if (HAL_FDCAN_ConfigFilter(&this->can_handle, &filter_config) != HAL_OK) {
    //     Error_Handler();
    // }
}

}  // namespace can
int HAL_RCC_FDCAN_CLK_ENABLED = 0;
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if (fdcanHandle->Instance == FDCAN1) {
        /* USER CODE BEGIN FDCAN1_MspInit 0 */

        /* USER CODE END FDCAN1_MspInit 0 */

        // /** Initializes the peripherals clocks
        //  */
        // PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        // PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        // if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        //     Error_Handler();
        // }

        /* FDCAN1 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**FDCAN1 GPIO Configuration
        PA11     ------> FDCAN1_RX
        PA12     ------> FDCAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
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


    } else if (fdcanHandle->Instance == FDCAN2) {
        /* USER CODE BEGIN FDCAN2_MspInit 0 */

        /* USER CODE END FDCAN2_MspInit 0 */

        /** Initializes the peripherals clocks
         */
        // PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        // PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        // if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        //     Error_Handler();
        // }

        /* FDCAN2 clock enable */
        HAL_RCC_FDCAN_CLK_ENABLED++;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 1) {
            __HAL_RCC_FDCAN_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**FDCAN2 GPIO Configuration
        PB12     ------> FDCAN2_RX
        PB13     ------> FDCAN2_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
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
    // }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{
    if (fdcanHandle->Instance == FDCAN1) {
        /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

        /* USER CODE END FDCAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN1 GPIO Configuration
        PA11     ------> FDCAN1_RX
        PA12     ------> FDCAN1_TX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* FDCAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
        /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

        /* USER CODE END FDCAN1_MspDeInit 1 */
    } else if (fdcanHandle->Instance == FDCAN2) {
        /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

        /* USER CODE END FDCAN2_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_FDCAN_CLK_ENABLED--;
        if (HAL_RCC_FDCAN_CLK_ENABLED == 0) {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }

        /**FDCAN2 GPIO Configuration
        PB12     ------> FDCAN2_RX
        PB13     ------> FDCAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13);

        /* FDCAN2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
        HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
        /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

        /* USER CODE END FDCAN2_MspDeInit 1 */
    }
}
volatile uint32_t test_v = 1234;
volatile bool x = 0;
void HAL_FDCAN_RxFifo0Callback_test(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    x = !x;
    digitalWrite(LED_BUILTIN, x);

    FDCAN_RxHeaderTypeDef CAN_RxHeader;  // CAN header, used for reception
    uint8_t CAN_RxData[64];              // CAN data, after the reception
    // test_v = RxFifo0ITs;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        test_v++;
        /* Retrieve Rx messages from RX FIFO0 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData) == HAL_OK) {
            // if (hfdcan->Instance == FDCAN1) {
            //     can::fdcan1.receive(CAN_RxHeader, CAN_RxData);
            // } else if (hfdcan->Instance == FDCAN2) {
            //     can::fdcan2.receive(CAN_RxHeader, CAN_RxData);
            // }
        } else {
            Error_Handler();
        }
    }
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    HAL_FDCAN_RxFifo0Callback(hfdcan, RxFifo1ITs);
}

/**
 * @brief This function handles FDCAN1 interrupt 0.
 */
void FDCAN1_IT0_IRQHandler(void)
{
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
    // test_v++;
    /* USER CODE END FDCAN1_IT0_IRQn 0 */
    HAL_FDCAN_IRQHandler(&can::fdcan1.can_handle);
    /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

    /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
 * @brief This function handles FDCAN1 interrupt 1.
 */
void FDCAN1_IT1_IRQHandler(void)
{
    /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */
    // test_v ++;
    /* USER CODE END FDCAN1_IT1_IRQn 0 */
    HAL_FDCAN_IRQHandler(&can::fdcan1.can_handle);
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
    HAL_FDCAN_IRQHandler(&can::fdcan2.can_handle);
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
    HAL_FDCAN_IRQHandler(&can::fdcan2.can_handle);
    /* USER CODE BEGIN FDCAN2_IT1_IRQn 1 */

    /* USER CODE END FDCAN2_IT1_IRQn 1 */
}

// extern "C" void FDHAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef* hcan)
// {
//     // if (receive_callback == nullptr) {
//     //     return;
//     // }

//     // CAN_RxHeaderTypeDef msg;
//     // uint8_t data[8];
//     // HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg, data);

//     // uint32_t id = 0;
//     // if (msg.IDE == CAN_ID_STD) {
//     //     id = msg.StdId;
//     // } else {
//     //     id = msg.ExtId;
//     // }

//     // receive_callback(id, data, msg.DLC);
// }

namespace can {

void init(Baudrate baudrate)
{
    // can_gpio_init();
    // can_init();
    // can_init_filter();

    // if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    //     Error_Handler();
    // }

    // // Change CAN from Silent to Active mode
    // HAL_GPIO_WritePin(CAN_S_GPIO_Port, CAN_S_Pin, GPIO_PIN_RESET);

    // set_baudrate(baudrate);
}

void FDCAN::set_baudrate(Baudrate baudrate)
{
    // copied from generated code, only 250k and 500k were in there:
    if (baudrate == Baudrate::Rate250k || baudrate == Baudrate::Rate500k) {
        can_handle.Init.ClockDivider = FDCAN_CLOCK_DIV1;

        can_handle.Init.NominalPrescaler = 20;
        if (baudrate == Baudrate::Rate250k) {
            can_handle.Init.NominalPrescaler = 40;
        }
        can_handle.Init.NominalSyncJumpWidth = 1;
        can_handle.Init.NominalTimeSeg1 = 12;
        can_handle.Init.NominalTimeSeg2 = 3;
        can_handle.Init.DataPrescaler = 4;
        can_handle.Init.DataSyncJumpWidth = 1;
        can_handle.Init.DataTimeSeg1 = 7;
        can_handle.Init.DataTimeSeg2 = 2;
        can_handle.Init.StdFiltersNbr = 0;
        can_handle.Init.ExtFiltersNbr = 0;
        can_handle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    }
    // hcan.Instance->BTR &= 0xFFFFFC00;
    // hcan.Instance->BTR |= static_cast<int>(baudrate);
}

void FDCAN::send(uint32_t id, uint8_t const* data, size_t size)
{
    auto x = test_v;
    ::SendCAN_ClassicMessage(this->can_handle, id, size, data);
    // CAN_TxHeaderTypeDef msg;
    // if (id < 2048) {
    //     msg.IDE = CAN_ID_STD;
    //     msg.StdId = id;
    // } else {
    //     msg.IDE = CAN_ID_EXT;
    //     msg.ExtId = id;
    // }
    // msg.DLC = size;
    // msg.RTR = CAN_RTR_DATA;
    // msg.TransmitGlobalTime = DISABLE;

    // uint32_t mailbox;
    // HAL_CAN_AddTxMessage(&hcan, &msg, const_cast<uint8_t*>(data), &mailbox);
}

void on_receive(ReceiveCallback callback)
{
    receive_callback = callback;
}

}  // namespace can

constexpr uint32_t DLC_TO_NUM(size_t DLC)
{
    switch (DLC) {
        case 0:
            return FDCAN_DLC_BYTES_0;
        case 1:
            return FDCAN_DLC_BYTES_1;
        case 2:
            return FDCAN_DLC_BYTES_2;
        case 3:
            return FDCAN_DLC_BYTES_3;
        case 4:
            return FDCAN_DLC_BYTES_4;
        case 5:
            return FDCAN_DLC_BYTES_5;
        case 6:
            return FDCAN_DLC_BYTES_6;
        case 7:
            return FDCAN_DLC_BYTES_7;
        case 8:
            return FDCAN_DLC_BYTES_8;
        default:
            return FDCAN_DLC_BYTES_0;
    }
}

void SendCAN_ClassicMessage(FDCAN_HandleTypeDef hfdcan, uint32_t CANID, uint32_t DLC, const uint8_t Bytes[8])
{
    FDCAN_TxHeaderTypeDef CAN_TxHeader = {0};  // CAN header, used for transmission
    uint8_t CAN_TxData[64] = {0};              // CAN data to send

    /* Prepare Tx Header */
    CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    CAN_TxHeader.MessageMarker = 0;
    CAN_TxHeader.Identifier = CANID;
    CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
    if (CANID >= 0x800) CAN_TxHeader.IdType = FDCAN_EXTENDED_ID;
    CAN_TxHeader.DataLength = DLC_TO_NUM(DLC);

    /* Fill Data */
    for (auto i = 0; i < DLC; i++) {
        CAN_TxData[i] = Bytes[i];
    }
    /* Start the Transmission process */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &CAN_TxHeader, CAN_TxData) != HAL_OK) {
        /* Transmission request Error */
        // Error_Handler();

        HAL_Delay(2);  // request to fast, add a small delay
        FDCAN_ErrorCountersTypeDef t;
        HAL_FDCAN_GetErrorCounters(&hfdcan, &t);
    }
}
