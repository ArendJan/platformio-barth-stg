/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
/*
 * Automatically generated from STM32F091C(B-C)Tx.xml, STM32F091C(B-C)Ux.xml
 * STM32F098CCTx.xml, STM32F098CCUx.xml
 * CubeMX DB release 6.0.30
 */
#if !defined(CUSTOM_PERIPHERAL_PINS)
#include "Arduino.h"
#include "PeripheralPins.h"

/* =====
 * Notes:
 * - The pins mentioned Px_y_ALTz are alternative possibilities which use other
 *   HW peripheral instances. You can use them the same way as any other "normal"
 *   pin (i.e. analogWrite(PA7_ALT1, 128);).
 *
 * - Commented lines are alternative possibilities which are not used per default.
 *   If you change them, you will have to know what you do
 * =====
 */

//*** ADC ***

#ifdef HAL_ADC_MODULE_ENABLED
WEAK const PinMap PinMap_ADC[] = {
                                  {NC, NP, 0}};
#endif

//*** DAC ***

#ifdef HAL_DAC_MODULE_ENABLED
WEAK const PinMap PinMap_DAC[] = {{NC, NP, 0}};
#endif

//*** I2C ***

#ifdef HAL_I2C_MODULE_ENABLED
WEAK const PinMap PinMap_I2C_SDA[] = {
                                      {NC, NP, 0}};
#endif

#ifdef HAL_I2C_MODULE_ENABLED
WEAK const PinMap PinMap_I2C_SCL[] = {
                                      {NC, NP, 0}};
#endif

//*** TIM ***

#ifdef HAL_TIM_MODULE_ENABLED
WEAK const PinMap PinMap_TIM[] = {
    // {PA_15, TIM2, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM2, 1, 0)},  // TIM2_CH1
    {NC, NP, 0}};
#endif

//*** UART ***

#ifdef HAL_UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_TX[] = {
                                      {NC, NP, 0}};
#endif

#ifdef HAL_UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_RX[] = {
                                      {NC, NP, 0}};
#endif

#ifdef HAL_UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_RTS[] = {{NC, NP, 0}};
#endif

#ifdef HAL_UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_CTS[] = {{NC, NP, 0}};
#endif

//*** SPI ***

#ifdef HAL_SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_MOSI[] = {{NC, NP, 0}};
#endif

#ifdef HAL_SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_MISO[] = {{NC, NP, 0}};
#endif

#ifdef HAL_SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_SCLK[] = {{NC, NP, 0}};
#endif

#ifdef HAL_SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_SSEL[] = {{NC, NP, 0}};
#endif

//*** CAN ***

#ifdef HAL_CAN_MODULE_ENABLED
WEAK const PinMap PinMap_CAN_RD[] = {
    {PA_11, FDCAN1, STM_PIN_DATA(GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_FDCAN1)},
    {PB_12, FDCAN2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_FDCAN2)},
                                     {NC, NP, 0}};
#endif

#ifdef HAL_CAN_MODULE_ENABLED
WEAK const PinMap PinMap_CAN_TD[] = {
    {PA_12, FDCAN1, STM_PIN_DATA(GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_FDCAN1)},
    {PB_13, FDCAN2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_FDCAN2)},
                                     {NC, NP, 0}};
#endif

//*** No ETHERNET ***

//*** No QUADSPI ***

//*** No USB ***

//*** No SD ***

#endif /* !CUSTOM_PERIPHERAL_PINS */
