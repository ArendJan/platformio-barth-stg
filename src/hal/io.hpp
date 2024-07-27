#pragma once
#include <stdint.h>

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

extern volatile uint32_t test_v;