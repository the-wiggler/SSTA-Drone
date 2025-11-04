#ifndef DEBUG_H
#define DEBUG_H

#include "STM32405_config.h"
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart1;

void debugInit(void);
void debugPrint();

#endif