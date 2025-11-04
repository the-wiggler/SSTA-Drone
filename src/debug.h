#ifndef DEBUG_H
#define DEBUG_H

#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart1;

void debugInit(void);

#endif