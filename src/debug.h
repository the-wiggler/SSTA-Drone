#ifndef DEBUG_H
#define DEBUG_H

#include "SpeedyBee_F405_conf.h"

UART_HandleTypeDef huart1;

void debugInit(void);
void debugPrint(const char *format, ...);

#endif