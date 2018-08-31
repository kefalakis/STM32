#include "motor.h"
#include <stm32f407xx.h>
#include <string.h>
#include "uart_intr_c.h"
#include <stdio.h>
#include <stdlib.h>

extern TIM_HandleTypeDef motor1,motor2,encoder1,encoder2;
