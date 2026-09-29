#pragma once
#include <stdint.h>

void motors_init(void);
void motors_set(uint8_t motor, uint16_t us);
