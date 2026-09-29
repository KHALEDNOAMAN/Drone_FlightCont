#pragma once
#include <stdint.h>
#include <stdbool.h>

#define IBUS_CHANNELS 14

void ibus_init(int rx_gpio);
bool ibus_read(uint16_t ch[IBUS_CHANNELS], bool *failsafe);
