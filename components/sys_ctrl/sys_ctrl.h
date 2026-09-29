#ifndef SYS_CTRL_H
#define SYS_CTRL_H

/**
 * @brief Initializes all system sub-components (I2C, Sensors, GPS, Telemetry)
 * and starts their respective FreeRTOS tasks.
 */
void sys_ctrl_init_and_run(void);

#endif