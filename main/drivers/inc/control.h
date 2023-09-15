#pragma once

#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"


esp_err_t PWM_channel_config();
esp_err_t setMotor_spd(uint8_t, uint16_t);


void GPIO_init();
void onLED();
void offLED();