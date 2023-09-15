#pragma once

/**
 * @file
 * @brief Measure voltage and current battery drivers
 */

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_system.h"
#include "esp_log.h"

void ADC_init();
static bool example_adc_calibration_init(adc_unit_t, adc_atten_t, adc_cali_handle_t*);
static void example_adc_calibration_deinit(adc_cali_handle_t);
float getVoltage_V();
float getCurrent_mA();
int AverageFilter(int);