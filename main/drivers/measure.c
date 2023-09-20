#include "inc/measure.h"
#include "inc/hardconfig.h"


static const char *TAG = "Measure driver >>>";
static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration1;

void ADC_init(){
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, voltage_chanel, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, current_chanel, &config));

    //-------------ADC1 Calibration Init---------------//
    
    do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }
    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}

float getVoltage_V(){
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, voltage_chanel, &adc_raw[0][0]));
    if (do_calibration1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage V: %d mV", ADC_UNIT_1 + 1, voltage_chanel, voltage[0][0]);
        return (voltage[0][0] * 1.333)/1000;
    }
    else {
        ESP_LOGI(TAG, "failed to measure voltage");
        return 0;
    }
        
}

float getCurrent_mA(){
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, current_chanel, &adc_raw[0][0]));
    if (do_calibration1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage I: %d mV", ADC_UNIT_1 + 1, current_chanel, voltage[0][0]);
        float cur = (((float)voltage[0][0] / (float)ampl_k) / 0.1);
        return cur;
    }
    else {
        ESP_LOGI(TAG, "failed to measure current");
        return 0;
    }   
}

int AverageFilter(int newVal) {
  static int t = 0;
  static int vals[NUM_READ];
  static int average = 0;
  if (++t >= NUM_READ) t = 0; // перемотка t
  average -= vals[t];         // вычитаем старое
  average += newVal;          // прибавляем новое
  vals[t] = newVal;           // запоминаем в массив
  return (average / NUM_READ);
}