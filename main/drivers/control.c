#include "inc/control.h"
#include "inc/hardconfig.h"



#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (2500) // Frequency in Hertz. Set frequency at 5 kHz

esp_err_t PWM_channel_config(){
    // PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    //Engeen 1
    //PWM channel 0 configuration
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_1_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    //Engeen 2
    //PWM channel 1 configuration
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_2_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    //Engeen 3
    //PWM channel 2 configuration
    ledc_channel_config_t ledc_channel_2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_3_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    //Engeen 4
    //PWM channel 3 configuration
    ledc_channel_config_t ledc_channel_3 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_3,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_4_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    if((ledc_channel_config(&ledc_channel_0) 
        && ledc_channel_config(&ledc_channel_1) 
        && ledc_channel_config(&ledc_channel_2) 
        && ledc_channel_config(&ledc_channel_3)) == ESP_OK){
            return ESP_OK;
        }
    else return ESP_ERR_INVALID_STATE;
}


esp_err_t setMotor_spd(uint8_t nunberMotor, uint16_t spd){
    if(spd > 8191) spd = 8191;
    switch (nunberMotor){
        case 1:
            return ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, spd);
            break;
        case 2:
            return ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, spd);
            break;
        case 3:
            return ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, spd);
            break;
        case 4:
            return ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, spd);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
        break;
    }
}

void GPIO_init(){
     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}
void onLED(){
     gpio_set_level(LED_PIN,1);
}
void offLED(){
    gpio_set_level(LED_PIN,0);
}
