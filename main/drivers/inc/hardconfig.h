/**
 * @file
 * @brief Hardware config file
 */


//------- I2C bus settings -------------------------------- 

#define I2C_MASTER_SCL_IO GPIO_NUM_39      
#define I2C_MASTER_SDA_IO GPIO_NUM_38      


#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define LED_PIN GPIO_NUM_2

//------- Settings motor GPIO pins ------------------------- 

#define MOTOR_1_PIN GPIO_NUM_40
#define MOTOR_2_PIN GPIO_NUM_8
#define MOTOR_3_PIN GPIO_NUM_3
#define MOTOR_4_PIN GPIO_NUM_36

//------- Settings current and voltage measure driver ------- 

#define voltage_chanel ADC_CHANNEL_8
#define current_chanel ADC_CHANNEL_9
// Setings average filter
#define NUM_READ 8
// Current shunt amplifier gain
#define ampl_k 51

//
