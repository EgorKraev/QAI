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

#define MOTOR_M1  1
#define MOTOR_M2  2
#define MOTOR_M3  3
#define MOTOR_M4  4
//------- Settings current and voltage measure driver ------- 

#define voltage_chanel ADC_CHANNEL_8
#define current_chanel ADC_CHANNEL_9
// Setings average filter
#define NUM_READ 8
// Current shunt amplifier gain
#define ampl_k 51

//
#define DEFAULT_BAT_LOW_VOLTAGE                   6.4f
#define DEFAULT_BAT_CRITICAL_LOW_VOLTAGE          6.0f
#define DEFAULT_BAT_LOW_DURATION_TO_TRIGGER_SEC   5

// Default value for system shutdown in minutes after radio silence.
// Requires kbuild config ENABLE_AUTO_SHUTDOWN to be activated.
#define DEFAULT_SYSTEM_SHUTDOWN_TIMEOUT_MIN       5

// Default PID gains
#define PID_ROLL_RATE_KP  250.0
#define PID_ROLL_RATE_KI  500.0
#define PID_ROLL_RATE_KD  2.5
#define PID_ROLL_RATE_KFF 0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  250.0
#define PID_PITCH_RATE_KI  500.0
#define PID_PITCH_RATE_KD  2.5
#define PID_PITCH_RATE_KFF 0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  120.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_KFF 0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  6.0
#define PID_ROLL_KI  3.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_KFF 0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  6.0
#define PID_PITCH_KI  3.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_KFF 0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  6.0
#define PID_YAW_KI  1.0
#define PID_YAW_KD  0.35
#define PID_YAW_KFF 0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0

#define PID_VEL_X_KP 25.0f
#define PID_VEL_X_KI 1.0f
#define PID_VEL_X_KD 0.0f
#define PID_VEL_X_KFF 0.0f

#define PID_VEL_Y_KP 25.0f
#define PID_VEL_Y_KI 1.0f
#define PID_VEL_Y_KD 0.0f
#define PID_VEL_Y_KFF 0.0f

#define PID_VEL_Z_KP 25.0f
#define PID_VEL_Z_KI 15.0f
#define PID_VEL_Z_KD 0.0f
#define PID_VEL_Z_KFF 0.0f

#define PID_VEL_Z_KP_BARO_Z_HOLD 3.0f
#define PID_VEL_Z_KI_BARO_Z_HOLD 1.0f
#define PID_VEL_Z_KD_BARO_Z_HOLD 1.5f
#define PID_VEL_Z_KFF_BARO_Z_HOLD 0.0f

#define PID_VEL_ROLL_MAX 20.0f
#define PID_VEL_PITCH_MAX 20.0f
#define PID_VEL_THRUST_BASE 36000.0f
#define PID_VEL_THRUST_BASE_BARO_Z_HOLD 38000.0f
#define PID_VEL_THRUST_MIN 20000.0f

#define PID_POS_X_KP 2.0f
#define PID_POS_X_KI 0.0f
#define PID_POS_X_KD 0.0f
#define PID_POS_X_KFF 0.0f

#define PID_POS_Y_KP 2.0f
#define PID_POS_Y_KI 0.0f
#define PID_POS_Y_KD 0.0f
#define PID_POS_Y_KFF 0.0f

#define PID_POS_Z_KP 2.0f
#define PID_POS_Z_KI 0.5f
#define PID_POS_Z_KD 0.0f
#define PID_POS_Z_KFF 0.0f

#define PID_POS_VEL_X_MAX 1.0f
#define PID_POS_VEL_Y_MAX 1.0f
#define PID_POS_VEL_Z_MAX 1.0f


// Attitude PID control filter settings
#ifndef ATTITUDE_LPF_CUTOFF_FREQ 
    #define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#endif
#ifndef ATTITUDE_LPF_ENABLE
    #define ATTITUDE_LPF_ENABLE false
#endif
#ifndef ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ
    #define ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ 30.0f
#endif
#ifndef ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ
    #define ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ 30.0f
#endif
#ifndef ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ
    #define ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ 30.0f
#endif
#ifndef ATTITUDE_RATE_LPF_ENABLE
    #define ATTITUDE_RATE_LPF_ENABLE false
#endif
#ifndef ATTITUDE_RATE_FF_YAW
    #define ATTITUDE_RATE_FF_YAW 0.0f
#endif
#ifndef YAW_MAX_DELTA
    #define YAW_MAX_DELTA     0.0f
#endif

#ifndef PID_POS_XY_FILT_ENABLE
    #define PID_POS_XY_FILT_ENABLE true
#endif
#ifndef PID_POS_XY_FILT_CUTOFF
    #define PID_POS_XY_FILT_CUTOFF 20.0f
#endif
#ifndef PID_POS_Z_FILT_ENABLE
    #define PID_POS_Z_FILT_ENABLE true
#endif
#ifndef PID_POS_Z_FILT_CUTOFF
    #define PID_POS_Z_FILT_CUTOFF 20.0f
#endif
#ifndef PID_VEL_XY_FILT_ENABLE
    #define PID_VEL_XY_FILT_ENABLE true
#endif
#ifndef PID_VEL_XY_FILT_CUTOFF
    #define PID_VEL_XY_FILT_CUTOFF 20.0f
#endif
#ifndef PID_VEL_Z_FILT_ENABLE
    #define PID_VEL_Z_FILT_ENABLE true
#endif
#ifndef PID_VEL_Z_FILT_CUTOFF
    #define PID_VEL_Z_FILT_CUTOFF 20.0f
#endif
#ifndef PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD
    #define PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD 0.7 f
#endif

// Tumble detection enabled by default
#ifndef SUPERVISOR_TUMBLE_CHECK_ENABLE
    #define SUPERVISOR_TUMBLE_CHECK_ENABLE true
#endif


// Health test parameters
#ifndef HEALTH_BRUSHED_ON_PERIOD_MSEC
    #define HEALTH_BRUSHED_ON_PERIOD_MSEC 50
#endif
#ifndef HEALTH_BRUSHED_OFF_PERIOD_MSEC
    #define HEALTH_BRUSHED_OFF_PERIOD_MSEC 950
#endif
#ifndef HEALTH_BRUSHED_VARIANCE_START_MSEC
    #define HEALTH_BRUSHED_VARIANCE_START_MSEC 0
#endif
#ifndef HEALTH_BRUSHED_PROP_ON_PERIOD_PWM_RATIO
    #define HEALTH_BRUSHED_PROP_ON_PERIOD_PWM_RATIO 0xFFFF
#endif
#ifndef HEALTH_BRUSHED_BAT_ON_PERIOD_PWM_RATIO
    #define HEALTH_BRUSHED_BAT_ON_PERIOD_PWM_RATIO 40000
#endif

#ifndef HEALTH_BRUSHLESS_ON_PERIOD_MSEC
    #define HEALTH_BRUSHLESS_ON_PERIOD_MSEC 2000
#endif
#ifndef HEALTH_BRUSHLESS_OFF_PERIOD_MSEC
    #define HEALTH_BRUSHLESS_OFF_PERIOD_MSEC 1000
#endif
#ifndef HEALTH_BRUSHLESS_VARIANCE_START_MSEC
    #define HEALTH_BRUSHLESS_VARIANCE_START_MSEC 1000
#endif

//constants of math
#define SPEED_OF_LIGHT (299792458.0)
#define GRAVITY_MAGNITUDE (9.81f)
#define M_PI   3.14159265358979323846
#define M_PI_F   (3.14159265358979323846f)
#define M_1_PI_F (0.31830988618379067154f)
#define M_PI_2_F (1.57079632679f)
#define ARCMINUTE (M_PI_F / 10800.0f)
#define CF_MASS (0.027f) // in kg
