
#ifndef __POWER_DISTRIBUTION_H__
#define __POWER_DISTRIBUTION_H__

#include "stabilizer_types.h"


void powerDistributionInit(void);
bool powerDistributionTest(void);

/**
 * @brief Calculate the power (thrust) of each motor based on the output from the controller
 *
 * @param control Data from the controller
 * @param motorThrustUncapped The desired thrust
 */
void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped);

/**
 * @brief Cap the thrust for the motors when out side of the valid range [0 - UINT16_MAX]. The platform specific
 * implementation can chose to cap the trust in a way that provides graceful degradation, for instance prioritizing
 * attitude over thrust.
 *
 * @param motorThrustBatCompUncapped The desired thrust for the motors
 * @param motorPwm The capped thrust
 */
void powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm);

/**
 * Returns a 1 when motor 'id' gives thrust, returns 0 otherwise
 */
int powerDistributionMotorType(uint32_t id);

/**
 * Returns the stop ratio of the motor 'id'
 */
uint16_t powerDistributionStopRatio(uint32_t id);

/**
 * @brief Get the current setting for idle thrust
 *
 * @return uint32_t The idle thrust
 */
uint32_t powerDistributionGetIdleThrust();

#endif //__POWER_DISTRIBUTION_H__
