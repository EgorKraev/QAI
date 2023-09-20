
#ifndef STABILIZER_H_
#define STABILIZER_H_

#include <stdbool.h>
#include <stdint.h>

#include "main/estimator/include/estimator.h"

/**
 * Initialize the stabilizer subsystem and launch the stabilizer loop task.
 * The stabilizer loop task will wait on systemWaitStart() before running.
 */
void stabilizerInit(StateEstimatorType estimator);

/**
 * Test the stabilizer subsystem. Calls test for all the stabilizer related
 * sensors.
 * @return True if all test has passed. False otherwise.
 */
bool stabilizerTest(void);

#endif /* STABILIZER_H_ */
