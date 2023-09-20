#pragma once

#include "main/include/stabilizer_types.h"

void estimatorComplementaryInit(void);
bool estimatorComplementaryTest(void);
void estimatorComplementary(state_t *state, const stabilizerStep_t stabilizerStep);

bool estimatorComplementaryEnqueueTOF(const tofMeasurement_t *tof);
