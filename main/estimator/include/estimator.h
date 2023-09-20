#pragma once

//#include "autoconf.h"
#include "main/include/stabilizer_types.h"

typedef enum {
  StateEstimatorTypeAutoSelect = 0,
  StateEstimatorTypeComplementary,
  StateEstimatorType_COUNT,
} StateEstimatorType;

typedef enum {
  MeasurementTypeTDOA,
  MeasurementTypePosition,
  MeasurementTypePose,
  MeasurementTypeDistance,
  MeasurementTypeTOF,
  MeasurementTypeAbsoluteHeight,
  MeasurementTypeFlow,
  MeasurementTypeYawError,
  MeasurementTypeSweepAngle,
  MeasurementTypeGyroscope,
  MeasurementTypeAcceleration,
  MeasurementTypeBarometer,
} MeasurementType;

typedef struct
{
  MeasurementType type;
  union
  {
    tdoaMeasurement_t tdoa;
    positionMeasurement_t position;
    poseMeasurement_t pose;
    distanceMeasurement_t distance;
    tofMeasurement_t tof;
    heightMeasurement_t height;
    flowMeasurement_t flow;
    yawErrorMeasurement_t yawError;
    sweepAngleMeasurement_t sweepAngle;
    gyroscopeMeasurement_t gyroscope;
    accelerationMeasurement_t acceleration;
    barometerMeasurement_t barometer;
  } data;
} measurement_t;

void stateEstimatorInit(StateEstimatorType estimator);
bool stateEstimatorTest(void);
void stateEstimatorSwitchTo(StateEstimatorType estimator);
void stateEstimator(state_t *state, const stabilizerStep_t stabilizerStep);
StateEstimatorType stateEstimatorGetType(void);
const char* stateEstimatorGetName();

// Support to incorporate additional sensors into the state estimate via the following functions
void estimatorEnqueue(const measurement_t *measurement);

// These helper functions simplify the caller code, but cause additional memory copies
static inline void estimatorEnqueueTDOA(const tdoaMeasurement_t *tdoa)
{
  measurement_t m;
  m.type = MeasurementTypeTDOA;
  m.data.tdoa = *tdoa;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueuePosition(const positionMeasurement_t *position)
{
  measurement_t m;
  m.type = MeasurementTypePosition;
  m.data.position = *position;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueuePose(const poseMeasurement_t *pose)
{
  measurement_t m;
  m.type = MeasurementTypePose;
  m.data.pose = *pose;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueDistance(const distanceMeasurement_t *distance)
{
  measurement_t m;
  m.type = MeasurementTypeDistance;
  m.data.distance = *distance;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueTOF(const tofMeasurement_t *tof)
{
  measurement_t m;
  m.type = MeasurementTypeTOF;
  m.data.tof = *tof;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  measurement_t m;
  m.type = MeasurementTypeAbsoluteHeight;
  m.data.height = *height;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueFlow(const flowMeasurement_t *flow)
{
  measurement_t m;
  m.type = MeasurementTypeFlow;
  m.data.flow = *flow;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueYawError(const yawErrorMeasurement_t *yawError)
{
  measurement_t m;
  m.type = MeasurementTypeYawError;
  m.data.yawError = *yawError;
  estimatorEnqueue(&m);
}

static inline void estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *sweepAngle)
{
  measurement_t m;
  m.type = MeasurementTypeSweepAngle;
  m.data.sweepAngle = *sweepAngle;
  estimatorEnqueue(&m);
}

// Helper function for state estimators
bool estimatorDequeue(measurement_t *measurement);
