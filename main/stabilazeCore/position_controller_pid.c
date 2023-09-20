#include <math.h>
#include "num.h"

//#include "log.h"
//#include "param.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"
#include "main/include/hardconfig.h"
#include "main/stabilazeCore/Include/imu_types.h"
//#include "platform_defaults.h"


struct pidAxis_s {
  PidObject pid;

  stab_mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

  struct pidAxis_s pidX;
  struct pidAxis_s pidY;
  struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rLimit = PID_VEL_ROLL_MAX;
static float pLimit = PID_VEL_PITCH_MAX;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xVelMax = PID_POS_VEL_X_MAX;
static float yVelMax = PID_POS_VEL_Y_MAX;
static float zVelMax = PID_POS_VEL_Z_MAX;
static float velMaxOverhead = 1.10f;

static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
static bool posFiltEnable = PID_POS_XY_FILT_ENABLE;
static bool velFiltEnable = PID_VEL_XY_FILT_ENABLE;
static float posFiltCutoff = PID_POS_XY_FILT_CUTOFF;
static float velFiltCutoff = PID_VEL_XY_FILT_CUTOFF;
static bool posZFiltEnable = PID_POS_Z_FILT_ENABLE;
static bool velZFiltEnable = PID_VEL_Z_FILT_ENABLE;
static float posZFiltCutoff = PID_POS_Z_FILT_CUTOFF;
#if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD;
#else
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF;
#endif

#ifndef UNIT_TEST
static struct this_s this = {
  .pidVX = {
    .pid = {
      .kp = PID_VEL_X_KP,
      .ki = PID_VEL_X_KI,
      .kd = PID_VEL_X_KD,
      .kff = PID_VEL_X_KFF,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .pid = {
      .kp = PID_VEL_Y_KP,
      .ki = PID_VEL_Y_KI,
      .kd = PID_VEL_Y_KD,
      .kff = PID_VEL_Y_KFF,
    },
    .pid.dt = DT,
  },
  #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
    .pidVZ = {
      .pid = {
        .kp = PID_VEL_Z_KP_BARO_Z_HOLD,
        .ki = PID_VEL_Z_KI_BARO_Z_HOLD,
        .kd = PID_VEL_Z_KD_BARO_Z_HOLD,
        .kff = PID_VEL_Z_KFF_BARO_Z_HOLD,
      },
      .pid.dt = DT,
    },
  #else
    .pidVZ = {
      .pid = {
        .kp = PID_VEL_Z_KP,
        .ki = PID_VEL_Z_KI,
        .kd = PID_VEL_Z_KD,
        .kff = PID_VEL_Z_KFF,
      },
      .pid.dt = DT,
    },
  #endif
  .pidX = {
    .pid = {
      .kp = PID_POS_X_KP,
      .ki = PID_POS_X_KI,
      .kd = PID_POS_X_KD,
      .kff = PID_POS_X_KFF,
    },
    .pid.dt = DT,
  },

  .pidY = {
    .pid = {
      .kp = PID_POS_Y_KP,
      .ki = PID_POS_Y_KI,
      .kd = PID_POS_Y_KD,
      .kff = PID_POS_Y_KFF,
    },
    .pid.dt = DT,
  },

  .pidZ = {
    .pid = {
      .kp = PID_POS_Z_KP,
      .ki = PID_POS_Z_KI,
      .kd = PID_POS_Z_KD,
      .kff = PID_POS_Z_KFF,
    },
    .pid.dt = DT,
  },
  #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
    .thrustBase = PID_VEL_THRUST_BASE_BARO_Z_HOLD,
  #else
    .thrustBase = PID_VEL_THRUST_BASE,
  #endif
  .thrustMin  = PID_VEL_THRUST_MIN,
};
#endif

void positionControllerInit()
{
  pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.pid.kp, this.pidX.pid.ki, this.pidX.pid.kd,
      this.pidX.pid.kff, this.pidX.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.pid.kp, this.pidY.pid.ki, this.pidY.pid.kd,
      this.pidY.pid.kff, this.pidY.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.pid.kp, this.pidZ.pid.ki, this.pidZ.pid.kd,
      this.pidZ.pid.kff, this.pidZ.pid.dt, POSITION_RATE, posZFiltCutoff, posZFiltEnable);

  pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.pid.kp, this.pidVX.pid.ki, this.pidVX.pid.kd,
      this.pidVX.pid.kff, this.pidVX.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.pid.kp, this.pidVY.pid.ki, this.pidVY.pid.kd,
      this.pidVY.pid.kff, this.pidVY.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.pid.kp, this.pidVZ.pid.ki, this.pidVZ.pid.kd,
      this.pidVZ.pid.kff, this.pidVZ.pid.dt, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, true);
}

float state_body_x, state_body_y, state_body_vx, state_body_vy;

void positionController(float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state)
{
  this.pidX.pid.outputLimit = xVelMax * velMaxOverhead;
  this.pidY.pid.outputLimit = yVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // this value is below 0.5
  this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f)  * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;

  float globalvx = setpoint->velocity.x;
  float globalvy = setpoint->velocity.y;

  //X, Y
  Axis3f setpoint_velocity;
  setpoint_velocity.x = setpoint->velocity.x;
  setpoint_velocity.y = setpoint->velocity.y;
  setpoint_velocity.z = setpoint->velocity.z;
  if (setpoint->mode.x == modeAbs) {
    setpoint_velocity.x = runPid(state_body_x, &this.pidX, setp_body_x, DT);
  } else if (!setpoint->velocity_body) {
    setpoint_velocity.x = globalvx * cosyaw + globalvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint_velocity.y = runPid(state_body_y, &this.pidY, setp_body_y, DT);
  } else if (!setpoint->velocity_body) {
    setpoint_velocity.y = globalvy * cosyaw - globalvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint_velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);
  }

  velocityController(thrust, attitude, &setpoint_velocity, state);
}

void velocityController(float* thrust, attitude_t *attitude, const Axis3f* setpoint_velocity,
                                                             const state_t *state)
{
  this.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  this.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  this.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //this.pidVZ.pid.outputLimit = (this.thrustBase - this.thrustMin) / thrustScale;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  state_body_vx = state->velocity.x * cosyaw + state->velocity.y * sinyaw;
  state_body_vy = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

  // Roll and Pitch
  attitude->pitch = -runPid(state_body_vx, &this.pidVX, setpoint_velocity->x, DT);
  attitude->roll = -runPid(state_body_vy, &this.pidVY, setpoint_velocity->y, DT);

  attitude->roll  = constrain(attitude->roll,  -rLimit, rLimit);
  attitude->pitch = constrain(attitude->pitch, -pLimit, pLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint_velocity->z, DT);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw*thrustScale + this.thrustBase;
  // Check for minimum thrust
  if (*thrust < this.thrustMin) {
    *thrust = this.thrustMin;
  }
    // saturate
  *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void positionControllerResetAllPID()
{
  pidReset(&this.pidX.pid);
  pidReset(&this.pidY.pid);
  pidReset(&this.pidZ.pid);
  pidReset(&this.pidVX.pid);
  pidReset(&this.pidVY.pid);
  pidReset(&this.pidVZ.pid);
}

void positionControllerResetAllfilters() {
  filterReset(&this.pidX.pid, POSITION_RATE, posFiltCutoff, posFiltEnable);
  filterReset(&this.pidY.pid, POSITION_RATE, posFiltCutoff, posFiltEnable);
  filterReset(&this.pidZ.pid, POSITION_RATE, posZFiltCutoff, posZFiltEnable);
  filterReset(&this.pidVX.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&this.pidVY.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&this.pidVZ.pid, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}


