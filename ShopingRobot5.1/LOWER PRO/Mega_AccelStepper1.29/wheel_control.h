/**********************************************************************************************
  by Nick chen
 **********************************************************************************************/

#ifndef _WHEEL_CONTROL_H_
#define _WHEEL_CONTROL_H_


typedef enum {
  kDirectionEast = 0,
  kDirectionWest,
  kDirectionSouth,
  kDirectionNorth,
  kDirectionSouthEast,
  kDirectionSouthWest,
  kDirectionNorthEast,
  kDirectionNorthWest,
} Direction;

typedef enum {
  kMotorStateForward = 0,
  kMotorStateBackward,
  kMotorStateTurnLeft,
  kMotorStateTurnRight,
  kMotorStateLeftward,
  kMotorStateRightward,
  kMotorStateStop,
  kMotorStateSlowForward,
  kMotorStateSlowBackward,
  kMotorStateLeftFroward,
  kMotorStateLeftBackward,
  kMotorStateRightFroward,
  kMotorStateRightBackward
} MotorState;

#endif
