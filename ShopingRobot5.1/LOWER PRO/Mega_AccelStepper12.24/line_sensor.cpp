/**********************************************************************************************
  by Qu Shen
 **********************************************************************************************/

#include "Arduino.h"
#include "line_sensor.h"
#include "wheel_control.h"
LineSensor::LineSensor(LineSensorState *sensor_state, int *sensor_state_real, unsigned long count_count_) {
  sensor_state_ = sensor_state;
  sensor_state_real_ = sensor_state_real;

  sample_time_ = 5;
  last_time_ = count_count_;
}

bool LineSensor::UpdateSensorState(unsigned long count_count_) {
  unsigned long now = count_count_;
  unsigned long time_change = (now - last_time_);
  *sensor_state_ = LineSensor::GetSensorState(*sensor_state_real_);
  return true;
}

void LineSensor::SetSampleTime(int new_sample_time) {
  if (new_sample_time > 0)
  {
    sample_time_ = (unsigned long)new_sample_time;
  }
}

LineSensorState LineSensor::GetSensorState(int sensor_state_real) {
  switch (sensor_state_real) {
    case B10000:
      return kLineSensorStateLeft;
      break;

    case B11000:
      return kLineSensorStateLeft;
      break;

    case B01000:
      return kLineSensorStateLeft;
      break;

    case B00011:
      return kLineSensorStateRight;
      break;

    case B00001:
      return kLineSensorStateRight;
      break;

    case B00010:
      return kLineSensorStateRight;
      break;

    case B00110:
      return kLineSensorStateMid;
      break;

    case B01100:
      return kLineSensorStateMid;
      break;

    case B01110:
      return kLineSensorStateMid;
      break;

    case B00100:
      return kLineSensorStateOneMid;
      break;

    case B11111:
      return kLineSensorStateAll;
      break;

    case B11011:
      return kLineSensorStateLine;

    default:
      return kLineSensorStateEmpty;
      break;
  }
}

