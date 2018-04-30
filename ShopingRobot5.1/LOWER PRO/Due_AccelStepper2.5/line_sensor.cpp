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

