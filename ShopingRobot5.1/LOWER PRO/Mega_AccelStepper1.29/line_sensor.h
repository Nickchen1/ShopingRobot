/**********************************************************************************************
  by Nick chen
 **********************************************************************************************/

#ifndef _LINE_SENSOR_H_
#define _LINE_SENSOR_H_

typedef enum {
  kLineSensorStateLeft = 0,
  kLineSensorStateMid,
  kLineSensorStateRight,
  kLineSensorStateEmpty,
  kLineSensorStateAll,
  kLineSensorStateLine,
  kLineSensorStateOneMid,
} LineSensorState;

class LineSensor {
  public:
    LineSensor(LineSensorState *sensor_state, int *sensor_state_real,unsigned long count_count_);

    bool UpdateSensorState(unsigned long count_count_);

    void SetSampleTime(int new_sample_time);
    LineSensorState GetSensorState(int sensor_state_real);

  private:
    LineSensorState *sensor_state_;

    int *sensor_state_real_;

    unsigned long last_time_;

    unsigned long sample_time_;


};

#endif
