/**********************************************************************************************
  by Nick chen
 **********************************************************************************************/

#include "AccelStepper.h"
#include "wheel_control.h"
#include "line_sensor.h"
#include "DueTimer.h"

//#define _DEBUG_PRINT_STATUS_

//mega2560 speed is 5000
static int Speed_F_L (0);
static int Speed_F_R (0);
static int Speed_B_L (0);
static int Speed_B_R (0);

const int carMaxSpeed(1500); //TO LIMIT THE SPEED-
const int Step_delay(200);
const int normalSpeed (1500);
const  int slowSpeed(3000);
const int carSpeed = normalSpeed;

const unsigned long kOneStepDelay (700);
const unsigned long kOneStepTimeoutDelay (2000);
static MotorState current_motor_state = kMotorStateStop;
static Direction current_car_direction = kDirectionNorth;  //FACE TO B SHELF
unsigned long count_count_;

const int dirPin_B_L (48);
const int stepperPin_B_L (49);
const int enPin_B_L (50);

const int dirPin_B_R (53);
const int stepperPin_B_R (52);
const int enPin_B_R (51);

const int dirPin_F_L (35);
const int stepperPin_F_L (36);
const int enPin_F_L (37);

const int dirPin_F_R (40);
const int stepperPin_F_R (39);
const int enPin_F_R (38);

const int FRONT_WIRE_SENSOR_S1_PIN (41);
const int FRONT_WIRE_SENSOR_S2_PIN (42);
const int FRONT_WIRE_SENSOR_S3_PIN (43);
const int FRONT_WIRE_SENSOR_S4_PIN (44);
const int FRONT_WIRE_SENSOR_S5_PIN (45);
const int FRONT_WIRE_SENSOR_CLP_PIN (46);
const int FRONT_WIRE_SENSOR_NEAR_PIN (47);

const int BACK_WIRE_SENSOR_S1_PIN (28);
const int BACK_WIRE_SENSOR_S2_PIN (29);
const int BACK_WIRE_SENSOR_S3_PIN (30);
const int BACK_WIRE_SENSOR_S4_PIN (31);
const int BACK_WIRE_SENSOR_S5_PIN (32);
const int BACK_WIRE_SENSOR_CLP_PIN (33);
const int BACK_WIRE_SENSOR_NEAR_PIN (34);

const int LEFT_WIRE_SENSOR_S1_PIN (27);
const int LEFT_WIRE_SENSOR_S2_PIN (26);
const int LEFT_WIRE_SENSOR_S3_PIN (25);
const int LEFT_WIRE_SENSOR_S4_PIN (24);
const int LEFT_WIRE_SENSOR_S5_PIN (23);
const int LEFT_WIRE_SENSOR_CLP_PIN (22);
const int LEFT_WIRE_SENSOR_NEAR_PIN (21);

const int RIGHT_WIRE_SENSOR_S1_PIN (14);
const int RIGHT_WIRE_SENSOR_S2_PIN (15);
const int RIGHT_WIRE_SENSOR_S3_PIN (16);
const int RIGHT_WIRE_SENSOR_S4_PIN (17);
const int RIGHT_WIRE_SENSOR_S5_PIN (18);
const int RIGHT_WIRE_SENSOR_CLP_PIN (19);
const int RIGHT_WIRE_SENSOR_NEAR_PIN (20);


AccelStepper Back_Left_Motor(1, stepperPin_B_L, dirPin_B_L);
AccelStepper Back_Right_Motor(1, stepperPin_B_R, dirPin_B_R);
AccelStepper Front_Left_Motor(1, stepperPin_F_L, dirPin_F_L);
AccelStepper Front_Right_Motor(1, stepperPin_F_R, dirPin_F_R);


static LineSensorState front_line_sensor_state(kLineSensorStateMid), back_line_sensor_state(kLineSensorStateMid), left_line_sensor_state(kLineSensorStateMid), right_line_sensor_state(kLineSensorStateMid);
static int front_line_sensor_state_real(B01110), back_line_sensor_state_real(B01110), left_line_sensor_state_real(B01110), right_line_sensor_state_real(B01110);


static LineSensor front_line_sensor(&front_line_sensor_state, &front_line_sensor_state_real, count_count_);
static LineSensor back_line_sensor(&back_line_sensor_state, &back_line_sensor_state_real, count_count_);
static LineSensor left_line_sensor(&left_line_sensor_state, &left_line_sensor_state_real, count_count_);
static LineSensor right_line_sensor(&right_line_sensor_state, &right_line_sensor_state_real, count_count_);


static char UpdateLineSensor() {
  front_line_sensor_state_real = digitalRead(FRONT_WIRE_SENSOR_S5_PIN) << 4 |
                                 digitalRead(FRONT_WIRE_SENSOR_S4_PIN) << 3 |
                                 digitalRead(FRONT_WIRE_SENSOR_S3_PIN) << 2 |
                                 digitalRead(FRONT_WIRE_SENSOR_S2_PIN) << 1 |
                                 digitalRead(FRONT_WIRE_SENSOR_S1_PIN);

  back_line_sensor_state_real = digitalRead(BACK_WIRE_SENSOR_S5_PIN) << 4 |
                                digitalRead(BACK_WIRE_SENSOR_S4_PIN) << 3 |
                                digitalRead(BACK_WIRE_SENSOR_S3_PIN) << 2 |
                                digitalRead(BACK_WIRE_SENSOR_S2_PIN) << 1 |
                                digitalRead(BACK_WIRE_SENSOR_S1_PIN);

  left_line_sensor_state_real =  digitalRead(LEFT_WIRE_SENSOR_S5_PIN) << 4 |
                                 digitalRead(LEFT_WIRE_SENSOR_S4_PIN) << 3 |
                                 digitalRead(LEFT_WIRE_SENSOR_S3_PIN) << 2 |
                                 digitalRead(LEFT_WIRE_SENSOR_S2_PIN) << 1 |
                                 digitalRead(LEFT_WIRE_SENSOR_S1_PIN);

  right_line_sensor_state_real = digitalRead(RIGHT_WIRE_SENSOR_S5_PIN) << 4 |
                                 digitalRead(RIGHT_WIRE_SENSOR_S4_PIN) << 3 |
                                 digitalRead(RIGHT_WIRE_SENSOR_S3_PIN) << 2 |
                                 digitalRead(RIGHT_WIRE_SENSOR_S2_PIN) << 1 |
                                 digitalRead(RIGHT_WIRE_SENSOR_S1_PIN);

}

LineSensorState position_sensor(int line_sensor_state_real) {
  switch (line_sensor_state_real) {
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


void ForwardStep(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    flag = true;
    unsigned long start_time = count_count_;
    for (;;) {
      UpdateLineSensor();
      if (count_count_ - start_time > kOneStepDelay ) {
        if (position_sensor(left_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;
# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
        else if (position_sensor(right_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
      switch (position_sensor(front_line_sensor_state_real)) {
        case kLineSensorStateLeft:
          LeftTurn();
          break;
        case kLineSensorStateRight:
          RightTurn();
          break;
        default:
          FrontMove();
          break;
      }
    }
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}



void BackwardStep(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    unsigned long start_time = count_count_;
    flag = true;
    for (;;) {
      UpdateLineSensor();
      if (count_count_ - start_time > kOneStepDelay) {
        if (position_sensor(left_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
        else if (position_sensor(right_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          // StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
      switch (position_sensor(back_line_sensor_state_real)) {
        case kLineSensorStateLeft:
          LeftTurn();
          break;
        case kLineSensorStateRight:
          RightTurn();
          break;
        default:
          BackMove();
          break;
      }

    }
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}


void LeftTurnStep(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    unsigned long start_time = count_count_;
    flag = true;
    for (;;) {
      UpdateLineSensor();
      if (count_count_ - start_time > kOneStepDelay) {
        if (position_sensor(left_line_sensor_state_real) == kLineSensorStateMid && position_sensor(back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          // Serial.print(whiteline);
#endif
          break;
        }
      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }

      LeftTurn();
    }
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }
}




void RightTurnStep(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    unsigned long start_time = count_count_;
    flag = true;
    for (;;) {
      UpdateLineSensor();
      if (count_count_ - start_time > kOneStepDelay) {
        if (position_sensor(right_line_sensor_state_real) == kLineSensorStateMid && position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          // StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }

      }
      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();
# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
      RightTurn();
    }
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }
}


void LeftShiftStep(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    unsigned long start_time = count_count_;
    flag = true;
    for (;;) {
      UpdateLineSensor();
      if (count_count_ - start_time > kOneStepDelay) {
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
        else if (position_sensor(back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
      }
      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
      switch (position_sensor(left_line_sensor_state_real)) {
        case kLineSensorStateLeft:
          LeftTurn();
          break;
        case kLineSensorStateRight:
          RightTurn();
          break;
        default:
          LeftShift();
          break;
      }

    }
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}


void RightShiftStep(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    unsigned long start_time = count_count_;
    flag = true;
    for (;;) {
      UpdateLineSensor();
      if (count_count_ - start_time > kOneStepDelay) {
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          // Serial.print(whiteline);
#endif
          break;
        }
        else if (position_sensor(back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          // StopMove();
          whiteline++;
          flag = false;

# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }
      }
      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        // Serial.print(whiteline);
#endif
        break;
      }
      switch (position_sensor(right_line_sensor_state_real)) {
        case kLineSensorStateLeft:
          LeftTurn();
          break;
        case kLineSensorStateRight:
          RightTurn();
          break;
        default:
          RightShift();
          break;
      }

    }
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }
}



void Front_Left_45(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    flag = true;
    unsigned long start_time = count_count_;
    for (;;) {
      if (count_count_ - start_time > kOneStepDelay ) {
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && position_sensor(back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;
# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }

      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
      Left_Up_45();
    }

    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}


void Front_Right_45(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    flag = true;
    unsigned long start_time = count_count_;
    for (;;) {
      if (count_count_ - start_time > kOneStepDelay ) {
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && position_sensor( back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;
# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }

      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
      Right_Up_45();
    }

    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}


void Back_Left_45(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    flag = true;
    unsigned long start_time = count_count_;
    for (;;) {
      if (count_count_ - start_time > kOneStepDelay ) {
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && position_sensor(back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;
# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }

      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
    }
    Left_Down_45();
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}


void Back_Right_45(int count) {
  bool flag = false;
  int whiteline = 0;
  for (;;) {
    flag = true;
    unsigned long start_time = count_count_;
    for (;;) {
      if (count_count_ - start_time > kOneStepDelay ) {
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid && position_sensor(back_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
          //StopMove();
          whiteline++;
          flag = false;
# ifdef _DEBUG_PRINT_STATUS_
          Serial.println("Timeout Triggered.");
          //Serial.print(whiteline);
#endif
          break;
        }

      }

      if (count_count_ - start_time > kOneStepTimeoutDelay) {
        StopMove();

# ifdef _DEBUG_PRINT_STATUS_
        Serial.println("Line Triggered.");
        //Serial.print(whiteline);
#endif
        break;
      }
    }
    Right_Down_45();
    if (whiteline >= count) {
      StopMove();
      delay(Step_delay);
      break;
    }
  }

}


void TurnToNorth() {
  switch (current_car_direction) {
    case kDirectionNorth:
      current_car_direction = kDirectionNorth;
      break;
    case kDirectionWest:
      RightTurnStep(1);
      current_car_direction = kDirectionNorth;
      break;
    case  kDirectionEast:
      LeftTurnStep(1);
      current_car_direction = kDirectionNorth;
      break;
    case  kDirectionSouth:
      LeftTurnStep(2);
      current_car_direction = kDirectionNorth;
      break;
    default:
      break;

  }
}

void TurnToSouth() {
  switch (current_car_direction) {
    case kDirectionNorth:
      LeftTurnStep(2);
      current_car_direction = kDirectionSouth;
      break;
    case kDirectionWest:
      LeftTurnStep(1);
      current_car_direction = kDirectionSouth;
      break;
    case  kDirectionEast:
      RightTurnStep(1);
      current_car_direction = kDirectionSouth;
      break;
    case  kDirectionSouth:
      current_car_direction = kDirectionSouth;
      break;
    default:
      break;
  }
}

void TurnToWest() {
  switch (current_car_direction) {
    case kDirectionNorth:
      LeftTurnStep(1);
      current_car_direction = kDirectionWest;
      break;
    case kDirectionWest:
      current_car_direction = kDirectionWest;
      break;
    case  kDirectionEast:
      LeftTurnStep(2);
      current_car_direction = kDirectionWest;
      break;
    case  kDirectionSouth:
      RightTurnStep(1);
      current_car_direction = kDirectionWest;
      break;
    default:
      break;
  }
}

void TurnToEast() {
  switch (current_car_direction) {
    case kDirectionNorth:
      RightTurnStep(1);
      current_car_direction = kDirectionEast;
      break;
    case kDirectionWest:
      LeftTurnStep(2);
      current_car_direction = kDirectionEast;
      break;
    case  kDirectionEast:
      current_car_direction = kDirectionEast;
      break;
    case  kDirectionSouth:
      LeftTurnStep(1);
      current_car_direction = kDirectionEast;
      break;
    default:
      break;
  }
}



void FrontMove() {
  Speed_B_L = carSpeed ;
  Speed_B_R = -carSpeed ;
  Speed_F_L = carSpeed ;
  Speed_F_R = -carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}



void BackMove() {
  Speed_B_L = -carSpeed ;
  Speed_B_R = carSpeed ;
  Speed_F_L = -carSpeed ;
  Speed_F_R = carSpeed;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}



void LeftTurn() {
  Speed_B_L = -carSpeed ;
  Speed_B_R = -carSpeed ;
  Speed_F_L = -carSpeed ;
  Speed_F_R = -carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}

void LeftShift() {
  Speed_B_L = carSpeed ;
  Speed_B_R = carSpeed ;
  Speed_F_L = -carSpeed ;
  Speed_F_R = -carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}


void RightTurn() {
  Speed_B_L = carSpeed ;
  Speed_B_R = carSpeed ;
  Speed_F_L = carSpeed ;
  Speed_F_R = carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}


void RightShift() {
  Speed_B_L = -carSpeed ;
  Speed_B_R = -carSpeed ;
  Speed_F_L = carSpeed ;
  Speed_F_R = carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}


void Left_Up_45() {
  Speed_B_L = carSpeed;
  Speed_B_R = 0;
  Speed_F_L = 0;
  Speed_F_R = -carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}


void Right_Up_45() {
  Speed_B_L = 0;
  Speed_B_R = -carSpeed ;
  Speed_F_L = carSpeed ;
  Speed_F_R = 0 ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}


void Left_Down_45() {
  Speed_B_L = 0;
  Speed_B_R = carSpeed ;
  Speed_F_L = -carSpeed ;
  Speed_F_R = 0 ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}



void Right_Down_45() {
  Speed_B_L = -carSpeed;
  Speed_B_R = 0;
  Speed_F_L = 0;
  Speed_F_R = carSpeed ;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.runSpeed();
  Back_Right_Motor.runSpeed();
  Front_Left_Motor.runSpeed();
  Front_Right_Motor.runSpeed();
}



void StopMove() {
  Speed_B_L = -carSpeed ;
  Speed_B_R = -carSpeed ;
  Speed_F_L = carSpeed;
  Speed_F_R = carSpeed;
  Back_Left_Motor.setSpeed(Speed_B_L);
  Back_Right_Motor.setSpeed(Speed_B_R);
  Front_Left_Motor.setSpeed(Speed_F_L);
  Front_Right_Motor.setSpeed(Speed_F_R);
  Back_Left_Motor.stop();
  Back_Right_Motor.stop();
  Front_Left_Motor.stop();
  Front_Right_Motor.stop();
}


void WheelEnable(bool flag) {
  //THE CAR WHEEL ENABLE(defalut: NOT ENABLE,LOW IS ENABLE)
  if (flag == true) {
    digitalWrite(enPin_B_L, LOW);
    digitalWrite(enPin_B_R, LOW);
    digitalWrite(enPin_F_L, LOW);
    digitalWrite(enPin_F_R, LOW);
  }
  else {
    digitalWrite(enPin_B_L, HIGH);
    digitalWrite(enPin_B_R, HIGH);
    digitalWrite(enPin_F_L, HIGH);
    digitalWrite(enPin_F_R, HIGH);
  }

}



void Count() {
  count_count_++;
}
void setup() {
  // put your setup code here, to run once:

  pinMode(enPin_B_L, OUTPUT);
  pinMode(enPin_B_R, OUTPUT);
  pinMode(enPin_F_L, OUTPUT);
  pinMode(enPin_F_R, OUTPUT);

  pinMode(dirPin_B_L, OUTPUT);
  pinMode(dirPin_B_R, OUTPUT);
  pinMode(dirPin_F_L, OUTPUT);
  pinMode(dirPin_F_R, OUTPUT);

  WheelEnable(false);


  Serial.begin(9600);
  pinMode(FRONT_WIRE_SENSOR_S1_PIN, INPUT);
  pinMode(FRONT_WIRE_SENSOR_S2_PIN, INPUT);
  pinMode(FRONT_WIRE_SENSOR_S3_PIN, INPUT);
  pinMode(FRONT_WIRE_SENSOR_S4_PIN, INPUT);
  pinMode(FRONT_WIRE_SENSOR_S5_PIN, INPUT);
  pinMode(FRONT_WIRE_SENSOR_CLP_PIN, INPUT);
  pinMode(FRONT_WIRE_SENSOR_NEAR_PIN, INPUT);

  pinMode(BACK_WIRE_SENSOR_S1_PIN, INPUT);
  pinMode(BACK_WIRE_SENSOR_S2_PIN, INPUT);
  pinMode(BACK_WIRE_SENSOR_S3_PIN, INPUT);
  pinMode(BACK_WIRE_SENSOR_S4_PIN, INPUT);
  pinMode(BACK_WIRE_SENSOR_S5_PIN, INPUT);
  pinMode(BACK_WIRE_SENSOR_CLP_PIN, INPUT);
  pinMode(BACK_WIRE_SENSOR_NEAR_PIN, INPUT);

  pinMode(LEFT_WIRE_SENSOR_S1_PIN, INPUT);
  pinMode(LEFT_WIRE_SENSOR_S2_PIN, INPUT);
  pinMode(LEFT_WIRE_SENSOR_S3_PIN, INPUT);
  pinMode(LEFT_WIRE_SENSOR_S4_PIN, INPUT);
  pinMode(LEFT_WIRE_SENSOR_S5_PIN, INPUT);
  pinMode(LEFT_WIRE_SENSOR_CLP_PIN, INPUT);
  pinMode(LEFT_WIRE_SENSOR_NEAR_PIN, INPUT);

  pinMode(RIGHT_WIRE_SENSOR_S1_PIN, INPUT);
  pinMode(RIGHT_WIRE_SENSOR_S2_PIN, INPUT);
  pinMode(RIGHT_WIRE_SENSOR_S3_PIN, INPUT);
  pinMode(RIGHT_WIRE_SENSOR_S4_PIN, INPUT);
  pinMode(RIGHT_WIRE_SENSOR_S5_PIN, INPUT);
  pinMode(RIGHT_WIRE_SENSOR_CLP_PIN, INPUT);
  pinMode(RIGHT_WIRE_SENSOR_NEAR_PIN, INPUT);


  Back_Left_Motor.setMaxSpeed(carMaxSpeed);
  Back_Right_Motor.setMaxSpeed(carMaxSpeed);
  Front_Left_Motor.setMaxSpeed(carMaxSpeed);
  Front_Right_Motor.setMaxSpeed(carMaxSpeed);

  //MAKE SURE THR DIRECTION IS RIGHT!!
  digitalWrite(dirPin_B_L, LOW);
  digitalWrite(dirPin_B_R, LOW);
  digitalWrite(dirPin_F_L, LOW);
  digitalWrite(dirPin_F_R, LOW);

  Timer.getAvailable().attachInterrupt(Count).start(1000); //- Set the timer period (in microseconds)


}



void loop() {
  if (Serial.available() > 0) {
    WheelEnable(true);
    //Serial.println("Acknowledge");
    int count = Serial.parseInt();
    char action = Serial.read();
    switch (action) {
      case'F':
        ForwardStep(count);
        break;
      case'B':
        BackwardStep(count);
        break;
      case'L':
        LeftTurnStep(count);
        break;
      case'l':
        LeftShiftStep(count);
        break;
      case'R':
        RightTurnStep(count);
        break;
      case'r':
        RightShiftStep(count);
        break;
      case'q':
        TurnToNorth();
        break;
      case'p':
        TurnToSouth();
        break;
      case'z':
        TurnToWest();
        break;
      case'm':
        TurnToEast();
        break;
      case's':
        WheelEnable(false);
        break;
      case'S':
        WheelEnable(true);
        break;
    }
    Serial.println("Complete!");
  }
  
}

