#include "AccelStepper.h"
#include "wheel_control.h"
#include "line_sensor.h"
#include "FlexiTimer2.h"

#define _DEBUG_PRINT_STATUS_

const int dirPin_F1 = 24;
const int stepperPin_F1 = 25;
const int dirPin_F2 = 27;
const int stepperPin_F2 = 26;
const int dirPin_B1 = 20;
const int stepperPin_B1 = 21;
const int dirPin_B2 = 23;
const int stepperPin_B2 = 22;

unsigned long count_count_;

static int Speed_F_L = 0;
static int Speed_F_R = 0;
static int Speed_B_L = 0;
static int Speed_B_R = 0;

//mega2560 speed is 5000

const int carMaxSpeed = 5000; //TO LIMIT THE SPEED-

const  int normalSpeed = 5000;
const  int slowSpeed = 3000;
const int carSpeed = normalSpeed;
const unsigned long kOneStepDelay (700);
const unsigned long kOneStepTimeoutDelay (2000);

static MotorState current_motor_state = kMotorStateStop;


const int FRONT_WIRE_SENSOR_S1_PIN (32);
const int FRONT_WIRE_SENSOR_S2_PIN (31);
const int FRONT_WIRE_SENSOR_S3_PIN (30);
const int FRONT_WIRE_SENSOR_S4_PIN (29);
const int FRONT_WIRE_SENSOR_S5_PIN (28);
const int FRONT_WIRE_SENSOR_CLP_PIN (34);
const int FRONT_WIRE_SENSOR_NEAR_PIN (35);

const int BACK_WIRE_SENSOR_S1_PIN (9);
const int BACK_WIRE_SENSOR_S2_PIN (8);
const int BACK_WIRE_SENSOR_S3_PIN (7);
const int BACK_WIRE_SENSOR_S4_PIN (6);
const int BACK_WIRE_SENSOR_S5_PIN (5);
const int BACK_WIRE_SENSOR_CLP_PIN (4);
const int BACK_WIRE_SENSOR_NEAR_PIN (3);

const int LEFT_WIRE_SENSOR_S1_PIN (15);
const int LEFT_WIRE_SENSOR_S2_PIN (16);
const int LEFT_WIRE_SENSOR_S3_PIN (17);
const int LEFT_WIRE_SENSOR_S4_PIN (18);
const int LEFT_WIRE_SENSOR_S5_PIN (19);
const int LEFT_WIRE_SENSOR_CLP_PIN (2);
const int LEFT_WIRE_SENSOR_NEAR_PIN (14);

const int RIGHT_WIRE_SENSOR_S1_PIN (41);
const int RIGHT_WIRE_SENSOR_S2_PIN (42);
const int RIGHT_WIRE_SENSOR_S3_PIN (43);
const int RIGHT_WIRE_SENSOR_S4_PIN (44);
const int RIGHT_WIRE_SENSOR_S5_PIN (45);
const int RIGHT_WIRE_SENSOR_CLP_PIN (46);
const int RIGHT_WIRE_SENSOR_NEAR_PIN (47);

AccelStepper mystepper1(1, stepperPin_F1, dirPin_F1);
AccelStepper mystepper2(1, stepperPin_F2, dirPin_F2);
AccelStepper mystepper3(1, stepperPin_B1, dirPin_B1);
AccelStepper mystepper4(1, stepperPin_B2, dirPin_B2);



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
        if (position_sensor(front_line_sensor_state_real) == kLineSensorStateMid &&position_sensor(right_line_sensor_state_real) == kLineSensorStateMid && flag == true) {
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
      break;
    }
  }
}






void FrontMove() {
  Speed_F_L = carSpeed ;
  Speed_F_R = -carSpeed ;
  Speed_B_L = carSpeed ;
  Speed_B_R = -carSpeed ;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.runSpeed();
  mystepper2.runSpeed();
  mystepper3.runSpeed();
  mystepper4.runSpeed();
}



void BackMove() {
  Speed_F_L = -carSpeed ;
  Speed_F_R = carSpeed ;
  Speed_B_L = -carSpeed ;
  Speed_B_R = carSpeed;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.runSpeed();
  mystepper2.runSpeed();
  mystepper3.runSpeed();
  mystepper4.runSpeed();
}



void LeftTurn() {
  Speed_F_L = -carSpeed ;
  Speed_F_R = -carSpeed ;
  Speed_B_L = -carSpeed ;
  Speed_B_R = -carSpeed ;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.runSpeed();
  mystepper2.runSpeed();
  mystepper3.runSpeed();
  mystepper4.runSpeed();
}

void LeftShift() {
  Speed_F_L = carSpeed ;
  Speed_F_R = carSpeed ;
  Speed_B_L = -carSpeed ;
  Speed_B_R = -carSpeed ;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.runSpeed();
  mystepper2.runSpeed();
  mystepper3.runSpeed();
  mystepper4.runSpeed();
}


void RightTurn() {
  Speed_F_L = carSpeed ;
  Speed_F_R = carSpeed ;
  Speed_B_L = carSpeed ;
  Speed_B_R = carSpeed ;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.runSpeed();
  mystepper2.runSpeed();
  mystepper3.runSpeed();
  mystepper4.runSpeed();
}


void RightShift() {
  Speed_F_L = -carSpeed ;
  Speed_F_R = -carSpeed ;
  Speed_B_L = carSpeed ;
  Speed_B_R = carSpeed ;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.runSpeed();
  mystepper2.runSpeed();
  mystepper3.runSpeed();
  mystepper4.runSpeed();
}


void StopMove() {
  Speed_F_L = -carSpeed ;
  Speed_F_R = -carSpeed ;
  Speed_B_L = carSpeed;
  Speed_B_R = carSpeed;
  mystepper1.setSpeed(Speed_F_L);
  mystepper2.setSpeed(Speed_F_R);
  mystepper3.setSpeed(Speed_B_L);
  mystepper4.setSpeed(Speed_B_R);
  mystepper1.stop();
  mystepper2.stop();
  mystepper3.stop();
  mystepper4.stop();
}






void Count() {
  count_count_++;
}
void setup() {
  // put your setup code here, to run once:

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
  Serial.begin(9600);
  mystepper1.setMaxSpeed(carMaxSpeed);
  mystepper2.setMaxSpeed(carMaxSpeed);
  mystepper3.setMaxSpeed(carMaxSpeed);
  mystepper4.setMaxSpeed(carMaxSpeed);
  FlexiTimer2::set(1, 1.0 / 1000, Count);   //设置定时器2计算1s=1000ms
  FlexiTimer2::start();                     //开启定时器2


}



void loop() {
  if (Serial.available() > 0) {
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
      case'S':
        StopMove();
        break;

    }
  }
}
