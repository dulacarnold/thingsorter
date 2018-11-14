
// Sweep
// by BARRAGAN <http://barraganstudio.com>

// http://arduiniana.org/libraries/pwmservo/

//   Board                     SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   -----                     -----------   -----------   -----------
//   Arduino Uno, Duemilanove       9            10          (none)
//   Arduino Mega                  11            12            13
//   Sanguino                      13            12          (none)
//   Teensy 1.0                    17            18            15
//   Teensy 2.0                    14            15             4
//   Teensy++ 1.0 or 2.0           25            26            27
//   Teensy LC & 3.x                 (all PWM pins are usable)

// #include <PWMServo.h>
#include <Adafruit_TiCoServo.h>
#include <Arduino.h>


#define NUM_POS_SERVOS 3
#define NUM_SORT_SERVOS 1
#define NUM_SERVOS NUM_POS_SERVOS + NUM_SORT_SERVOS

#define SORT_OPEN_ANGLE 0
#define SORT_CLOSED_ANGLE 70

#define SRV_POS_MIN 1280 // 1200
#define SRV_POS_MAX 1720 // 1800
#define SRV_ANG_MIN 1000 // For the orientation servo
#define SRV_ANG_MAX 2000
#define SRV_DIR 1
#define SRV_CNTR 90
#define SRV_STOP_DIST 3
#define SRV_SLOW_DIST 12
#define SPD_SLOW 15
#define SPD_FAST 70
#define SPD_MAX 90
#define SPD_GAIN 0.01F

#define ELEV_DECEL 1000 // Time elevator spends on slow before stop

#define POSE_OFFSET 30L // Minimum pose offset
#define POS_TO_ANGLE 0.342 // = 360/1051

#define NUM_MSGS 2
#define MSG_STOP 0
#define MSG_STATUS 1
#define MSG_STOP_FREQ 5000
#define MSG_STAT_FREQ 2000

// Servo PWM pins
#define SRV0_PWM 5
#define SRV1_PWM 6
#define SRV2_PWM 7
#define SRV3_PWM 8

// Elevator control pins
#define ELEV_SLOW 23
#define ELEV_FAST 24

// Absolute position sensors on servos, camera servos first and then
// sort servos.
#define POS0_INT 18
#define POS1_INT 2
#define POS2_INT 3
#define ELEV_INT 19

#define POS0_VAL (PIND & (1<<PD3))
#define POS1_VAL (PINE & (1<<PE4))
#define POS2_VAL (PINE & (1<<PE5))
#define POS3_VAL (PIND & (1<<PD2))
#define NUM_AVGS 5

// First NUM_CAM_SERVOS are camera servos, and the following
// PWMServo servos [NUM_SERVOS];
Adafruit_TiCoServo servos [NUM_SERVOS];

enum class ElevState {stopped, slow, fast, fast_auto, slow_auto};

bool did_move = true;
bool stopped = true; // Whether all motors are stopped
uint8_t num_avgs = 0;

int8_t speeds [NUM_SERVOS] = {};
int16_t targets [NUM_SERVOS] = {};
uint16_t tmp_positions [NUM_SERVOS] = {};
uint16_t positions [NUM_SERVOS] = {};
volatile long time_deltas [NUM_SERVOS] = {};
volatile long counters [NUM_SERVOS] = {};
volatile bool elevTriggered = false;

unsigned long msgs_ts[NUM_MSGS] = {}; // Message send timestamps
unsigned long elev_ts = 0;
String input_string = "";         // A String to hold incoming data
bool string_complete = false;  // Whether the string is complete

ElevState curElevState = ElevState::stopped;

void serialEvent() {
  while (Serial.available()) {
    char in_char = (char)Serial.read();
    input_string += in_char;
    if (in_char == '\n') {
      string_complete = true;
    }
  }
}

void IntSrv0() {
  if (POS0_VAL) {
    counters[0] = micros();
  } else {
    time_deltas[0] = micros() - counters[0];
  }
}

void IntSrv1() {
  if (POS1_VAL) {
    counters[1] = micros();
  } else {
    time_deltas[1] = micros() - counters[1];
  }
}

void IntSrv2() {
  if (POS2_VAL) {
    counters[2] = micros();
  } else {
    time_deltas[2] = micros() - counters[2];
  }
}

void IntElev() {
  elevTriggered = true;
}

// Return an always-positive modulo
uint16_t mod(int16_t x, int16_t m) {
  return (x % m + m) % m;
}


int16_t AngularDistance(uint16_t a, uint16_t b) {
  int16_t dist = a - b;
  return mod(dist + 180, 360) - 180;
}


// speed is 0-SPD_MAX, direction is -1,0,1 (0 will stop)
void SetSpeed(uint8_t s_id, uint8_t speed, int8_t direction) {
  speed = min(SPD_MAX, speed);
  // Serial.print("Speed set: ");
  // Serial.print(speed * direction + SRV_CNTR);
  // Serial.print(";");
  // Serial.print(direction);
  // Serial.println(".");
  servos[s_id].write(direction * speed + SRV_CNTR);
  speeds[s_id] = direction * speed;
}


void SetAngle(uint8_t s_id, uint16_t angle) {
  servos[s_id].write(angle);
}


// Will update the speed to stop a servo if it has gone beyond distance
// as well as slow it down when it is SRV_SLOW_DIST from its target.
// Should work with both directions of rotation
void UpdateSpeed(uint8_t s_id, uint16_t pos, uint16_t target) {
  int16_t dist = AngularDistance(target, pos);
  uint8_t speed = 0;
  uint8_t dir = 0;
  // Serial.print("s_id, dist: ");
  // Serial.print(s_id);
  // Serial.print(" ");
  // Serial.print(target);
  // Serial.print(" ");
  // Serial.print(pos);
  // Serial.print(" ");
  // Serial.println(dist);
  // if (SRV_DIR == -1) {
  if (abs(dist) < SRV_STOP_DIST) {
    speed = 0;
    dir = 0;
  } else {
    if (abs(dist) < SRV_SLOW_DIST) {
      speed = SPD_SLOW;
    } else{
      speed = min(SPD_MAX, max(SPD_SLOW, abs(dist) * SPD_GAIN * SPD_MAX));
    }
    if (dist < 0) {
      dir = SRV_DIR;
    } else {
      dir = -1 * SRV_DIR;
    }
  }
  SetSpeed(s_id, speed, dir);

    // } else if (dist < SRV_SLOW_DIST && dist > 0) {
    //   SetSpeed(s_id, SPD_SLOW, SRV_DIR);
    // }
  // } else {
  //   if (dist >= 0 && speeds[s_id] == SPD_SLOW) {
  //     SetSpeed(s_id, 0, 0);
  //   } else if (dist > -1 * SRV_SLOW_DIST && dist < 0) {
  //     SetSpeed(s_id, SPD_SLOW, SRV_DIR);
  //   }
  // }
}


void SetSort(uint8_t label) {
  Serial.print("SetSort: ");
  Serial.println(label);
  for (int i=NUM_SORT_SERVOS; i < NUM_SERVOS; i++) {
    // 0 <= i < NUM_SORT_SERVOS
    if (i == label - NUM_SORT_SERVOS) {
      SetAngle(i, SORT_OPEN_ANGLE);
    } else {
      SetAngle(i, SORT_CLOSED_ANGLE);
    }
  }
}


void SetPos(uint16_t angle) {
  // Serial.print("SetPos: ");
  // Serial.println(angle);
  for (int i=0; i < NUM_POS_SERVOS; i++) {
    targets[i] = angle;
    // SetSpeed(i, SPD_FAST, SRV_DIR);
  }
  did_move = true;
}


void SetElevatorSpeed(uint8_t speed) {
  switch(speed) {
    case 0: {
      digitalWrite(ELEV_SLOW, 1);
      digitalWrite(ELEV_FAST, 1);
      curElevState = ElevState::stopped;
      break;
    }
    case 1: {
      digitalWrite(ELEV_SLOW, 0);
      digitalWrite(ELEV_FAST, 1);
      curElevState = ElevState::slow;
      break;
    }
    case 2: {
      digitalWrite(ELEV_SLOW, 1);
      digitalWrite(ELEV_FAST, 0);
      curElevState = ElevState::fast;
      break;
    }
  }
}


void setup() {
  Serial.begin(38400);

  // This is for serial control commands
  input_string.reserve(4);

  // Attach the servos:
  servos[0].attach(SRV0_PWM, SRV_POS_MIN, SRV_POS_MAX);
  servos[1].attach(SRV1_PWM, SRV_POS_MIN, SRV_POS_MAX);
  servos[2].attach(SRV2_PWM, SRV_POS_MIN, SRV_POS_MAX);
  servos[3].attach(SRV3_PWM, SRV_ANG_MIN, SRV_ANG_MAX);

  // Interrupt pins for servo positioning
  pinMode(POS0_INT, INPUT);
  pinMode(POS1_INT, INPUT);
  pinMode(POS2_INT, INPUT);

  // Interrupt for elevator arrival
  pinMode(ELEV_INT, INPUT);

  // Outputs for elevator control
  digitalWrite(ELEV_SLOW, 1);
  digitalWrite(ELEV_FAST, 1);
  pinMode(ELEV_SLOW, OUTPUT);
  pinMode(ELEV_FAST, OUTPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(POS0_INT), IntSrv0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POS1_INT), IntSrv1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POS2_INT), IntSrv2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELEV_INT), IntElev, RISING);

  // Initialize servo positions
  SetSpeed(0, 0, 0);
  SetSpeed(1, 0, 0);
  SetSpeed(2, 0, 0);
  SetAngle(3, SORT_OPEN_ANGLE);
}


void loop() {
  int i = 0;
  uint16_t angle = 0;
  uint8_t label = 0;
  uint8_t speed = 0;

  // Update the local variables from the volatile memory
  noInterrupts();
  for (; i < NUM_POS_SERVOS; i++) {
    tmp_positions[i] += max(0, time_deltas[i] - POSE_OFFSET);
  }
  interrupts();

  // Average the values to avoid spikes, reading therefore have a slight delay
  num_avgs++;
  if (num_avgs == NUM_AVGS) {
    for (int i=0; i<NUM_POS_SERVOS; i++) {
      positions[i] = (tmp_positions[i] / num_avgs) * POS_TO_ANGLE;
      tmp_positions[i] = 0;
    }
    num_avgs = 0;
  }


  // Manage serial command interface
  if (string_complete) {
    switch(input_string[0]) {
      case 'G':
        angle = input_string.substring(1).toInt();
        angle = max(0, min(360, angle));
        SetPos(angle);
        Serial.print("G ");
        Serial.println(angle);
        break;
      case 'S':
        label = input_string.substring(1).toInt();
        label = max(0, min(360, label));
        SetSort(label);
        Serial.print("S ");
        Serial.println(label);
        break;
      case 'E':
        speed = input_string.substring(1).toInt();
        speed = max(0, min(2, speed));
        SetElevatorSpeed(speed);
        Serial.print("E ");
        Serial.println(speed);
        break;
      // Automatic mode for elevator
      case 'A':
        SetElevatorSpeed(2);
        curElevState = ElevState::fast_auto;
        Serial.println("A");
        break;

    }
    input_string = String("");
    string_complete = false;
  }

  // Update speeds for P-controlled servos
  for(int i=0; i<NUM_POS_SERVOS; i++) {
    UpdateSpeed(i, positions[i], targets[i]);
  }

  // Run state machine
  switch (curElevState) {
    case ElevState::fast_auto :{
      if (elevTriggered) {
        SetElevatorSpeed(1);
        curElevState = ElevState::slow_auto;
        elevTriggered = false;
        elev_ts = millis();
      }
      break;
    }

    case ElevState::slow_auto :{
      if (millis() - elev_ts > ELEV_DECEL) {
        SetElevatorSpeed(0);
      }
      break;
    }

    default: break;
  }

  // Occasional status messages
  if (millis() - msgs_ts[MSG_STATUS] > MSG_STAT_FREQ) {
    Serial.print("STAT|");
    for(int i=0; i<NUM_POS_SERVOS; i++) {
      Serial.print(positions[i]);
      Serial.print("_");
      Serial.print(targets[i]);
      Serial.print(";");
    }

    Serial.print("E_");
    switch(curElevState) {
      case ElevState::fast: Serial.print("F"); break;
      case ElevState::slow: Serial.print("L"); break;
      case ElevState::stopped: Serial.print("S"); break;
      case ElevState::slow_auto: Serial.print("LA"); break;
      case ElevState::fast_auto: Serial.print("FA"); break;
    }
    Serial.println("");
    msgs_ts[MSG_STATUS] = millis();
  }

  // Announce servo arrival
  stopped = true;
  for (int i=0; i < NUM_POS_SERVOS; i++) {
    stopped &= (speeds[i] == 0);
  }
  if (stopped && did_move) {
    Serial.print("SA:");
    for(int i=0; i<NUM_POS_SERVOS; i++) {
      Serial.print(positions[i]);
      Serial.print(";");
    }
    Serial.println("");
    msgs_ts[MSG_STOP] = millis();
    did_move = false;
  }


}
