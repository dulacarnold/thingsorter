#include <Adafruit_TiCoServo.h>
#include <Arduino.h>


#define NUM_POS_SERVOS 3
#define NUM_SORT_SERVOS 1
#define NUM_SERVOS NUM_POS_SERVOS + NUM_SORT_SERVOS

#define SORT_OPEN_ANGLE 0
#define SORT_CLOSED_ANGLE 65

#define SRV_POS_MIN 1280 // 1200
#define SRV_POS_MAX 1720 // 1800
#define SRV_ANG_MIN 1000 // For the orientation servo
#define SRV_ANG_MAX 2000
#define SRV_DIR 1 // Which direction is positive angle movement
#define SRV_CNTR 90
#define SRV_STOP_DIST 3
#define SRV_SLOW_DIST 8
#define SPD_SLOW 30
#define SPD_FAST 80
#define SPD_MAX 90
#define SPD_GAIN 0.03F

#define ELEV_DECEL 200 // Time elevator spends on slow before stop
#define ELEV_TRIG_OFFSET 150
 // Offset between actual trigger interrupt and action on signal
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
#define ELEV_SLOW 48
#define ELEV_FAST 47

// Absolute position sensors on servos, camera servos first and then
// sort servos.
#define POS0_INT 18
#define POS1_INT 2
#define POS2_INT 3
#define ELEV_INT 19
#define ELEV_DEBOUNCE 100

#define POS0_VAL (PIND & (1<<PD3))
#define POS1_VAL (PINE & (1<<PE4))
#define POS2_VAL (PINE & (1<<PE5))
#define ELEV_VAL (PIND & (1<<PD2))
#define NUM_AVGS 5

// First NUM_CAM_SERVOS are camera servos, and the following
// PWMServo servos [NUM_SERVOS];
Adafruit_TiCoServo servos [NUM_SERVOS];

enum class ElevState {stopped, slow, fast, fast_auto, slow_auto};

bool sorter_ready = false; // Whethere the sorting mechanism is ready
bool did_move = true; // Whether a move was requested
bool stopped = true; // Whether all motors are stopped
uint8_t num_avgs = 0;

// Motor control variables
int8_t speeds [NUM_SERVOS] = {}; // Current speeds of rotor motors
int16_t targets [NUM_SERVOS] = {};
volatile long time_deltas [NUM_SERVOS] = {}; // Volatile storage of time delta
volatile long counters [NUM_SERVOS] = {}; // Starting timestamps for PWM counters
uint16_t tmp_positions [NUM_SERVOS] = {}; // Local variable for pre-average positions
uint16_t positions [NUM_SERVOS] = {}; // Averaged positions of variables
// Elevator trigger variables
volatile bool elev_down_vol = false;
volatile bool elev_triggered_vol = false;
volatile long elev_triggered_ts_vol = 0; // Timestamp for stop delay + debounce
volatile long elev_debounce_ts_vol = 0;
bool elev_triggered = false;
// long elev_triggered_ts = 0;
// Input string management
volatile bool string_complete = false;  // Whether the string is complete
String input_string_buffer = String(""); // A String to hold incoming data

String input_string = String("");
unsigned long msgs_ts[NUM_MSGS] = {}; // Message send timestamps
unsigned long elev_ts = 0;

ElevState curElevState = ElevState::stopped;

void serialEvent() {
  while (Serial.available()) {
    if (string_complete) {
      break;
    }
    char in_char = (char)Serial.read();
    input_string_buffer += in_char;
    if(in_char == '\n') {
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
    if(ELEV_VAL == 0) {
      elev_debounce_ts_vol = micros();
      elev_down_vol = true;
    } else {
      if(elev_down_vol && micros() - elev_debounce_ts_vol > ELEV_DEBOUNCE) {
        elev_triggered_vol = true;
        elev_down_vol = false;
        elev_triggered_ts_vol = millis();
      }
    }
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
}


void SetSort(uint8_t label) {
  for (int i=0; i < NUM_SORT_SERVOS; i++) {
    if (i == label) {
      SetAngle(i + NUM_POS_SERVOS, SORT_OPEN_ANGLE);
    } else {
      SetAngle(i + NUM_POS_SERVOS, SORT_CLOSED_ANGLE);
    }
  }
}


void SetPosAll(uint16_t angle) {
  for (int i=0; i < NUM_POS_SERVOS; i++) {
    targets[i] = angle;
  }
  did_move = true;
}


void SetPos(uint16_t angle, uint8_t motor_id) {
  targets[motor_id] = angle;
  did_move = true;
}


void SetElevatorSpeed(uint8_t speed) {
  switch(speed) {
    case 0: {
      digitalWrite(ELEV_SLOW, 0);
      digitalWrite(ELEV_FAST, 0);
      // curElevState = ElevState::stopped;
      break;
    }
    case 1: {
      digitalWrite(ELEV_SLOW, 1);
      digitalWrite(ELEV_FAST, 0);
      // curElevState = ElevState::slow;
      break;
    }
    case 2: {
      digitalWrite(ELEV_SLOW, 0);
      digitalWrite(ELEV_FAST, 1);
      // curElevState = ElevState::fast;
      break;
    }
  }
}


void setup() {
  // Attach the servos:
  servos[0].attach(SRV0_PWM, SRV_POS_MIN, SRV_POS_MAX);
  servos[1].attach(SRV1_PWM, SRV_POS_MIN, SRV_POS_MAX);
  servos[2].attach(SRV2_PWM, SRV_POS_MIN, SRV_POS_MAX);
  servos[3].attach(SRV3_PWM, SRV_ANG_MIN, SRV_ANG_MAX);

  Serial.begin(38400);

  // This is for serial control commands
  input_string.reserve(50);

  // Interrupt pins for servo positioning
  pinMode(POS0_INT, INPUT);
  pinMode(POS1_INT, INPUT);
  pinMode(POS2_INT, INPUT);

  // Interrupt for elevator arrival
  pinMode(ELEV_INT, INPUT_PULLUP);

  // Outputs for elevator control
  digitalWrite(ELEV_SLOW, 0);
  digitalWrite(ELEV_FAST, 0);
  pinMode(ELEV_SLOW, OUTPUT);
  pinMode(ELEV_FAST, OUTPUT);

  delay(2000);
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(POS0_INT), IntSrv0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POS1_INT), IntSrv1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POS2_INT), IntSrv2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELEV_INT), IntElev, CHANGE);

  // Initialize servo positions
  SetSpeed(0, 0, 0);
  SetSpeed(1, 0, 0);
  SetSpeed(2, 0, 0);
  SetAngle(3, SORT_OPEN_ANGLE);
}


void loop() {
  // int i = 0;
  uint16_t angle = 0;
  uint8_t motor_id = 0;
  uint8_t label = 0;
  uint8_t speed = 0;

  // Update the local variables from the volatile memory
  noInterrupts();
  for (int i = 0; i < NUM_POS_SERVOS; i++) {
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
    noInterrupts();
    input_string = String(input_string_buffer.c_str());
    input_string_buffer = String("");
    string_complete = false;
    // Serial.print("RECV: ");
    // Serial.println(input_string);
    interrupts();
    switch(input_string[0]) {

      case 'G':
        if (input_string[1] == ' ') {
          angle = input_string.substring(1).toInt();
          angle = max(0, min(360, angle));
          SetPosAll(angle);
          Serial.print("G ");
          Serial.println(angle);
        } else {
          angle = input_string.substring(2).toInt();
          motor_id = input_string.substring(1,2).toInt();
          angle = max(0, min(360, angle));
          SetPos(angle, motor_id);
          Serial.println(String("G") + motor_id + String(" ") + angle);
        }
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
        switch (speed) {
          case 2: curElevState = ElevState::fast; break;
          case 1: curElevState = ElevState::slow; break;
          case 0: curElevState = ElevState::stopped; break;
        }
        // SetElevatorSpeed(speed);
        Serial.print("E ");
        Serial.println(speed);
        break;

      // Automatic mode for elevator
      case 'A': {
        if (curElevState == ElevState::fast_auto) {
          sorter_ready = true;
        }
        curElevState = ElevState::fast_auto;
        Serial.println("A");
        break;
      }

      case 'R':
        sorter_ready = true;
        Serial.println("R");
        break;
    }
  }

  // Update speeds for P-controlled servos
  for(int i=0; i<NUM_POS_SERVOS; i++) {
    UpdateSpeed(i, positions[i], targets[i]);
  }

  if (elev_triggered_vol && (millis() - elev_triggered_ts_vol) > ELEV_TRIG_OFFSET) {
    elev_triggered = true;
    elev_triggered_vol = false;
  }

  // Run state machine
  switch (curElevState) {
    case ElevState::fast_auto: {
      SetElevatorSpeed(2);
      if (elev_triggered) {
        // If the sorter is ready anyways, carry on and wipe the flag.
        if(sorter_ready) {
          sorter_ready = false;
        // Otherwise slow down the line and prepare to stop.
        } else {
          curElevState = ElevState::slow_auto;
          elev_ts = millis();
        }
      }
      break;
    }

    case ElevState::slow_auto: {
      SetElevatorSpeed(1);
      if (millis() - elev_ts > ELEV_DECEL) {
        curElevState = ElevState::stopped;
      }
      break;
    }

    case ElevState::fast: SetElevatorSpeed(2); break;
    case ElevState::slow: SetElevatorSpeed(1); break;
    case ElevState::stopped: SetElevatorSpeed(0); break;

    default:
      SetElevatorSpeed(0);
      curElevState = ElevState::stopped;
      break;
  }
  if (elev_triggered) {
    Serial.println("ET");
    elev_triggered = false;
  }

  // Occasional status messages
  if (millis() - msgs_ts[MSG_STATUS] > MSG_STAT_FREQ) {
    Serial.print("STAT:");
    for(int i=0; i<NUM_POS_SERVOS; i++) {
      Serial.print(positions[i]);
      Serial.print("_");
      Serial.print(targets[i]);
      Serial.print(";");
    }
    Serial.println("");

    Serial.print("ELEV:");
    switch(curElevState) {
      case ElevState::fast: Serial.print("F"); break;
      case ElevState::slow: Serial.print("L"); break;
      case ElevState::stopped: Serial.print("S"); break;
      case ElevState::slow_auto: Serial.print("LA"); break;
      case ElevState::fast_auto: Serial.print("FA"); break;
      default: Serial.print("FE"); break;
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
