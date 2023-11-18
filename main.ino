#include "planner.h"
#include "stepper.h"
#include "inverseKinematics.h"
#include "Marlin.h"

#include <PID_v1.h>
#include <thermistor.h>

thermistor therm1(A13,0);
#define PIN_INPUT 0
#define PIN_OUTPUT 8

#define DELTA_SEGMENTS_PER_SECONDS 100

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp1=22.2, Ki1=1.08, Kd1=114;
PID myPID(&Input, &Output, &Setpoint, Kp1, Ki1, Kd1, DIRECT);

double cur_position[4] = {13.216, 2.8078, -451.667, 0}; // XYZE
double delta[4] = {0.0};
double alpha = -0.0808; // Orientation
uint8_t active_extruder = 0;

inline bool prepare_kinematic_move_to(double ltarget[4], double ori) {

  // Get the top feedrate of the move in the XY plane
  const float _feedrate_mm_s = 100;

  // Get the cartesian distances moved in XYZE
  const double difference[4] = {
    ltarget[0] - cur_position[0],
    ltarget[1] - cur_position[1],
    ltarget[2] - cur_position[2],
    ltarget[3] - cur_position[3]
  };

  // Get the linear distance in XYZ
  float cartesian_mm = sqrt(sq(difference[0]) + sq(difference[1]) + sq(difference[2]));

  // If the move is very short, check the E move distance
  if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = fabs(difference[E_AXIS]);

  // No E move either? Game over.
  if (UNEAR_ZERO(cartesian_mm)) return true;

  // Minimum number of seconds to move the given distance
  const float seconds = cartesian_mm / _feedrate_mm_s;

  // The number of segments-per-second times the duration
  // gives the number of segments
  uint16_t segments = DELTA_SEGMENTS_PER_SECONDS * seconds;

  // At least one segment is required
  NOLESS(segments, 1);

  // The approximate length of each segment
  const float inv_segments = 1.0 / float(segments),
              segment_distance[4] = {
                difference[0] * inv_segments,
                difference[1] * inv_segments,
                difference[2] * inv_segments,
                difference[3] * inv_segments
              };
  
  // Get the logical current position as starting point
  double logical[4];
  memcpy(logical, cur_position, sizeof(current_position));
  
  // Drop one segment so the last move is to the exact target.
  // If there's only 1 segment, loops will be skipped entirely.
  --segments;

  // Calculate and execute the segments
  for (uint16_t s = segments + 1; --s;) {
    for (uint8_t i = 0; i < 4; i++) {
      logical[i] += segment_distance[i];
    }
    inverseKinematics inv(logical[0] , logical[1] , logical[2] , ori);
      delta[0] = inv.d[0] - 129.7;
      delta[1] = inv.d[1] - 141.25;
      delta[2] = inv.d[2] - 131.75;
      delta[3] = inv.d[3] - 159.8;

  /*    delta[0] = inv.d[0] - 129.7;
      delta[1] = inv.d[1] - 141.25;
      delta[2] = inv.d[2] - 131.75;
      delta[3] = inv.d[3] - 159.8;  */


    plan_buffer_line(delta[3], delta[0], delta[1], delta[2], 0, logical[3], _feedrate_mm_s, active_extruder);
  }

  // Since segment_distance is only approximate,
  // the final move must be to the exact destination.
  inverseKinematics inv(ltarget[0], ltarget[1], ltarget[2], ori);
    delta[0] = inv.d[0] - 129.7;
    delta[1] = inv.d[1] - 141.25;
    delta[2] = inv.d[2] - 131.75;
    delta[3] = inv.d[3] - 159.8;

  plan_buffer_line(delta[3], delta[0], delta[1], delta[2], 0, ltarget[3],  _feedrate_mm_s, active_extruder);
  memcpy(cur_position, ltarget, sizeof(ltarget));
  
  return false;
}

char str[10];
double ret;
void read_double() {
  char *ptr;
  ret = strtod(str, &ptr);
}

double x;
double y;
double z;
double e;
double d[5];
double target[4];
uint8_t i = 0;
char c;
uint8_t G = 0;
bool f = false;
bool t = false;


void step_A() {
  digitalWrite(54, HIGH);
  digitalWrite(54, LOW);
}

void step_B() {
  digitalWrite(60, HIGH);
  digitalWrite(60, LOW);
}

void step_C() {
  digitalWrite(41, HIGH);
  digitalWrite(41, LOW);
}

void step_D() {
  digitalWrite(36, HIGH);
  digitalWrite(36, LOW);
}

void homing_cycle() {
  digitalWrite(55, LOW);
  digitalWrite(61, LOW);
  digitalWrite(43, HIGH);
  digitalWrite(34, HIGH);
  
  while (digitalRead(32) == LOW || digitalRead(B_MIN) == LOW || digitalRead(C_MIN) == LOW || digitalRead(D_MIN) == LOW) {
    if (digitalRead(A_MIN) == LOW) step_A();
    if (digitalRead(B_MIN) == LOW) step_B();
    if (digitalRead(C_MIN) == LOW) step_C();
    if (digitalRead(D_MIN) == LOW) step_D();
    delayMicroseconds(2000);
  } 
}

void temp_control() {
  Input = therm1.analog2temp();
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}

void setup() {
  pinMode(A_MIN, INPUT_PULLUP);
  pinMode(B_MIN, INPUT_PULLUP);
  pinMode(C_MIN, INPUT_PULLUP);
  pinMode(D_MIN, INPUT_PULLUP);
  pinMode(10, OUTPUT);
  plan_init();
  st_init();
  Serial.begin(115200);
  Input = therm1.analog2temp();
  Setpoint = 265;
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

bool homed = false;

void loop() {

 //plan_buffer_line(-50, -50, -50, -50, 0, 0, 20, 1);
  if (homed == false) {
    homed = true;
    homing_cycle();
  }
   
  analogWrite(10, 100);
  Input = therm1.analog2temp();
  
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  if (Input+10 >= Setpoint && t == false ) {
    t = true;
    Serial.print('r');
  }
  
  while ((c=Serial.read()) != -1 ) {

    if (f == false) {
      f = true;
      plan_buffer_line(81.377, 97.085, 100.043, 84.13, 0, 0, 30, 1);
    }

    Input = therm1.analog2temp();
    myPID.Compute();
    analogWrite(PIN_OUTPUT, Output);
    analogWrite(10, 100);

    if (c == 'A') {
      read_double();
      d[0] = ret;
      memset(str, 0, sizeof(str));
      i = 0;
    } else if (c == 'B') {
      read_double();
      d[1] = ret;
      memset(str, 0, sizeof(str));
      i = 0;
    } else if (c == 'C') {
      read_double();
      d[2] = ret;
      memset(str, 0, sizeof(str));
      i = 0;
    } else if (c == 'D') {
       read_double();
       d[3] = ret;
       memset(str, 0, sizeof(str));
       i = 0;
    } else if (c == 'E') {
      read_double();
      d[4] = ret;
      memset(str, 0, sizeof(str));
      i = 0;
    } else if (c == 10) { // Line is complete
      plan_buffer_line(d[3], d[0], d[1], d[2], 0, d[4] / 11.56, 12, 1);
      //plan_buffer_line(0, 0, 0, 0, 0, d[4], 1, 1);
      Serial.print('r');
    } else { // Continue recieving 
       str[i] = c;
       i++;
    }
  }
}
