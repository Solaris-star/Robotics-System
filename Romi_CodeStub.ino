#include "encoders.h"
#include "pid.h"
#include "kinematics.h"
#include "lineSensors.h"
#include "motor.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define LINE_LEFT_PIN   A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A4 //Pin for the right line sensor
#define kp_linefw 5
#define ki_linefw 0.05
#define kd_linefw 0.15
#define kp_return 3.50
#define ki_return 0.0003
#define kd_return 0.05
#define BUZZER 6

int i = 0; //For recording the direction of route. 
           //If i = 0 then it's a forward route, if i = 1. then it's a reverse route.
float start_time = 0; //It's used to keep track of how much time has passed since the Romi started.
unsigned long startTime;
unsigned long prev_update;
unsigned long prev_move;
unsigned long timeStamp;
long count_prev_left;
long count_prev_right;
float homeTheta;
int LeftSensorRead,  CentreSensorRead,  RightSensorRead; //define sensor readings
int state;
bool setup_dist;
bool setup_check;
motor_c M; //Control the condition of the left and right wheels.
PID_c pid_returnd( kp_return, ki_return, kd_return );
PID_c pid_linefw( kp_linefw, ki_linefw, kd_linefw );

lineSensor_c line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
lineSensor_c line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
lineSensor_c line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics_c kinematics;


void setup() 
{
  // Initialise your other globals variables
  // and devices.

  setupEncoder0();
  setupEncoder1();

  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  state = 0; // Remember the Romi's current state.
  setup_dist = false;
  setup_check = false;

  startTime = millis();
  prev_update = millis(); 
  prev_move = millis();
  timeStamp = micros();
  count_prev_left = 0;
  count_prev_right = 0;

  LeftSensorRead = 0;
  CentreSensorRead = 0;
  RightSensorRead = 0;
  
  // Initialise the Serial communication
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
}
// over set


void loop(){
  unsigned long current_time = millis();
  unsigned long update_time = current_time - prev_update;
  unsigned long move_time = current_time - prev_move;

  if (update_time > 2) {
    prev_update = current_time;
    kinematics.update(count_left, count_right); // call an update to your kinematics at a time interval
  }
  if (move_time > 5) {
    prev_move = current_time;
    Serial.println(state);
      if (state == 0){
        find_line();
      }
      else if (state == 1){
        BangBang();
      }
      else if (state == 2){
        line_follow();
      }
      else if (state ==3){
        find_home();
      }
      else if (state ==4){
        return_home();
      }
      else if (state == 5){
        M.leftWheel(0.0);
        M.rightWheel(0.0);
      }
  }
}
// over loop


//Go straight until find the line.
  void find_line() {

  bool onLine = CheckForLine();

  if (!onLine) {
    float theta_error = kinematics.getTheta();
    int turn_pwm = 0;
  
    if (theta_error < 0){
      turn_pwm = -2;
    }
    else if (theta_error > 0) {
      turn_pwm = 2;
    }
    else turn_pwm = 0;
  
    int left_demand = 50 - turn_pwm;
    int right_demand = 50 + turn_pwm;
  
    M.leftWheel(left_demand);
    M.rightWheel(right_demand);
  }
  else {
    M.leftWheel(0.0);
    M.rightWheel(0.0);

    //Set state to follow line.
    state = 1;
  }
}

//Determine if Romi is online.
bool CheckForLine() {

  bool onLine = false;

  LeftSensorRead = line_left.readCalibrated();
  CentreSensorRead = line_centre.readCalibrated();
  RightSensorRead = line_right.readCalibrated();

  if ( LeftSensorRead > 300 || CentreSensorRead > 300 || RightSensorRead > 300 ) {
    onLine = true;
  }
  return onLine;   
}

void BangBang() {
  
  LeftSensorRead = line_left.readCalibrated();
  CentreSensorRead = line_centre.readCalibrated();
  RightSensorRead = line_right.readCalibrated();

  bool left_on_line = false;
  bool centre_on_line = false;
  bool right_on_line = false;
  
  if (LeftSensorRead > 90) left_on_line = true;
  if (CentreSensorRead > 110) centre_on_line = true;
  if (RightSensorRead > 90) right_on_line = true;

  if (centre_on_line) {
    M.leftWheel(37.0);
    M.rightWheel(36.0);
  }
  else if (left_on_line) {
    M.rightWheel(36.0);
    M.leftWheel(-37.0);
  }
  else if (right_on_line) {
    M.leftWheel(37.0);
    M.rightWheel(-36.0);
  }
  else {
    M.leftWheel(0.0);
    M.rightWheel(0.0);

    //Try to rejoin line
    state = 2;
  }
}

//Try to find line if lost
  bool line_follow() {

  bool FoundLine = false;

  unsigned long current_time = millis();
  
  if (!setup_check) {
    startTime = millis();
    setup_check = true;
  }

  unsigned long elapsedTime = current_time - startTime;

  if (elapsedTime < 1150) {
    M.leftWheel(33.0);
    M.rightWheel(-32.0);
    FoundLine = CheckForLine();
  }
  else if (elapsedTime < 3680) {
    M.leftWheel(-33.0);
    M.rightWheel(32.0);
    FoundLine = CheckForLine();
  }
  else if (elapsedTime < 5000){
    M.leftWheel(33.0);
    M.rightWheel(-32.0);
    FoundLine = CheckForLine();  
  }  
  else if (elapsedTime < 5550){
    M.leftWheel(33.0);
    M.rightWheel(32.0);
    FoundLine = CheckForLine();  
  }
  else if((current_time - start_time) < 16000){
    i ++;
    if(elapsedTime < 8150){
    M.leftWheel(33.0);
    M.rightWheel(-32.0);
    FoundLine = CheckForLine();
  }else if (elapsedTime < 8550){
    M.leftWheel(33.0);
    M.rightWheel(32.0);
    FoundLine = CheckForLine();
    }}
  else {
    M.leftWheel(0.0);
    M.rightWheel(0.0);
    analogWrite(BUZZER, 10);
    delay(500);
    analogWrite(BUZZER, 0);
    state = 3;
  }
  if (FoundLine) {
    setup_check = false;
    state = 1;
  }
}

//Make sure the Romi is pointing to the starting position
void find_home() {
  if(i == 0){
  float angle = kinematics.homeAngle();
  turn_angle_right(angle);
  } else if (i > 0){
  float angle = kinematics.homeAnglez();
  turn_angle_left(angle);
    }
}

void return_home() {
  
  if (!setup_dist) {
    homeTheta = kinematics.getTheta();
    setup_dist = true;
  }

  float theta_error = homeTheta - kinematics.getTheta();
  int turn_pwm = 0;

  if (theta_error > 0){
    turn_pwm = -2;
  }
  else if (theta_error < 0) {
    turn_pwm = 2;
  }
  else turn_pwm = 0;

  int left_demand = 50 - turn_pwm;
  int right_demand = 50 + turn_pwm;

  M.leftWheel(left_demand);
  M.rightWheel(right_demand);

  if (abs(kinematics.getx_pos()) < 10) {
    setup_dist = false;
    state = 5;
  }
  
}

//
void turn_angle_right(float angle) {

  float new_count_left, new_count_right;

  if (!setup_dist) {
    float count = (((angle / 360.0) * (140.0 * PI)) / (70.0 * PI)) * 1440.0;
    new_count_left = count_left + count;
    new_count_right = count_right - count;
    setup_dist = true;
  }

  float power = 75.0;
  
  if (count_left < new_count_left) {
    M.leftWheel(power + 1.0);
  }
  else M.leftWheel(0.0);

  if (count_right > new_count_right) {
    M.rightWheel(-power);
  }
  else M.rightWheel(0.0);

  if (count_left > new_count_left && count_right < new_count_right) {
    setup_dist = false;
    
    //Set state to drive towards home.
    state = 4;
  }
}

void turn_angle_left(float angle) {

  float new_count_left, new_count_right;

  if (!setup_dist) {
    float count = (((angle / 360.0) * (140.0 * PI)) / (70.0 * PI)) * 1440.0;
    new_count_left = count_left - count;
    new_count_right = count_right + count;
    setup_dist = true;
  }

  float power = 75.0;
  
  if (count_left < new_count_left) {
    M.leftWheel(power + 1.0);
  }
  else M.leftWheel(0.0);

  if (count_right > new_count_right) {
    M.rightWheel(-power);
  }
  else M.leftWheel(0.0);

  if (count_left > new_count_left && count_right < new_count_right) {
    setup_dist = false;

    state = 4;
  }
}
