#ifndef _Kinematics_h
#define _Kinematics_h
#define PI 3.142

// Constant variables

const float wheelRad = 40; // mm
const float wheelDia = 80.0; 
const float wheelScale = 120.0;
const float gearRatio = 1/120;
const float COUNTS_PER_SHAFT_REVOLUTION = 12.0;
const float COUNTS_PER_WHEEL_REVOLUTION =  1440.0;
const float MM_PER_COUNT  = (1 / (wheelDia * PI)) * COUNTS_PER_WHEEL_REVOLUTION;

class Kinematics_c
{
  public:
    
    Kinematics_c();   // Constructor, required.

    void update(long count_left, long count_right);  // should calucate an update to position.
    float homeAngle();
    float homeAnglez();
    void  resetTheta();
    float getTheta();
    float getx_pos();
    float gety_pos();
    
  private:
    
    //Private variables and methods go here
    float x_pos;
    float y_pos;
    float theta;

    long old_count_left;
    long old_count_right;
    
};


// Required constructor.  Initialise variables.
Kinematics_c::Kinematics_c() {
  x_pos =  0;
  y_pos =  0;
  theta = 0;

  old_count_left = 0;
  old_count_right = 0;

}

void Kinematics_c :: update(long count_left, long count_right) {

  long change_left = count_left - old_count_left;
  long change_right = count_right - old_count_right;

  float leftDist = (float)change_left / MM_PER_COUNT;
  float RightDist = (float)change_right / MM_PER_COUNT;

  float avgDist = (leftDist + RightDist) / 2.0f;
  
  x_pos = x_pos + avgDist * cos(theta);
  y_pos = y_pos + avgDist * sin(theta); 

  theta = theta + (leftDist - RightDist) / wheelScale;

  old_count_left = count_left;
  old_count_right = count_right;
  return;
}


float Kinematics_c :: homeAngle() {
  float angle = (- theta / (2 * PI) * 360) + 165;
  //float angle = atan2(y_pos, x_pos);
  return angle;
}

float Kinematics_c :: homeAnglez() {
  float angle = ( - theta / (2 * PI) * 360 + 355
  
  ) ;
  //float angle = atan2(y_pos, x_pos);
  return angle;
}

void Kinematics_c :: resetTheta() {
  theta = 0;
}


float Kinematics_c :: getTheta() {
  return theta;
}

float Kinematics_c :: getx_pos() {
  return x_pos;
}

float Kinematics_c :: gety_pos() {
  return y_pos;
}

#endif
