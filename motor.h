#ifndef _MOTOR_H
#define _MOTOR_H
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15 //Activation Pins

class motor_c {
  public:
  
    motor_c(); 
    void leftWheel(float power); 
    void rightWheel(float power);
};

motor_c::motor_c(){
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT ); 
}

void motor_c:: leftWheel(float power){
    if (power >= 0){
    digitalWrite( L_DIR_PIN, LOW );
    }
    else {
    digitalWrite( L_DIR_PIN, HIGH );
    }
    power = abs(power);
    analogWrite( L_PWM_PIN, power );
  }

void motor_c::rightWheel(float power) {
    if (power >= 0){
    digitalWrite( R_DIR_PIN, LOW );
    }
    else{
    digitalWrite( R_DIR_PIN, HIGH );
    }
    power = abs(power);
    analogWrite( R_PWM_PIN, power );
  }




#endif
