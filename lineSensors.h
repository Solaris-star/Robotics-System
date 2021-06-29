#ifndef _LineSensors_h
#define _LineSensors_h
#define LINE_LEFT_PIN A4
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A2
#define BUZZER 6

const int numCalibration = 100;

class lineSensor_c {
  public:

    lineSensor_c(int pin);  

    void calibrate();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalibrated();  //Return the calibrated value from the sensor

  private:
    int pin;
    int avg;
};

lineSensor_c::lineSensor_c(int Pin) {
  pin = Pin;
  pinMode(pin, INPUT);
}

// Returns uncalibrated reading.
  int lineSensor_c::readRaw() {
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void lineSensor_c::calibrate() {
    long temp = 0;
    for (float  i  = 0; i < numCalibration; i = i+ 1) {
    temp = temp + analogRead(pin);
  }

  avg = temp / numCalibration;
  analogWrite(BUZZER, 10);
  delay(75);
  analogWrite(BUZZER, 0);
  
}

// calibrated sensor reading.
   int lineSensor_c::readCalibrated() {  
   int calibration = analogRead(pin) - avg;
   calibration = constrain(calibration, 0, 1023);
   return calibration;
}


#endif
