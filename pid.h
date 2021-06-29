#ifndef _PID_h
#define _PID_h

class PID_c {
  
  public:

    PID_c(float P, float I, float D);               
    void setGains(float P, float I, float D );      
    void reset();                                   
    float update(float demand, float measurement); 
    void printComponents();                        
    void setMax(float newMax);                    
    void setDebug(bool state);                     
    void printResponse();                         
    void setShowResponse(bool state);             

  private:

    //Control gains
    float Kp; //Proportional gain
    float Ki; //Integral gain
    float Kd; //Derivative gain

    //We can use this to limit the output to a certain value
    float maxOutput = 255; 

    //Output components
    //These are used for debugging purposes
    float Kp_output = 0; 
    float Ki_output = 0;
    float Kd_output = 0;
    float totalOutput = 0;

    //Values to store between updates().
    float lastDemand = 0;            
    float lastMeasurement = 0;      
    float lastError = 0;            
    float integralError = 0;        
    long  lastMillis = 0;           
    bool  debug = false;             
    bool  showResponse = false;     
    float timeDelta;
};

 PID_c::PID_c(float P, float I, float D)
{
  //Store PID_c gains
  setGains(P,I,D);
  long lastMillis = millis();
}

/*
 * This function sets the gains of the PID_c controller
 */
void PID_c::setGains(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}

float PID_c::update(float demand, float measurement) {
  //Calculate how much time (in milliseconds) has passed since the last update call
  long timeNow = millis();
  int timeDelta = timeNow - lastMillis;
  lastMillis = timeNow;
  
  //This represents the error term
  float error;
  error = demand - measurement;   
  
  //This represents the error derivative
 float errorDelta;
 errorDelta= (error - lastError)/float(timeDelta);
 lastError = error;

  // This represents the error integral.
  // Integrate error over time.
  integralError = integralError + (error * float(timeDelta));

  //Attenuate above error components by gain values.
  Kp_output = Kp * error;
  Ki_output = Ki * integralError;
  Kd_output = Kd * errorDelta;

  // Add the three components to get the total output

  float totalOutput = Kp_output + Ki_output + Kd_output;

  /*
   * ===========================
   * Code below this point should not need to be changed
   * But of course, feel free to improve / experiment :)
   */

   
  //Update persistent variables.
  lastDemand = demand;
  lastMeasurement = measurement;

  // Catching max in positive sign.
  if (totalOutput > maxOutput) {
    totalOutput = maxOutput;
  } 

  // Catching max in negative sign
  if (totalOutput < -maxOutput) {
    totalOutput = -maxOutput;
  }

  //Print debugging information if required
  if (debug) {
    Serial.print("error:");
    Serial.print(error);
    Serial.print("errorDelta:");
    Serial.print(errorDelta);
    Serial.print("integralError:");
    Serial.print(integralError);
    printComponents();
  }

  //Print response if required
  if (showResponse) {
    printResponse();
  }
  
  return totalOutput;
}

void PID_c::setMax(float newMax)
{
  if (newMax > 0) {
    maxOutput = newMax;
  } else {
    Serial.println("Maximum output must be strictly +ve");
  }
}

void PID_c::setDebug(bool state) {
  debug = state;
}

void PID_c::reset() {
  
  lastError = 0;
  integralError = 0;
  lastMillis = millis();
  
}

void PID_c::printResponse() {
  float response = lastMeasurement / lastDemand;
  Serial.println(response);
}

void PID_c::setShowResponse(bool state) {
  showResponse = state;
}
#endif
