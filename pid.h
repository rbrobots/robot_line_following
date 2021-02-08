#ifndef _PID_h
#define _PID_h
#include <stdint.h>

class PID
{

  public:

    PID(float P, float D, float I); // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float D, float I); // This function updates the values of the gains
    void reset(); //This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement); //This function calculates the PID control signal. It should be called in a loop
    void print_components(); //This function prints the individual components of the control signal and can be used for debugging
    void setMax(float  newMax); //This function sets the maximum output the controller can ask for
    void setDebug(bool state); //This function sets the debug flag;
    void print_response(); // This function prints the ratio of input to output in a way that is nicely interpreted by the Serial plotter
    void set_show_response(bool state); //This functions set the show_response flag
    bool task_completed(float error); //checks task completed against threshold of error and returns true/false
    float checkDistance();

  private:

    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative
    float max_output=255; 

    float Kp_output=0; 
    float Ki_output=0;
    float Kd_output=0;
    float total=0;

    //Values to store
    float last_error=0; //For calculating the derivative term
    float integral_error=0; //For storing the integral of the error
    long last_millis = 0;
    bool debug=false; //This flag controls whether we print the contributions of each component when update is called

    float last_demand=0; //For storing the previous input
    float last_measurement=0; //For storing the last measurement
    bool show_response = false; // This flag controls whether we print the response of the controller on each update
};


 PID::PID(float P, float D, float I)
{
  //Store the gains
  setGains(P, D, I);
  //Set last_millis
  last_millis = 0;
}

void PID::print_components()
{
  Serial.print("Proportional component: ");
  Serial.print(Kp_output);
  Serial.print(" Differential component: ");
  Serial.print(Kd_output);
  Serial.print(" Integral component: ");
  Serial.print(Ki_output);
  Serial.print(" Total: ");
  Serial.println(total);
}

/*
 * This function sets the gains of the PID controller
 */
void PID::setGains(float P, float D, float I)
{
  Kp = P;
  Kd = D;
  Ki = I;
}


float PID::update(float demand, float measurement)
{
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;

  last_demand = demand;
  last_measurement = measurement;
  //This represents the error term
  //float error = 0;
  float error=demand-measurement;
  //This represents the error derivative
  
  float error_delta = (error-last_error)/time_delta;//change in error divided by change in time

  last_error=error;//set the current error as last for next iteration
  //Update storage
  integral_error = integral_error+(error*time_delta);//integral-error*dt; Is this the cumulative sum? CHECK THIS
  //integral_error =0;//got rid of integral for time being

  //Calculate components
  Kp_output = Kp*error;
  Kd_output = Kd*error_delta;
  Ki_output = Ki*integral_error;

  //Add the three components to get the total output
  total = Kp_output + Kd_output + Ki_output;

  //Make sure we don't exceed the maximum output
  if (total > max_output)
  {
    total = max_output;
  }

  if (total < -max_output)
  {
    total = -max_output;
    }

  //Print debugging information if required
  if (debug)
  {/*
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" Error Delta");
    Serial.println(error_delta);
    Serial.print(" kd_output");
    Serial.println(Kd_output);*/
    Serial.print(" measurement=");
    Serial.print(measurement);
    Serial.print(" demand= ");
    Serial.print(demand);
    Serial.print("Error: ");
    Serial.println(error);
    print_components();
  }
  
  return total;
}

float PID::checkDistance(){//this function will check that a large distance is decreased by a magnitude so that we are still able to get to the goal state
  //will return a reduced speed for output
  //split demand into smaller chunks?
}

bool PID::task_completed(float e){
  float t=0;
  if(e>t){//if error above threshold return false
    return false;
  }
  else{//if error below threshold return true
    return true;
  }
}

void PID::setMax(float newMax)
{
  if (newMax > 0)
  {
    max_output = newMax;
  }
  else
  {
    Serial.println("Max output must be positive");
  }
}

void PID::setDebug(bool state)
{
  debug = state;
}

void PID::reset()
{
  
  last_error = 0;
  integral_error = 0;
  last_millis = millis();
  
}

void PID::print_response()
{
  float response = last_measurement / last_demand;
  Serial.println(response);
}

void PID::set_show_response(bool state)
{
  show_response = state;
}


#endif
