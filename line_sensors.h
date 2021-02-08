#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration


/* 
 *  Class to represent a single line sensor
 */
class Line_Sensor
{
  public:
    //Constructor
    Line_Sensor(int pin);
    //Calibrate
    void calibrate();
    //Return the uncalibrated value from the sensor
    int read_raw();
    //Return the calibrated value from the sensor
    int read_calibrated();
    void checkLine(int sensor);
    bool overLine();
    
    
    
  private:
  
    int pin;
    int avg_read=0;//average reading for calibration
    const int NUM_CALIBRATIONS = 75;
    
    int sl_t=400;//threshold for sensor
    int sm_t=400;
    int sr_t=400;
    bool blackLine=false;
};

Line_Sensor::Line_Sensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

int Line_Sensor::read_raw()
{
  return analogRead(pin);
}

void Line_Sensor::calibrate()
{//calibrate the sensor based on 100 readings

  for(int i=0;i<=NUM_CALIBRATIONS;i++){
    int v=read_raw();
    avg_read=avg_read+v;//cumulative sensor reading
  }
  avg_read=avg_read/NUM_CALIBRATIONS;
}

int Line_Sensor::read_calibrated()
{
  int v=read_raw();
  return v-avg_read;//sensor reading - stored value from calibration
}

void Line_Sensor::checkLine(int sensor){//different for sensor l, m and r respectively
  if(sensor==1){
    if(read_calibrated()>=sl_t){
      blackLine=true;
    }
    else{
      blackLine=false;
    }
  }
  if(sensor==2){//>800 and >400 for L and R is perfectly centred
    if(read_calibrated()>=sm_t){
      blackLine=true;
    }
    else{
      blackLine=false;
    }
  }
  if(sensor==3){
    if(read_calibrated()>=sr_t){
      blackLine=true;
    }
    else{
      blackLine=false;
    }
  }
}

bool Line_Sensor::overLine(){
  return blackLine;
}

#endif
