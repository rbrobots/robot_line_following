#include "encoders.h"
#include "pid.h"
#include "kinematics.h"
#include "line_sensors.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define BUZZER 6

//behaviours
bool c1=false;//cawlibration
bool b1=false;
bool b2=false;
bool b3=false;

unsigned long time_of_read;   // We will store a timestamp in this.
int b2_counter=0;//for outliers
int cf4=0;
float Kp_pose = 0; //Proportional gain for position controller
float Kd_pose = 0; //Derivative gain for position controller
float Ki_pose = 0; //Integral gain for position controller
//PID leftSpd(Kp_pose, Kd_pose, Ki_pose); //Position controller for left wheel position
PID pid_spd(Kp_pose, Kd_pose, Ki_pose);
Kinematics Pose; //Kinematics class to store position

#define LINE_LEFT_PIN A0 //Pin for the left line sensor ==>RIGHT
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor 
#define LINE_RIGHT_PIN A2 //Pin for the right line sensor ==> LEFT
Line_Sensor lineLeft(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
Line_Sensor lineCentre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
Line_Sensor lineRight(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

PID rightPose(Kp_pose, Kd_pose, Ki_pose); //Position controller for right wheel position
PID leftPose(Kp_pose, Kd_pose, Ki_pose); //Position controller for left wheel position

Kinematics kinematics;

float rs=0;//right speed for pid demand
float ls=0;

int n=1;
int count=0;
int cnt2=0;

float facingAngle=0;
float headingAngle=0;
float distHome=0;

#define BAUD_RATE = 115200;

void setupMotorPins()
{
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );
  pinMode( BUZZER, OUTPUT );
  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );

}

// put your setup code here, to run once:
void setup() 
{
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  pinMode( A0, INPUT );

  //Assign motor pins and set direction
  
  // These two function set up the pin
  // change interrupts for the encoders.
  setupMotorPins();
  setupEncoder0();
  setupEncoder1();

  calibrate_sensors();  

   //pid_spd.setGains(-0.15,0.00,0);//might need to increase proportion a little
   //pid_spd.setGains(-0.15,0.00,0.00000000005);//might need to increase proportion a little

  //pid_spd.setGains(-0.12,0.00001,0.0);//might need to increase proportion a little
  //pid_spd.setGains(-0.12,0.00003,0.0);//might need to increase proportion a little
  //pid_spd.setGains(-0.12,0.00003,0.0);//might need to increase proportion a little
  //pid_spd.setGains(-0.11,0.00008,0.0);//might need to increase proportion a little
  pid_spd.setGains(-0.10,0.00000,0.0);//might need to increase proportion a little
  pid_spd.setMax(63 );
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW  );
  //b1=false;
  Serial.begin( 9600 );
}
  
void loop() 
{
  kinematics.update(count_e0,count_e1);
 switch(n){
    case 1:
      detectLine();
    break;
    case 2:
      followLine1();
    break;
    case 3:
      endOfLine();
    break;
    case 4:
      turnAngle();
    break;
    case 5:
      goHome();
    break;
   
  }
  
}

void goHome(){
  if(count_e0<(distHome-125)/0.15){
    digitalWrite( R_DIR_PIN, LOW  );
   digitalWrite( L_DIR_PIN, LOW  );
   analogWrite(L_PWM_PIN,25);
   analogWrite(R_PWM_PIN,25);
  }
  else{
    analogWrite(L_PWM_PIN,0);
   analogWrite(R_PWM_PIN,0);
   analogWrite(BUZZER, 55);
  delay(200);
  analogWrite(BUZZER, 0);
  delay(200);//5 second wait before calibration starts
  analogWrite(BUZZER, 55);
  delay(200);
  analogWrite(BUZZER, 0);
  delay(5000);//5 second wait after calibration end to get robot into right position
  }
}

void turnAngle(){
  //we calculate the angle
  //AND use pythagoras to calculate the distance back home, therefore we can set encoder count back to 0
  float turnAngle=facingAngle-headingAngle;
  unsigned long time_now = millis();
  unsigned long elapsed_time = time_now - time_of_read;
  float biasT=0.03;
  
   //Serial.print(kinematics.returnX());Serial.print(" "); Serial.print(kinematics.returnY());Serial.print(" ");Serial.print(turnAngle);Serial.print(" ");Serial.println(kinematics.returnT());


if(elapsed_time<15000){
  if(turnAngle-biasT<kinematics.returnT()){
    //keep turning
   digitalWrite( R_DIR_PIN, LOW  );
   digitalWrite( L_DIR_PIN, HIGH  );
   analogWrite(L_PWM_PIN,25);
   analogWrite(R_PWM_PIN,25);
  }
  else if(turnAngle-biasT>kinematics.returnT()){
    
   digitalWrite( R_DIR_PIN, HIGH  );
   digitalWrite( L_DIR_PIN, LOW  );
   analogWrite(L_PWM_PIN,25);
   analogWrite(R_PWM_PIN,25);
  }
  else{
    //stop turning
    Serial.println("STOP TURNING");
    analogWrite(L_PWM_PIN,0);
    analogWrite(R_PWM_PIN,0);
  }
}
else{
  
  count_e0=0;
  count_e1=0;
  n++;
}
}

void endOfLine(){
  Serial.println("END OF LINE");
  analogWrite(BUZZER, 55);
  delay(200);
  analogWrite(BUZZER, 0);
  delay(200);//5 second wait before calibration starts
  analogWrite(BUZZER, 55);
  delay(200);
  analogWrite(BUZZER, 0);
  delay(5000);//5 second wait after calibration end to get robot into right position
  //float angle2=kinematics.returnT()-kinematics.turnToAngle(0);
  facingAngle=kinematics.turnToAngle(0);
  headingAngle=kinematics.returnT();
  distHome=kinematics.calcDist();
  Serial.print("Dist Home= ");Serial.println(distHome);
  count_e0=0;
  count_e1=0;
  
  n++;
}


void followLine1(){//THIS WORKS FAIRLY WELL
  //pid_spd.setGains(-0.05,1.2,0);//THIS WORKED FAIRLY WELL
  //maxspeed=15
byte l_speed=0;
byte r_speed=0;
float lc=isLineCentre2();
float target=0;
float output =pid_spd.update(target,lc);
float bias=1.0;

float turnR=16-output;
float turnL=16+output;

unsigned long time_now = millis();
unsigned long elapsed_time = time_now - time_of_read;

if(turnR>0){
  digitalWrite( R_DIR_PIN, LOW  );
}
else{
  digitalWrite( R_DIR_PIN, HIGH  );
}

if(turnL>0){
    digitalWrite( L_DIR_PIN, LOW  );
}
else{
  digitalWrite( L_DIR_PIN, HIGH  );
}

  lineCentre.checkLine(2);
  lineRight.checkLine(3);
  lineLeft.checkLine(1);

  if(lineLeft.overLine()==0&&lineCentre.overLine()==0&&lineRight.overLine()==0){
   //cnt2++;
   //if time elapsed is greated than 2 seconds where there is no reading of line then move to next function
   if(elapsed_time>2200){
      turnL=0;
      turnR=0;     
      Serial.print("Elapsed Time 02=");Serial.print(" ");Serial.println();
      digitalWrite( L_DIR_PIN, LOW  );
      digitalWrite( R_DIR_PIN, LOW  );
      analogWrite(L_PWM_PIN,turnL);
      analogWrite(R_PWM_PIN,turnR);
      delay(2000);
      n++;
   }
  }

  if(elapsed_time>2200&&n!=4){
    time_of_read = millis();
    Serial.print("Elapsed Time 01=");Serial.print(" ");Serial.println(elapsed_time);
  }
  if(turnL!=0&&turnR!=0&&n!=4){
  analogWrite(L_PWM_PIN,abs(turnL));
  analogWrite(R_PWM_PIN,abs(turnR));
}
  delay(100);

}


void detectLine(){
  if(lineCentre.read_calibrated()>350||lineRight.read_calibrated()>350||lineLeft.read_calibrated()>350){
  //if(lineCentre.overLine()==true||lineRight.overLine()==true||lineLeft.overLine()==true){//change this logic slightly    
  analogWrite(L_PWM_PIN,0);
  analogWrite(R_PWM_PIN,0);
  delay(500);
  b1=true;
  n++;
  analogWrite(L_PWM_PIN,22);
  
  delay(600);
  }
  else{
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW  );
  analogWrite(L_PWM_PIN,29);
  analogWrite(R_PWM_PIN,30);
  }
  //rotate about 90 degrees
 
  //Serial.print(lineLeft.overLine());//check sensor 1 
}



float isLineCentre2(){//checks if the line is in the centre
//close to centre is near 2000
// ex 3 task 1
float pv1=1000;
float pv2=2000;
float pv3=3000;
  
float rc=lineRight.read_calibrated();//right calibrated
float lc=lineLeft.read_calibrated();
float cc=lineCentre.read_calibrated();
float iTotal=rc+lc+cc;

float p1=lc/iTotal;
float p2=cc/iTotal;
float p3=rc/iTotal;

float line_centre=(p1*pv1)+(p2*pv2)+(p3*pv3);

//float leftOrRight=(line_centre-2000); //if negative then closer to left, if greater than 0 then closer to right

return line_centre-2000;//returns a value between -1000 and 1000
//return line_centre;
}


void calibrate_sensors(){
  //after calibration below 0 seems to be white and above 0 is black
  analogWrite(BUZZER, 55);
  delay(200);
  analogWrite(BUZZER, 0);
  delay(200);//5 second wait before calibration starts
  lineLeft.calibrate();
  lineRight.calibrate();
  lineCentre.calibrate();
  
  analogWrite(BUZZER, 55);
  delay(200);
  analogWrite(BUZZER, 0);
  delay(5000);//5 second wait after calibration end to get robot into right position
  c1=true;
}

void followLine5(){//got to end, but a bit jerky -- use this as behaviour 3?
//pid_spd.setGains(-0.005,0.2,0);
//pid_spd.setMax(30);
  
  //is line closer to left or right?

  float lineC=isLineCentre2();
  float demand=0;
  float spd=pid_spd.update(demand, lineC);//-3000 to 3000
  float bias=25;
  //switches around PID
  float r_spd=bias-spd;//bias
  float l_spd=bias+spd;//bias  
  count++;
  if(count>30){
    n++;
  }
    
  if(lineRight.read_calibrated()>=300){//turn right
  digitalWrite( L_DIR_PIN, LOW  ); //FORWARD
  digitalWrite( R_DIR_PIN, HIGH  ); //FORWARD

  r_spd=20;
  l_spd=28;
  
  }
  else if(lineLeft.read_calibrated()>=300){//turn left
  digitalWrite( L_DIR_PIN, HIGH  ); //FORWARD
  digitalWrite( R_DIR_PIN, LOW  ); //FORWAR
  r_spd=25;
  l_spd=20;
  }
  else if(lineCentre.read_calibrated()>=300){//go forward
  digitalWrite( R_DIR_PIN, LOW  ); //FORWAR
  digitalWrite( L_DIR_PIN, LOW  ); //FORWAR
    r_spd=25;
    l_spd=25;
  }
  else{
    r_spd=0;
    l_spd=0;
    b2_counter++;
    if(b2_counter>5){
      b2=true;
       digitalWrite( R_DIR_PIN, HIGH  ); //FORWAR
       digitalWrite( L_DIR_PIN, HIGH  ); //FORWAR
       analogWrite(L_PWM_PIN,20);
       analogWrite(R_PWM_PIN,20);
       delay(100);
       r_spd=0;
       l_spd=0;
       
    }
  }
  analogWrite(L_PWM_PIN,l_spd);
  analogWrite(R_PWM_PIN,r_spd);
  
  //delay(3000);
  delay(200);
}

  
