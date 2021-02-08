#ifndef _Kinematics
#define _Kinematics_h

class Kinematics
{
  public:
    Kinematics();//class constructor
    void setParameters(float x, float y, float t);
    void update(float l_encoder, float r_encoder);
    float returnX();
    float returnY();
    float returnT();
    float turnToAngle(float tP);
    float currentAngle(float a);
    float calcDist();

  private:
    //You may want to use these variables
  const int WHEEL_DIAMETER = 70;//14.6
  const int WHEEL_DISTANCE = 146;
  const int GEAR_RATIO = 120;
  const int COUNTS_PER_SHAFT_REVOLUTION = 12;
  const int COUNTS_PER_WHEEL_REVOLUTION =  1440;
  const float COUNTS_PER_MM = 1/0.15;
  const float MM_PER_COUNT = 0.15;//0.1527
  float r_last_encoder=0;
  float l_last_encoder=0;
  float r_encoder=0;
  float l_encoder=0;
  float X=0;
  float Y=0;
  float T=0;
  
    //Private variables and methods go here
};

Kinematics::Kinematics()
{
  //setParameters(x,y,t);  
}

void Kinematics::update(float l_encoder, float r_encoder){//input lencoder, rencoder
//this class should update the position

//calculate change in encoder

float d =(((l_encoder-l_last_encoder)+(r_encoder-r_last_encoder))/2)*MM_PER_COUNT;//change into counts from MM
float t_new=(T+(((l_encoder-l_last_encoder)-(r_encoder-r_last_encoder))/WHEEL_DISTANCE)*MM_PER_COUNT);

float x_new=X+d*cos(t_new);//old theta or new theta??
float y_new=Y+d*sin(t_new);

X=x_new;
Y=y_new;
T=t_new;

l_last_encoder=l_encoder;
r_last_encoder=r_encoder;
//output: xnew, ynew
}

void Kinematics::setParameters(float x, float y, float t){
  X=x;
  Y=y;
  T=t;
}

float Kinematics::returnX(){
  return X;
}

float Kinematics::returnY(){
  return Y;
}

float Kinematics::returnT(){
  return T;
}

float Kinematics::turnToAngle(float tP){
  //current angle
  //float tP=0;//target position
  float dx=tP-X;
  float dy=tP-Y;
  float angle=atan2(dy,dx);

  return angle;
  //return angle to encoder counts
}

float Kinematics::currentAngle(float a){//calculate current angle for rotation home
  
}

float Kinematics::calcDist(){//calculate the distance back
  float dist=sqrt(X*X+Y*Y);

  return dist;
}

#endif
