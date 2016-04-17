#include "Controller.h"
#include "Arduino.h"
#include "Gimbal.h"
#include "Math.h"

#define _X 0
#define _Y 1
#define _Z 2

//controller Gains
float Kp_Heading = .8;
float Kd_Heading = .5;

float Kp_Depth = 1.75;
float Kd_Depth = 0;
float Ki_Depth = 0;
float Integral_Depth = 0;
float lastDepth = 0;
float lastDepthTime = 0;

float Kp_X = 1;
float Kd_X = 0;

float Kp_Y = 1.5;
float Kd_Y = 0;

float FAuton[2] = {0, 0};

float getVXAuton( void ){
  return FAuton[_X];
}

float getVYAuton( void ){
  return FAuton[_Y];
}

float HeadingController(float HeadingDesired, float HeadingRateDesired, float heading, float HeadingRate, int mode){

  int HeadingError = (float)( HeadingDesired - heading );
  
  //handle wrap
  if(HeadingError > 180){ HeadingError -= 360; }
  if(HeadingError < -180){ HeadingError += 360; }
  //Serial.println(HeadingError);
  
  //Serial.print(HeadingError);
  //Serial.print("\t");

  float u =  Kp_Heading * HeadingError + Kd_Heading * (HeadingRateDesired - HeadingRate);
  
  //Serial.println(u);
  if(getValidDiver() || mode == 2){ return u; }
  else{  return 0;  }
  
}


float DepthController(float DepthDesired, float Depth, float DepthRateDeisred, float currTime, int mode){
  if(lastDepthTime == 0){lastDepthTime = currTime - CONTROLTIME;}
  
  Integral_Depth += (DepthDesired - Depth) * (currTime-lastDepthTime);
  Integral_Depth = constrain(Integral_Depth, -(float)2/Ki_Depth, (float)2/Ki_Depth);
  //Serial.print(Integral_Depth);

  float u =  Kp_Depth * (DepthDesired - Depth) + Ki_Depth * Integral_Depth; //+ Kd_Depth * (DepthRateDeisred - DepthRate)
  
  //Serial.println(u);
  lastDepthTime = currTime;
  return u;
}

void PositionController(float XDesired, float XPos, float YDesired, float YPos, float yaw){
  float vCurr[2];
  vCurr[0] = XDesired - XPos;
  vCurr[1] = YDesired - YPos;
  
  
  yaw = yaw*3.1415/180;
  //Serial.println(yaw);
  if(getValidDiver()){
    FAuton[0] = Kp_X * ( cos(yaw) * vCurr[_X] - sin(yaw) * vCurr[_Y]);
    FAuton[1] = Kp_Y * (-sin(yaw) * vCurr[_X] - cos(yaw) * vCurr[_Y]);
  }
  else{
    FAuton[0] = 0;
    FAuton[1] = 0;
  }
}

void PositionController2(float RDesired, float HeadingDesired, float XPos, float YPos, float VX, float VY, float yaw){
  float vCurr[2];
  float RPos = sqrt( XPos*XPos + YPos*YPos );
  float thetaPos = getHeadingToDiver();

  float uTheta =  (HeadingDesired - thetaPos);
  if(uTheta > 180) { uTheta -= 360; }
  if(uTheta < -180) { uTheta += 360; }
  uTheta = uTheta*3.1415/180;
  
  float uR = (RPos - RDesired);
  
  yaw = yaw*3.1415/180;
  //Serial.println(yaw);
  
  if(getValidDiver()){
    vCurr[_X] = -(uR*XPos - uTheta*YPos)/RPos;
    vCurr[_Y] = -(uR*YPos + uTheta*XPos)/RPos;  

//    Serial.print("\ruR: ");
//    Serial.print( uR );
//    Serial.print("\tuTheta: ");
//    Serial.print( uTheta );

    
    //Serial.println(- Kd_X*( cos(yaw) * VX - sin(yaw) * VY ))
    FAuton[_X] = Kp_X * ( cos(yaw) * vCurr[_X] - sin(yaw) * vCurr[_Y]) - Kd_X*( cos(yaw) * VX - sin(yaw) * VY );
    FAuton[_Y] = Kp_Y * (-sin(yaw) * vCurr[_X] - cos(yaw) * vCurr[_Y]) - Kd_Y*(-sin(yaw) * VX - cos(yaw) * VY );
    
    Serial.print("\tFxBody: ");
    Serial.print( FAuton[_X] );
    Serial.print("\tFyBody: ");
    Serial.print( FAuton[_Y] );
  }
  else{
    FAuton[0] = 0;
    FAuton[1] = 0;
  }
}

void resetDepthI( void ){
  Integral_Depth = 0;
}

void changeGains( float Gains[] ){
  Kp_Heading = Gains[0];
  Kd_Heading = Gains[1];
  
  Kp_Depth = Gains[2];
  Kd_Depth = Gains[3];
  Ki_Depth = Gains[4];
  Serial.println(Ki_Depth);

  Kp_X = Gains[5];
  Kd_X = Gains[6];

  Kp_Y = Gains[7];
  Kd_Y = Gains[8];
  
  resetDepthI();
  //Serial.println(Kd_X);
}


