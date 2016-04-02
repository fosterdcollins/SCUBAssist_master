#include "Controller.h"
#include "Arduino.h"
#include "Gimbal.h"
#include "Math.h"

//controller Gains
float Kp_Heading = .75;
float Kd_Heading = .35;

float Kp_Depth = 2;
float Kd_Depth = 0;
float Ki_Depth = 0;
float Integral_Depth = 0;
float lastDepth = 0;
float lastDepthTime = 0;

float Kp_Lateral = 1;
float Kd_Lateral = 0;

float FAuton[2] = {0, 0};

float getVXAuton( void ){
  return FAuton[0];
}

float getVYAuton( void ){
  return FAuton[1];
}

float HeadingController(float HeadingDesired, float HeadingRateDesired, float heading, float HeadingRate, int mode){

  int HeadingError = (float)( HeadingDesired - heading );
  
  //handle wrap
  if(HeadingError > 180){ HeadingError -= 360; }
  if(HeadingError < -180){ HeadingError += 360; }
  //Serial.println(HeadingError);
  
  //Serial.print(HeadingError);
  //Serial.print("\t");

  float u =  -Kp_Heading * HeadingError + -Kd_Heading * (HeadingRateDesired - HeadingRate);
  
  //Serial.println(u);
  if(getValidDiver() || mode == 2){ return u; }
  else{  return 0;  }
  
}


float DepthController(float DepthDesired, float Depth, float DepthRateDeisred, float currTime){

  if(lastDepthTime == 0){lastDepthTime = currTime - CONTROLTIME;}
  float DepthRate = (Depth - lastDepth) / (currTime-lastDepthTime); //possible problem the first time through

  Integral_Depth += (DepthDesired - Depth) * (currTime-lastDepthTime);
  Integral_Depth = constrain(Integral_Depth, -(float)2/Ki_Depth, (float)2/Ki_Depth);
  //Serial.print(Integral_Depth);
  float u =  Kp_Depth * (DepthDesired - Depth) + Kd_Depth * (DepthRateDeisred - DepthRate) + Ki_Depth * Integral_Depth;
  
  //Serial.println(u);
  lastDepthTime = currTime;
  
  return u;
}

void PositionController(float XDesired, float XPos, float YDesired, float YPos, float yaw){
  float vCurr[2];
  vCurr[0] = XDesired - XPos;
  vCurr[1] = YDesired - YPos;
  
  
  yaw = yaw*3.1415/180;
  Serial.println(yaw);
  FAuton[0] = Kp_Lateral * (cos(yaw) * vCurr[0] + sin(yaw) * vCurr[1]);
  FAuton[1] = Kp_Lateral * (vCurr[0] *sin(yaw) - cos(yaw) * vCurr[1]);
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

  Kp_Lateral = Gains[5];
  Kd_Lateral = Gains[6];
  resetDepthI();
}




