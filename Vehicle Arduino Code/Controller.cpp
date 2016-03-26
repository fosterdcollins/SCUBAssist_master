#include "Controller.h"
#include "Arduino.h"

//controller Gains
float Kp_Heading = .01;
float Kd_Heading = .001;

float Kp_Depth = 6;
float Kd_Depth = 0;
float Ki_Depth = 0;
float Integral_Depth = 0;
float lastDepth = 0;
float lastDepthTime = 0;



float HeadingController(float HeadingDesired, float HeadingRateDesired, float heading, float HeadingRate){

  int HeadingError = (float)( HeadingDesired - heading );
  
  //handle wrap
  if(HeadingError > 180){ HeadingError -= 360; }
  if(HeadingError < -180){ HeadingError += 360; }
  Serial.println(HeadingError);
  
  //Serial.print(HeadingError);
  //Serial.print("\t");

  float u =  Kp_Heading * HeadingError + Kd_Heading * (HeadingRateDesired - HeadingRate);
  Serial.println(u);
  return u;
}


float DepthController(float DepthDesired, float Depth, float DepthRateDeisred, float currTime){

  if(lastDepthTime == 0){lastDepthTime = currTime - CONTROLTIME;}
  float DepthRate = (Depth - lastDepth) / (currTime-lastDepthTime); //possible problem the first time through

  Integral_Depth += (DepthDesired - Depth) * (currTime-lastDepthTime);
  Integral_Depth = constrain(Integral_Depth, -5*Ki_Depth, -5*Ki_Depth);
  
  float u =  Kp_Depth * (DepthDesired - Depth) + Kd_Depth * (DepthRateDeisred - DepthRate) + Ki_Depth * Integral_Depth;
  
  //Serial.println(u);
  lastDepthTime = currTime;
  
  return u;
}

void resetDepthI( void ){
  Integral_Depth = 0;
}

void changeGains( float Gains[] ){
  Kp_Heading = Gains[0];
  Kd_Heading = Gains[1];
  Serial.println(Kp_Heading);
  Kp_Depth = Gains[2];
  Kd_Depth = Gains[3];
  Ki_Depth = Gains[4];
  
  resetDepthI();
}


