#include <SoftwareSerial.h>
#include <Servo.h>
#include "TetherComms.h"
#include "MatrixMath.h"
#include "Controller.h"
#include "UM7.h"
#include "Gimbal.h"
#include <Wire.h>
#include <stdio.h>
#include "MS5837.h"
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>
#include "SoftReset.h"
#include "LED.h"

int MODE = 0;
int lastmode;
extern int MODE;
UM7 imu;

void SendTelem(void);
void processGimbal( void );

boolean possibleMode3 = 0;
boolean enableMode3 = 0;
boolean Mode3Latch = 0;
boolean NewSetpoint = 0;
long Mode3Time = 0;

float BootSetpoint[3] = {0, 0, 0};

#define SendingEnabled 1
#define ENABLE_TRANSMIT digitalWrite(22, HIGH)
#define DISABLE_TRANSMIT digitalWrite(22, LOW)
#define BATT_PIN A0

int battery = 0;
float currDepth = 0;
float dt;
boolean compassFlag;
int readtype = 1;
int ms = 0;
int lastms = 0;
long GimbalTimeLast = millis();
int thrustersEnabled = 1;
int VthrustersEnabled = 1;


const float pi = 3.1415926;

#define ESC1 2
#define ESC2 4
#define ESC3 3
#define ESC4 5

#define MAXTHRUST 8
//float Ainv[3][3] = {
//  {
//    0.3996, 0.5774, -0.0333
//  }
//  ,
//  {
//    -0.3996, 0.5774, 0.0333
//  }
//  ,
//  {
//    0.6004, 0, 0.0333
//  }
//};
//new controller Ainv
float Ainv[3][3] = {
  {
    -0.5774, 0.3996, 0.0333
  }
  ,
  {
    -0.5774, -0.3996, -0.0333
  }
  ,
  {
    0, 0.6004, -0.0333
  }
};

float Thrust[3][1] = {
  0, 0, 0
};
float ZThrust = 0;
float u[3][1];

float SetPoint[6] = {
  0, 0, 0, 0, 0, 0
};
#define _X 0
#define _Y 1
#define YAW 2
#define YAW_RATE 3
#define DEPTH 4
#define DEPTH_RATE 5

MS5837 depthSensor;

Servo HThruster1; //Front Left
Servo HThruster2; //Front Right
Servo HThruster3; //Rear
Servo VThruster;

float timeLast = 0;

long DisableTime = 0;




/////////////////////////// Setup  /////////////////////////////
void setup() {
  HThruster1.attach(ESC1); //right
  HThruster2.attach(ESC3); //left
  HThruster3.attach(ESC2); //rear
  VThruster.attach(ESC4);  //vertical
  HThruster1.writeMicroseconds(1500);
  HThruster2.writeMicroseconds(1500);
  HThruster3.writeMicroseconds(1500);
  VThruster.writeMicroseconds(1500);
  Wire.begin();

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  initEdison();
  
  for (int led = 34; led <= 53; led++) {
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
    delay(50);
  }

  for (int led = 34; led <= 53; led++) {
    digitalWrite(led, HIGH);
  }

  while (Serial3.available()) {
    imu.encode(Serial3.read());
  }
  Serial.println(imu.yaw);
  //initGimbal(imu.yaw);

  pinMode(27, OUTPUT); //GoPro Pin
  pinMode(26, OUTPUT); //GoPro Pin
  digitalWrite(27, HIGH);
  digitalWrite(26, HIGH);

  pinMode(22, OUTPUT);
  DISABLE_TRANSMIT;
  
  depthSensor.init();
  depthSensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)

  pinMode(30, OUTPUT);
  digitalWrite(30, HIGH);


  pinMode(13, OUTPUT);

  MODE = 3;

  Serial.println("Ready!");
}





/////////////////////////// Main Loop  /////////////////////////////
void loop() {

  //Serial.println("mainloop");

  //Serial.println(getPitch());
  //Request Gimbal Positions
  //  processGimbal();
  //  if(millis() - GimbalTimeLast > 40){ //
  //    //setAnglesRel( 10, 0, imu.yaw );
  //    //setAnglesAbs( imu.yaw, 0);
  //    //operateGimbal( imu.yaw, MODE );
  //}

  //Depth Sensor
  ms = millis();
  if ( ms - lastms >= 20 ) { 
    lastms = ms;
    if (readtype == 1) {
      depthSensor.read1();
      readtype = 2;
    }
    else if (readtype == 2) {
      depthSensor.read2();
      readtype = 3;
    }
    else if (readtype == 3) {
      depthSensor.read3();
      currDepth = depthSensor.depth() * 3.28; //ft
      //Serial.println(currDepth);
      readtype = 1;
    }
  }

if ( millis() - DisableTime >= 25 ){
    DISABLE_TRANSMIT;
}



  delayMicroseconds(100);
  /////////////////////   Main Controls  ///////////////////////////

  compassFlag = updateControls();
  SendTelem();
  updateVision();
  dt = imu.AccTime - timeLast;
  if ( dt >= CONTROLTIME ) {//dt >= CONTROLTIME 
    timeLast = imu.AccTime;
    //Serial.println(MODE);


    //Serial.println(thrustersEnabled);
    switch (MODE) {
      case 0: // Debug

        digitalWrite(53, HIGH);
        digitalWrite(51, HIGH);
        digitalWrite(49, HIGH);
        digitalWrite(52, HIGH);
        digitalWrite(50, HIGH);
        digitalWrite(48, HIGH);
        //Serial.println(MODE);
        Matrix.Multiply((float*)Ainv, (float*)ReceivedData, 3, 3, 1, (float*)Thrust);
        lastmode = 0;
        break;




      case 1: // Tethered, Open Loop
        digitalWrite(53, LOW);
        digitalWrite(51, HIGH);
        digitalWrite(49, HIGH);
        digitalWrite(52, HIGH);
        digitalWrite(50, HIGH);
        digitalWrite(48, HIGH);
        //Serial.println("MODE1");
        Matrix.Multiply((float*)Ainv, (float*)ReceivedData, 3, 3, 1, (float*)Thrust);
        ZThrust = ZReceivedData;
        lastmode = 1;
        break;




      case 2: // Tethered, Closed Loop     
        //updateVision();
        digitalWrite(53, LOW);
        digitalWrite(51, LOW);
        digitalWrite(49, HIGH);
        digitalWrite(52, HIGH);
        digitalWrite(50, HIGH);
        digitalWrite(48, HIGH);
        
        if (lastmode != 2) {
          Serial.println("New Mode2 Detected...");
          SetPoint[YAW] = imu.yaw; //current Heading
          Serial.println(imu.yaw);
          SetPoint[YAW_RATE] = 0; //current Heading
          SetPoint[DEPTH] = currDepth ; //current Depth
          SetPoint[DEPTH_RATE] = 0;
          resetDepthI();
        }

        //increment heading setpoint
        SetPoint[YAW] += ReceivedData[2][0] * dt;
        SetPoint[YAW_RATE] = ReceivedData[2][0];
        if (SetPoint[YAW] >= 360) {
          SetPoint[YAW] -= 360;
        }
        if (SetPoint[YAW] < 0) {
          SetPoint[YAW] += 360;
        }

        Serial.print("dt: ");
        Serial.print(imu.AccTime);
        Serial.print("\tY: ");
        Serial.print(imu.yaw);
        Serial.print("\tYSet: ");
        Serial.print(SetPoint[YAW]);

        //increment depth setpoint
        SetPoint[DEPTH] += ZReceivedData * dt;
        SetPoint[DEPTH_RATE] = ZReceivedData;
        SetPoint[DEPTH] = constrain(SetPoint[DEPTH], -1, 12);

        Serial.print("\tD: ");
        Serial.print(SetPoint[DEPTH]);

        u[0][0] = ReceivedData[0][0];
        u[1][0] = ReceivedData[1][0];
        u[2][0] = HeadingController(SetPoint[YAW], SetPoint[YAW_RATE], imu.yaw, imu.yaw_rate, MODE);
        Serial.print("\tuHead: ");
        Serial.print(u[2][0]);

        Matrix.Multiply((float*)Ainv, (float*)u, 3, 3, 1, (float*)Thrust); //x and y same as open loop
        ZThrust = DepthController(SetPoint[DEPTH], currDepth, SetPoint[DEPTH_RATE], imu.AccTime, MODE); //DEPTH temporarily 0

        Serial.print("\tuDepth: ");
        Serial.println(ZThrust);

        lastmode = 2;
        break;



      case 3: // Autonomous
      int leds[3] = {49, 51, 53};
      //Serial.println("here");
        if(imu.pitch > 35 || imu.pitch < -35){
          if(possibleMode3 == 0){ Mode3Time = millis(); Serial.println("initial");}
          if(millis() - Mode3Time > 1500 && Mode3Latch == 0){ 
            Serial.println(enableMode3);
            if(enableMode3 == 0){ 
              enableMode3 = 1; 
              Mode3Latch = 1;
              Serial.println("Mode 3 Enabled"); 
              if(!StaleDataFlag){
                BootSetpoint[0] = Mode3RecivedData[0]; //R
                BootSetpoint[1] = Mode3RecivedData[1]; //Heading
                BootSetpoint[2] = Mode3RecivedData[2];
                Serial.println("Tether Setpoint");
                NewSetpoint = 0;
              } 
              else{
                NewSetpoint = 1;
                Serial.println("waiting for Camera Setpoint...");
              }
           }
           else if(enableMode3){ 
              enableMode3 = 0;
              Mode3Latch = 1;
              Serial.println("Mode 3 Disabled"); 
            }
          }
          possibleMode3 = 1;
        }
        else{ possibleMode3 = 0; Mode3Latch = 0;}

        if(enableMode3){
          //blue LEDs on
          int fakeHeading = imu.yaw;
          //Cartesian Coordinate Controller
  //        getRelativePose(fakeHeading, currDepth);
  //        SetPoint[_X] = -3;
  //        SetPoint[_Y] = 0;
  //        SetPoint[YAW] = getHeadingToDiver();
  //        SetPoint[YAW_RATE] = 0;
  //        SetPoint[DEPTH] = getDiverZ() - 2; //this returns in absolute depth
  //        SetPoint[DEPTH_RATE] = 0;
  //
  //        PositionController(SetPoint[_X], getDiverX(), SetPoint[_Y], getDiverY(), fakeHeading);
  //

  //        Cylindrical Coordinate Controller
          getRelativePose(fakeHeading, currDepth);

          //if you havent gotten anything from the tether and you see something
          if(NewSetpoint){//NewSetpoint && getValidDiver()){
                BootSetpoint[0] = 5.5;//sqrt(getDiverX()*getDiverX() + getDiverY()*getDiverY()); //R
                BootSetpoint[1] = 0;//getHeadingToDiver(); //Heading
                BootSetpoint[2] = getDiverZ()-currDepth;
                NewSetpoint = 0;

                Serial.println(BootSetpoint[0]);
                Serial.println(BootSetpoint[1]);
                Serial.println(BootSetpoint[2]);
                Serial.println(BootSetpoint[3]);
                Serial.println("Camera Setpoint");
          }
          else if (NewSetpoint){ LEDblink(leds, 1, 100, 3); }

          
          if(!NewSetpoint){
            LEDon(leds, 3);
            SetPoint[0] = BootSetpoint[0]; //R
            SetPoint[1] = BootSetpoint[1]; //Heading
            SetPoint[YAW] = getHeadingToDiver();
            SetPoint[YAW_RATE] = 0;
            SetPoint[DEPTH] = getDiverZ(); //this returns in absolute depth
            SetPoint[DEPTH_RATE] = 0;
    
            PositionController2(SetPoint[0], SetPoint[1], getDiverX(), getDiverY(), getDiverVX(), getDiverVY(), fakeHeading);
            
            u[0][0] = getVXAuton();
            u[1][0] = getVYAuton();
            u[2][0] = HeadingController(SetPoint[YAW], SetPoint[YAW_RATE], fakeHeading, imu.yaw_rate, MODE);
    
            Matrix.Multiply((float*)Ainv, (float*)u, 3, 3, 1, (float*)Thrust);
            ZThrust = DepthController(SetPoint[DEPTH], currDepth, SetPoint[DEPTH_RATE], imu.AccTime, MODE);
    
    
//            Serial.print("\tYaw: ");
//            Serial.print(imu.yaw, 1);
//            Serial.print("\tHead2Diver: ");
//            Serial.print(getHeadingToDiver(), 1);
//            Serial.print("\tX: ");
            Serial.print(getDiverX());
            Serial.print("\tY: ");
            Serial.print(getDiverY());
            Serial.print("\tCD: ");
            Serial.print(currDepth);
            Serial.print("\tZ: ");
            Serial.print(getDiverZ());
            Serial.print("\tuX_V: ");
            Serial.print(u[0][0]);
            Serial.print("\tuY_V: ");
            Serial.print(u[1][0]);
            Serial.print("\tuH: ");
            Serial.print(u[2][0]);
            Serial.print("\tuZ: ");
            Serial.println(ZThrust);
          }
          
          lastmode = 3;
        }
        else{
          Thrust[0][0] = 0;
          Thrust[1][0] = 0;
          Thrust[2][0] = 0;
          ZThrust = 0;
          LEDblink(leds, 1, 500, 3); 
          lastmode = 3;
        }
        break;
    }

    //Matrix.Print((float*)Thrust,3,1,"thrust");
    //Update ESCs
    if (thrustersEnabled && (MODE != 3 || enableMode3)) {
      //Serial.println(PWMVals(Thrust[0][0]));
      HThruster1.writeMicroseconds(PWMVals(Thrust[0][0]));
      HThruster2.writeMicroseconds(PWMVals(Thrust[1][0]));
      HThruster3.writeMicroseconds(PWMVals(Thrust[2][0]));
      if(VthrustersEnabled){
        VThruster.writeMicroseconds(PWMVals(ZThrust));
      }
    }
    else{
        HThruster1.writeMicroseconds(1500);
        HThruster2.writeMicroseconds(1500);
        HThruster3.writeMicroseconds(1500);
        VThruster.writeMicroseconds(1500);
    }
    //    HThruster1.writeMicroseconds(1400);
    //    HThruster2.writeMicroseconds(1400);
    //    HThruster3.writeMicroseconds(1460);
    //    VThruster.writeMicroseconds(1400);
    //Serial.println(PWMVals(ZThrust));

  }
  //HThruster1.writeMicroseconds(PWMVals(-1.0));
}

uint16_t PWMVals( float Thrust ) {
  uint16_t PWM_VAL;
  if (Thrust > 0) { //Positive
    PWM_VAL = 122.33 * pow(abs(Thrust), (float)2 / 3) + 1499;
  }
  else if (Thrust == 0)
    PWM_VAL = 1500;

  else { //positive
    PWM_VAL = 157.32 * -pow(abs(Thrust), (float)2 / 3) + 1484;
  }

  return constrain(PWM_VAL, 1000, 2000);
}


void SendTelem(void) {
  if (compassFlag && SendingEnabled) {
    ENABLE_TRANSMIT;
    char report[80];
    //Serial.println(imu.yaw);
    battery = analogRead(BATT_PIN);
    sprintf(report, "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i;\0", battery, (int)((imu.yaw - 180) * 100), (int)(imu.pitch * 100), (int)(imu.roll * 100), (int)(currDepth * 100), (int)(imu.ax * 1000), (int)(imu.ay * 1000), (int)(imu.az * 1000), (int)(SetPoint[YAW] * 10),  (int)(SetPoint[DEPTH] * 100));

    delay(7);
    Serial1.print(report);
    //Serial.println(report);
    DisableTime = millis();
    compassFlag = 0;
  }
  //delayMicroseconds(10);
}



////////// Debug Commands //////////////
void serialEvent() {
  Serial.println("Got Command");
  char inChar = Serial.read();

  switch (inChar) {
    case 'G':
      Serial.println("Gimbal Enabled");
      zeroGimbal( imu.yaw );
      enableGimbal();
      break;

    case 'g':
      Serial.println("Gimbal Disabled");
      delay(20);
      disableGimbal();
      disableGimbal();
      disableGimbal();
      disableGimbal();
      disableGimbal();
      break;

    case 'T':
      Serial.println("Thrusters Enabled");
      thrustersEnabled = 1;
      break;

    case 't':
      Serial.println("Thrusters Disabled");
      thrustersEnabled = 0;
      break;

    case 'R':
      Serial.println("Restarting.............");
      soft_restart();
      break;

    case '1':
      Serial.println("Mode 1");
      MODE = 1;
      break;

    case '2':
      Serial.println("Mode 2");
      MODE = 2;
      break;

    case '3':
      Serial.println("Mode 3");
      MODE = 3;
      break;
  }
}


void serialEvent3() {
  //digitalWrite(30, HIGH);
  //Serial.println("\tHere\t");
  while (Serial3.available()) {
    imu.encode(Serial3.read());
  }
  //digitalWrite(30, LOW);
}

//
//void serialEvent2() {
//  Serial.println("here");
//  //digitalWrite(30, HIGH);
//  //Serial.println("\tSerial2\t");
////  while (Serial2.available()) {
////    
////  }
//}


