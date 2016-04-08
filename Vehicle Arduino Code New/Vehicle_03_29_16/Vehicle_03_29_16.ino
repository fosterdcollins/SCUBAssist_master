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

int MODE = 0;
int lastmode;
extern int MODE;
UM7 imu;

void SendTelem(void);
void processGimbal( void );

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
boolean thrustersEnabled = 0;
boolean VthrustersEnabled = 0;

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
  Serial2.begin(115200);
  Serial3.begin(115200);


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

  MODE = 1;

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
  if ( ms - lastms >= 200 ) { //
    lastms = ms;
    if (readtype == 1) {
      depthSensor.read1();/////
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
  dt = imu.AccTime - timeLast;
  if ( dt >= CONTROLTIME ) {//dt >= CONTROLTIME 
    timeLast = imu.AccTime;
    Serial.println(MODE);



    switch (MODE) {
      case 0: // Debug
        //Serial.println("MODE0");
        Matrix.Multiply((float*)Ainv, (float*)ReceivedData, 3, 3, 1, (float*)Thrust);
        lastmode = 0;
        break;




      case 1: // Tethered, Open Loop
        //Serial.println("MODE1");
        Matrix.Multiply((float*)Ainv, (float*)ReceivedData, 3, 3, 1, (float*)Thrust);
        ZThrust = ZReceivedData;
        lastmode = 1;
        break;




      case 2: // Tethered, Closed Loop
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
        ZThrust = DepthController(SetPoint[DEPTH], currDepth, SetPoint[DEPTH_RATE], imu.AccTime); //DEPTH temporarily 0

        Serial.print("\tuDepth: ");
        Serial.println(ZThrust);

        lastmode = 2;
        break;



      case 3: // Autonomous
        int fakeHeading = 0;
        //Cartesian Coordinate Controller
//        getRelativePose(fakeHeading, currDepth);
//        SetPoint[_X] = -3;
//        SetPoint[_Y] = 0;
//        SetPoint[YAW] = getHeadingToDiver();
//        SetPoint[YAW_RATE] = 0;
//        SetPoint[DEPTH] = getDiverZ() - 2; //this returns in absolute depth
//        SetPoint[DEPTH_RATE] = 0;
//
//        PositionController2(SetPoint[_X], getDiverX(), SetPoint[_Y], getDiverY(), fakeHeading);

        //Cylindrical Coordinate Controller
        getRelativePose(fakeHeading, currDepth);
        SetPoint[0] = Mode3RecivedData[0]; //R
        SetPoint[1] = Mode3RecivedData[1]; //Heading
        SetPoint[YAW] = getHeadingToDiver();
        SetPoint[YAW_RATE] = 0;
        SetPoint[DEPTH] = getDiverZ() - 2; //this returns in absolute depth
        SetPoint[DEPTH_RATE] = 0;

        PositionController2(SetPoint[0], SetPoint[1], getDiverX(), getDiverY(), imu.yaw);
        
        u[0][0] = getVXAuton();
        u[1][0] = getVYAuton();
        u[2][0] = HeadingController(SetPoint[YAW], SetPoint[YAW_RATE], imu.yaw, imu.yaw_rate, MODE);

        Matrix.Multiply((float*)Ainv, (float*)u, 3, 3, 1, (float*)Thrust);

        ZThrust = DepthController(SetPoint[DEPTH], currDepth, SetPoint[DEPTH_RATE], imu.AccTime);


        Serial.print("Yaw: ");
        Serial.print(imu.yaw, 1);
        Serial.print("\tHead2Diver: ");
        Serial.print(getHeadingToDiver(), 1);
        Serial.print("\tX: ");
        Serial.print(getDiverX());
        Serial.print("\tY: ");
        Serial.print(getDiverY());
        Serial.print("\tCD: ");
        Serial.print(currDepth);
        Serial.print("\tZ: ");
        Serial.print(getDiverZ());
        Serial.print("\tuX: ");
        Serial.print(u[0][0]);
        Serial.print("\tuY: ");
        Serial.print(u[1][0]);
        Serial.print("\tuH: ");
        Serial.print(u[2][0]);
        Serial.print("\tuZ: ");
        Serial.println(ZThrust);

        lastmode = 3;
        break;
    }

    //Matrix.Print((float*)Thrust,3,1,"thrust");
    //Update ESCs
    if (thrustersEnabled) {
      //Serial.println(PWMVals(Thrust[0][0]));
      HThruster1.writeMicroseconds(PWMVals(Thrust[0][0]));
      HThruster2.writeMicroseconds(PWMVals(Thrust[1][0]));
      HThruster3.writeMicroseconds(PWMVals(Thrust[2][0]));
      if(VthrustersEnabled){
        VThruster.writeMicroseconds(PWMVals(ZThrust));
      }
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

void serialEvent2() {
  //digitalWrite(30, HIGH);
  //Serial.println("\tSerial2\t");
  while (Serial2.available()) {
    updateVision();
  }

}
