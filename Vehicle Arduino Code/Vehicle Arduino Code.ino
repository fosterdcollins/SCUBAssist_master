#include <Servo.h>
#include "TetherComms.h"
#include "MatrixMath.h"
#include "Controller.h"
#include "UM7.h"
#include <Wire.h>
#include <stdio.h>

#include "MS5837.h"

int MODE = 0;
int lastmode;
extern int MODE;

void SendTelem(void);

#define SendingEnabled 1
#define ENABLE_TRANSMIT digitalWrite(22, HIGH)
#define DISABLE_TRANSMIT digitalWrite(22, LOW)
#define BATT_PIN A0
int battery = 0;
float currDepth = 0;
float dt;

boolean compassFlag = 0;

const float pi = 3.1415926;

#define ESE1 3
#define ESE2 4
#define ESE3 5
#define ESE4 2

#define MAXTHRUST 8
float Ainv[3][3] = {
  {
    0.3996, 0.5774, -0.0333
  }
  ,
  {
    -0.3996, 0.5774, 0.0333
  }
  ,
  {
    0.6004, 0, 0.0333
  }
};

float Thrust[3][1] = {
  0, 0, 0};
float ZThrust = 0;
float u[3][1];

float SetPoint[6] = {
  0, 0, 0, 0, 0, 0};
#define X 0
#define Y 1
#define YAW 2
#define YAW_RATE 3
#define DEPTH 4
#define DEPTH_RATE 5

UM7 imu;
MS5837 depthSensor;

Servo HThruster1; //Front Left
Servo HThruster2; //Front Right
Servo HThruster3; //Rear
Servo VThruster;

float timeLast = 0;



void setup() {
  HThruster1.attach(ESE1); //right
  HThruster2.attach(ESE3); //left
  HThruster3.attach(ESE2); //rear
  VThruster.attach(ESE4);  //vertical
  HThruster1.writeMicroseconds(1500);
  HThruster2.writeMicroseconds(1500);
  HThruster3.writeMicroseconds(1500);
  VThruster.writeMicroseconds(1500);
  Wire.begin();

  delay(5000);

  pinMode(53, OUTPUT);
  pinMode(22, OUTPUT);
  DISABLE_TRANSMIT;

  depthSensor.init();
  depthSensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)  

  pinMode(30, OUTPUT);
  digitalWrite(30, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  pinMode(13, OUTPUT);

  MODE = 1;

  digitalWrite(53,LOW);
  Serial.println("Ready!");
}




void loop() {
  //ENABLE_TRANSMIT;
  //Serial1.println("hi;");
  //Serial.println("hi");
  //Serial.println("mainloop");
  //dt = imu.AccTime - timeLast;

  //Transmit data
  dt = millis() - timeLast;
  if ( dt >= CONTROLTIME ) {
    //timeLast = imu.AccTime;
    timeLast = millis();
    //Serial.println(dt);
    //depthSensor.read();
    //currDepth = depthSensor.depth();

    switch (MODE) {
    case 0: // Debug
      compassFlag = updateControls();
      SendTelem();
      //Serial.println("MODE0");
      Matrix.Multiply((float*)Ainv, (float*)ReceivedData, 3, 3, 1, (float*)Thrust);
      lastmode = 0;
      break;

    case 1: // Tethered, Open Loop
      compassFlag = updateControls();
      //Serial.println(compassFlag);
      SendTelem();
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

      compassFlag = updateControls();
      SendTelem();

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
      Serial.print(SetPoint[YAW]);

      //increment depth setpoint
      SetPoint[DEPTH] += ZReceivedData * dt;
      SetPoint[DEPTH_RATE] = ZReceivedData;
      SetPoint[DEPTH] = constrain(SetPoint[DEPTH], -1, 4);

      Serial.print("\tD: ");
      Serial.print(SetPoint[DEPTH]);

      u[0][0] = ReceivedData[0][0];
      u[1][0] = ReceivedData[1][0];
      u[2][0] = HeadingController(SetPoint[YAW], SetPoint[YAW_RATE], imu.yaw, imu.yaw_rate);
      Serial.print("\tu: ");
      Serial.println(u[2][0]);

      Matrix.Multiply((float*)Ainv, (float*)u, 3, 3, 1, (float*)Thrust); //x and y same as open
      ZThrust = DepthController(SetPoint[DEPTH], currDepth, SetPoint[DEPTH_RATE], imu.AccTime); //DEPTH temporarily 0

      lastmode = 2;
      break;

    case 3: // Autonomous
      lastmode = 3;
      break;
    }

    //Matrix.Print((float*)Thrust,3,1,"thrust");
    //Update ESCs
//    HThruster1.writeMicroseconds(PWMVals(Thrust[0][0]));
//    HThruster2.writeMicroseconds(PWMVals(Thrust[1][0]));
//    HThruster3.writeMicroseconds(PWMVals(Thrust[2][0]));
//    VThruster.writeMicroseconds(PWMVals(ZThrust));

    //Serial.println(PWMVals(ZThrust));

  }
  //HThruster1.writeMicroseconds(PWMVals(-1.0));
    HThruster1.writeMicroseconds(1200);
    HThruster2.writeMicroseconds(1200);
    HThruster3.writeMicroseconds(1200);
    VThruster.writeMicroseconds(1200);
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
    Serial.println("ping;");
    Serial1.print("ping;");
    char report[80];
    //Serial.println(imu.yaw);
    battery = analogRead(BATT_PIN);
    sprintf(report, "%i,%i,%i,%i,%i,%i,%i,%i,%i,%i;\0", battery, (int)((imu.yaw-180) * 100), (int)(imu.pitch * 100), (int)(imu.roll * 100), (int)(currDepth * 100), (int)(imu.ax * 1000), (int)(imu.ay * 1000), (int)(imu.az * 1000), (int)(SetPoint[YAW]*10),  (int)(SetPoint[DEPTH]*100));

    delay(5);
    Serial1.println(report);
    delay(25);
    DISABLE_TRANSMIT;
    compassFlag = 0;
  }
  //delayMicroseconds(10);
}

void serialEvent3() {
  //digitalWrite(30, HIGH);
  //Serial.println("\tHere\t");
  while (Serial3.available()) {
    imu.encode(Serial3.read());
  }
  //digitalWrite(30, LOW);
}
