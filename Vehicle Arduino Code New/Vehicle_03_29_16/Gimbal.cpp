

#include <inttypes.h>

#include "Arduino.h"
#include "Gimbal.h"
#include "MatrixMath.h"

#define VAL_MAX 2300
#define VAL_MIN 700
#define DATATRANSFACTOR 1000
#define GIMBALSTARTOFFSET 193

//HardwareSerial &Edison = Serial2;
SoftwareSerial Gimbal(11, 10); // RX, TX

SBGC_cmd_control_t c;
SerialCommand cmd;
static SBGC_cmd_realtime_data_t rt_data;

static uint8_t is_connected = 0;
float euler[3] = {0, 0, 0};



float eulerGimbal[3] = {0, 0, 0}; //roll pitch yaw
float eulerGimbal_Set[3] = {0, 0, 0}; //roll pitch yaw
float Pdiver_cam[3] = {0, 0, 0}; //x y z (ft)
float Pdiver[3] = {0, 0, 0};

float V[3] = {0, 0, 0};
unsigned long LastTime = 0;
float Vfilter = .5;

float gimbalOffset = 0;
float initHeading = 0;

boolean GoodPose = 0;
float LastValidHeading = 0;
float LastValidDepth = 0;

uint16_t checksum(byte Buff[], int num);


boolean getValidDiver(void){
  return GoodPose;
}

void getRelativePose(float heading, float depth){

  heading = heading / 180 * 3.1415; 
  float LastPose[3] = {Pdiver[_X], Pdiver[_Y], depth};
  
  if(GoodPose){
    Pdiver[0] =  -(-Pdiver_cam[_X]*sin(heading + eulerGimbal_Set[_YAW]) + Pdiver_cam[_Y]*cos(heading + eulerGimbal_Set[_YAW])*sin(eulerGimbal[_PITCH]) + Pdiver_cam[_Z]*cos(heading + eulerGimbal_Set[_YAW])*cos(eulerGimbal_Set[_PITCH])) ;
    Pdiver[1] =  -(-Pdiver_cam[_X]*cos(heading + eulerGimbal_Set[_YAW]) - Pdiver_cam[_Y]*sin(heading + eulerGimbal_Set[_YAW])*sin(eulerGimbal[_PITCH]) - Pdiver_cam[_Z]*sin(heading + eulerGimbal_Set[_YAW])*cos(eulerGimbal_Set[_PITCH])) ;
      
    float deltaZ =  (-Pdiver_cam[_Y]*cos(eulerGimbal_Set[_PITCH]) + Pdiver_cam[_Z]*sin(eulerGimbal_Set[_PITCH]));
    Pdiver[2] = depth - deltaZ;
    
    V[_X] = Vfilter*(Pdiver[_X]-LastPose[_X])/(micros() - LastTime)*1000000 + (1-Vfilter)*V[_X];
    V[_Y] = Vfilter*(Pdiver[_Y]-LastPose[_Y])/(micros() - LastTime)*1000000 + (1-Vfilter)*V[_Y];
    V[_Z] = Vfilter*(depth-LastPose[_Z])/(micros() - LastTime)*1000000 + (1-Vfilter)*V[_Z];
    
    LastTime = micros();
  }
  
}

float getDiverX(void){
  if(GoodPose) { return  Pdiver[_X]; }
  else{ return  0; }
}

float getDiverY(void){
  if(GoodPose) { return  Pdiver[_Y]; }
  else{ return  0; }
}

float getDiverZ(void){
  if(GoodPose){
    LastValidDepth = Pdiver[_Z];
    return  Pdiver[_Z];
  }
  else{
    return LastValidDepth;
  }
}

float getDiverVX(void){
  return  V[_X];
}

float getDiverVY(void){
  return  V[_Y];
}

float getDiverVZ(void){
  return  V[_Z];
}

float getHeadingToDiver(void){
  float head = atan2(-getDiverY(), -getDiverX())*180/3.1415;  
  if(head < 0) head += 360;
  head = 360 - head;
  //Serial.println(diff);
  if(GoodPose){ 
    LastValidHeading = head;
    return head;
    } 
  else{ return LastValidHeading; }
}

float getHeadingToDiverRel(float heading){
  float diveHead = atan2(-getDiverY(), -getDiverX())*180/3.1415;  
  if(diveHead < 0) diveHead += 360;
  diveHead = 360 - diveHead;
  //Serial.println(diff);
  if(GoodPose){ 
    LastValidHeading = diveHead-heading;
    if(LastValidHeading < -180) LastValidHeading += 360;
    if(LastValidHeading > 180) LastValidHeading -= 360;
    return LastValidHeading;
    } 
  else{ return LastValidHeading; }
}

float getPitch( void ){
  return eulerGimbal[1]; 
}

float getYaw( void ){
  return eulerGimbal[2]; 
}

void requestAngles( void ){
  Serial.println("request");
  SerialCommand cmd;
  cmd.init(73);
  sbgc_parser.send_cmd(cmd, 0);
}

void operateGimbal( float heading, int Mode ){
  if(Mode == 1 || Mode == 2){
    setAnglesRel( 0, 0, heading );
  }
  if(Mode == 3){
    setAnglesRel(getHeadingToDiverRel(heading),0, heading);
  }
}

void setAnglesRel( float yaw, float pitch, float heading ){
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(-pitch);
  c.angleYAW = SBGC_DEGREE_TO_ANGLE(yaw + heading + gimbalOffset);
  SBGC_cmd_control_send(c, sbgc_parser);
  eulerGimbal_Set[_PITCH] = pitch;
  eulerGimbal_Set[_YAW] = yaw;
  //Serial.println(yaw + heading + gimbalOffset);
}

void setAnglesAbs( float yaw, float pitch){
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(-pitch);
  //c.angleYAW = SBGC_DEGREE_TO_ANGLE(yaw + gimbalOffset);

  SBGC_cmd_control_send(c, sbgc_parser);
}

void initGimbal( float heading ){ 
  Serial.println("Gimbal Initialized...");
  Gimbal.begin(19200);
  pinMode(28, OUTPUT); //Gimbal
  powerOnGimbal();
  delay(1000);
  SBGC_Demo_setup(&Gimbal);
    
  c.mode = SBGC_CONTROL_MODE_ANGLE;
  c.speedROLL = c.speedPITCH = c.speedYAW = 50 * SBGC_SPEED_SCALE;
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(0);
  c.angleROLL = SBGC_DEGREE_TO_ANGLE(0);
  SBGC_cmd_control_send(c, sbgc_parser);
  pinMode(35, OUTPUT);
  //zeroGimbal(heading);
}

void zeroGimbal(float heading){
    disableGimbal();
    delay(500);
    while( getYaw() == 0){
      digitalWrite(35, LOW);
      requestAngles();
      delay(40);
      processGimbal();
    }
    Serial.println(heading);
    digitalWrite(35, HIGH);
    initHeading = heading;
    gimbalOffset = getYaw() + GIMBALSTARTOFFSET - heading;
    enableGimbal();
    c.anglePITCH = SBGC_DEGREE_TO_ANGLE(0);
    c.angleYAW = SBGC_DEGREE_TO_ANGLE(gimbalOffset + heading);
    SBGC_cmd_control_send(c, sbgc_parser);
  }
  
void processGimbal() {
  //Serial.println("here");
  while(sbgc_parser.read_cmd()) {
    
    SerialCommand &cmd = sbgc_parser.in_cmd;
    //cmd = sbgc_parser.in_cmd;
    
    uint8_t error = 0;
    Serial.println(cmd.id);  
    switch(cmd.id) {
    // Receive realtime data
    case 73:
      error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
      //Serial.write(cmd.data, cmd.len); 
      eulerGimbal[0] = (int)((cmd.data[1] << 8) | cmd.data[0])  * SBGC_ANGLE_DEGREE_SCALE;
      eulerGimbal[1] = (int)((cmd.data[7] << 8) | cmd.data[6])  * SBGC_ANGLE_DEGREE_SCALE;
      eulerGimbal[2] = (int)((cmd.data[13] << 8) | cmd.data[12])* SBGC_ANGLE_DEGREE_SCALE;
      //Serial.println("\n");
      Serial.println(eulerGimbal[0]);
      Serial.println(eulerGimbal[1]);
      Serial.println(eulerGimbal[2]);
      break;
    }
  }
}

void centerGimbal( void ){
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(0);
  c.angleYAW = SBGC_DEGREE_TO_ANGLE(0);
}

void powerOnGimbal(void){
  digitalWrite(28, HIGH);
}

void powerOffGimbal(void){
  digitalWrite(28, LOW);
}

void enableGimbal(void){
  Serial.println("Enable Gimbal");
  SerialCommand cmd;
  cmd.init(77);
  sbgc_parser.send_cmd(cmd, 0);
  sbgc_parser.send_cmd(cmd, 0);
}

void disableGimbal(void){
  Serial.println("Disable Gimbal");
  SerialCommand cmd;
  cmd.init(109);
  sbgc_parser.send_cmd(cmd, 0);
  sbgc_parser.send_cmd(cmd, 0);
}




/////////////////  Edison Comms  ////////////////////////////

void initEdison(void){
    Serial2.begin(115200);
}

static char TempBuffer_Edison[40];
boolean PossibleMsg_Edison = 0;
int i_Edison = 0;

boolean updateVision(void){
  
  while(Serial2.available() > 0){ 
    //    digitalWrite(13, HIGH);
    //    Serial.print(micros());
    //    Serial.print(" ");
    char inByte = Serial2.read();
    //Serial.println("here");
    if(inByte == 'A' || inByte == 'N'){
      PossibleMsg_Edison = 1;
    }
    if(PossibleMsg_Edison){
      TempBuffer_Edison[i_Edison] = inByte;
      i_Edison++;
    }
    //Serial.print(inByte);

    if(i_Edison >= 50){
      i_Edison = 0; 
      return 0;
    }

    if(inByte == ';'){
      //Serial.println("got it");
      TempBuffer_Edison[i_Edison] = '\0';
      i_Edison = 0;
      //Serial.println(TempBuffer_Edison);
      PossibleMsg_Edison = 0;
      
      if(TempBuffer_Edison[0] == 'A'){
        long int Temp[3]; 
        i_Edison = 0;
        //Serial.println(TempBuffer_Edison);
        int Check = sscanf(TempBuffer_Edison, "A%ld,%ld,%ld;\0", &Temp, &Temp[1], &Temp[2]);  

//        Serial.print("Check: ");
//        Serial.println(Check);
        if(Check == 3){ // valid data
          Pdiver_cam[0] = (float)Temp[0]/DATATRANSFACTOR; 
          Pdiver_cam[1] = (float)Temp[1]/DATATRANSFACTOR;
          Pdiver_cam[2] = (float)Temp[2]/DATATRANSFACTOR;
          GoodPose = 1;
          digitalWrite(34, HIGH); //light on
          digitalWrite(35, LOW);
          //Serial.println(ReceivedData[2][0]);        
          //Serial.flush();
          //Serial.print("got it");
//          Serial.print(Pdiver[0]);
//          Serial.print(", ");
//          Serial.print(Pdiver[1]);
//          Serial.print(", ");
//          Serial.print(Pdiver[2]);
//          Serial.println(" ");
          return 1;
        }
      }
      else if(TempBuffer_Edison[0] == 'N'){
        //Serial.print("here");
        //Serial.println(TempBuffer_Edison);
        GoodPose = 0;
        digitalWrite(34, LOW); //red ON
        digitalWrite(35, HIGH); //blue OFF
        return 0;
      }
    }
  return 0;
  }
}
