#include "Arduino.h"
#include "Gimbal.h"
#include "MatrixMath.h"

#define VAL_MAX 2300
#define VAL_MIN 700
#define DATATRANSFACTOR 1000
#define _X 0
#define _Y 1
#define _Z 2
#define _ROLL 0
#define _PITCH 1
#define _YAW 2

 /////////////     RADIANS!!!!  //////////////////
float eulerGimbal[3] = {0, 0, 0}; //roll pitch yaw
 /////////////     RADIANS!!!!  //////////////////
 
float Pdiver_cam[3] = {0, 0, 0}; //x y z (ft)
float Pdiver[3] = {0, 0, 0};

boolean GoodPose = 0;
float LastValidHeading = 0;

uint16_t checksum(byte Buff[], int num);

boolean getValidDiver(void){
  return GoodPose;
}

void getRelativePose(float heading, float depth){

  heading = (heading) / 180 * 3.1415; 
  
  if(GoodPose){
    Pdiver[0] =  -(-Pdiver_cam[_X]*sin(heading + eulerGimbal[_YAW]) + Pdiver_cam[_Y]*cos(heading + eulerGimbal[_YAW])*sin(eulerGimbal[_PITCH]) + Pdiver_cam[_Z]*cos(heading + eulerGimbal[_YAW])*cos(eulerGimbal[_PITCH])) ;
    Pdiver[1] =  -(-Pdiver_cam[_X]*cos(heading + eulerGimbal[_YAW]) - Pdiver_cam[_Y]*sin(heading + eulerGimbal[_YAW])*sin(eulerGimbal[_PITCH]) - Pdiver_cam[_Z]*sin(heading + eulerGimbal[_YAW])*cos(eulerGimbal[_PITCH])) ;
    
    float deltaZ =  -Pdiver_cam[_Y]*cos(eulerGimbal[_PITCH]) + Pdiver_cam[_Z]*sin(eulerGimbal[_PITCH]);
    Pdiver[2] = depth - deltaZ;
  }
  else{
     Pdiver[0] = 0;
     Pdiver[1] = 0;
     Pdiver[2] = 0;
  }
  
}

float getDiverX(void){
  return  Pdiver[_X];
}

float getDiverY(void){
  return  Pdiver[_Y];
}

float getDiverZ(void){
  return  Pdiver[_Z];
}

float getHeadingToDiver(void){
  float head = atan2(-getDiverY(), -getDiverX())*180/3.1415;  
  if(head < 0) head += 360;
  head = 360 - head;
  
  Serial.println(head);
  if(GoodPose){ 
    LastValidHeading = head;
    return head;
    } 
  else{ return LastValidHeading; }
  
}

void Set_rpy(float roll, float pitch, float yaw){
  
  int rollConv = map(roll, -90, 90, VAL_MIN, VAL_MAX);
  int pitchConv = map(pitch, -180, 180, VAL_MAX, VAL_MIN);
  int yawConv = map(yaw, 180, -180, VAL_MIN, VAL_MAX);
//  
//    Serial.println(rollConv);
//    Serial.println(pitchConv);
//    Serial.println(yawConv);
    
  byte data[11];
  data[0] = 0xFA;
  data[1] = 0x06;
  data[2] = 0x12;
  data[3] = (byte)(pitchConv & 0x00FF);
  data[4] = (byte)(pitchConv >> 8);
  data[5] = (byte)(rollConv & 0x00FF);
  data[6] = (byte)(rollConv >> 8);
  data[7] = (byte)(yawConv & 0x00FF);
  data[8] = (byte)(yawConv >> 8);
  
  uint16_t crc = checksum(data, 9);
  Serial.println(crc);
  
  data[9] = (byte)(crc & 0x00FF);
  data[10] = (byte)(crc >> 8);
  Serial3.write(data, 11);
  return;
}

void centerGimbal(){
  
  byte data[11];
  data[0] = 0xFA;
  data[1] = 0x06;
  data[2] = 0x12;
  
  uint16_t crc = checksum(data, 9);
  
  data[9] = (byte)(crc & 0x00FF);
  data[10] = (byte)(crc >> 8);
  
  Serial3.write(data, 11);
  return;
}

uint16_t checksum(byte Buff[], int num){
  uint16_t checksum = 0;
  
  for(int ibyte = 0; ibyte < num; ibyte++){
    char tempByte = Buff[ibyte];
     //Serial.println("Byte");
     //Serial.println(checksum);
    for(int i = 0; i < 8; i++){
      checksum += (tempByte >> i) && 0x01;
      //Serial.println(checksum);
    }
  }
  return checksum;
}


/////////////////////  Edison Comms  ////////////////////////////


static char TempBuffer_Edison[40];
boolean PossibleMsg_Edison = 0;
int i_Edison = 0;

boolean updateVision(void){

  while(Serial2.available() > 0){ 
    //    digitalWrite(13, HIGH);
    //    Serial.print(micros());
    //    Serial.print(" ");
    char inByte = Serial2.read();
    //     Serial.println("here");
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
          Pdiver_cam[_X] = (float)Temp[0]/DATATRANSFACTOR; 
          Pdiver_cam[_Y] = (float)Temp[1]/DATATRANSFACTOR;
          Pdiver_cam[_Z] = (float)Temp[2]/DATATRANSFACTOR;
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
        GoodPose = 0;
        digitalWrite(34, LOW); //red ON
        digitalWrite(35, HIGH); //blue OFF
        return 0;
      }
    }
  return 0;
  }
}
