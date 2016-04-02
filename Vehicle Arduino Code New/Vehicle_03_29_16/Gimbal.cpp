#include "Arduino.h"
#include "Gimbal.h"
#include "MatrixMath.h"

#define VAL_MAX 2300
#define VAL_MIN 700
#define DATATRANSFACTOR 1000

float eulerGimbal[3] = {0, 0, 0}; //roll pitch yaw
float Pdiver_cam[3] = {0, 0, 0}; //x y z (ft)
float Pdiver[3] = {0, 0, 0};

boolean GoodPose = 0;

uint16_t checksum(byte Buff[], int num);

boolean getValidDiver(void){
  return GoodPose;
}

void getRelativePose(float heading, float depth){

  heading = heading/180*3.1415;
  
  if(GoodPose == 1){
    Pdiver[0] =  Pdiver_cam[0]*sin(heading - eulerGimbal[2]) - Pdiver_cam[2]*cos(heading - eulerGimbal[2])*cos(eulerGimbal[1]) - Pdiver_cam[1]*cos(heading - eulerGimbal[2])*sin(eulerGimbal[1]);
    Pdiver[1] = -Pdiver_cam[0]*cos(heading - eulerGimbal[2]) - Pdiver_cam[2]*sin(heading - eulerGimbal[2])*cos(eulerGimbal[1]) - Pdiver_cam[2]*sin(heading - eulerGimbal[2])*sin(eulerGimbal[1]);
    
    float deltaZ =  Pdiver_cam[1]*cos(eulerGimbal[1]) - Pdiver_cam[2]*sin(eulerGimbal[1]);
    Pdiver[2] = depth + deltaZ;
  }
  else{
     Pdiver[0] = 0;
     Pdiver[1] = 0;
     Pdiver[2] = 0;
  }
  
}

float getDiverX(void){
  return  Pdiver[0];
}

float getDiverY(void){
  return  Pdiver[1];
}

float getDiverZ(void){
  return  Pdiver[2];
}

float getHeadingToDiver(void){
  float diff = atan2(-getDiverY(), -getDiverX())*180/3.1415;  
  if(diff < 0) diff += 180;
  Serial.println(diff);
  return diff;
  
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


/////////////////  Edison Comms  ////////////////////////////


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
        GoodPose = 0;
        digitalWrite(34, LOW); //red ON
        digitalWrite(35, HIGH); //blue OFF
        return 0;
      }
    }
  return 0;
  }
}
