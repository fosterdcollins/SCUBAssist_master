#include "Arduino.h"
#include "Gimbal.h"

#define VAL_MAX 2300
#define VAL_MIN 700
uint16_t checksum(byte Buff[], int num);

void Set_rpy(float roll, float pitch, float yaw){
  int rollConv = map(roll, -90, 90, VAL_MIN, VAL_MAX);
  int pitchConv = map(pitch, -180, 180, VAL_MAX, VAL_MIN);
  int yawConv = map(yaw, 180, -180, VAL_MIN, VAL_MAX);
  
      Serial.println(rollConv);
    Serial.println(pitchConv);
    Serial.println(yawConv);
    
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
