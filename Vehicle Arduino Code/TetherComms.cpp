#include "TetherComms.h"

#define DATATRANSFACTOR 1000
static char TempBuffer[40];
int i = 0;

float ReceivedData[3][1] = {0, 0, 0};
float ZReceivedData = 0;
int StaleDataCheck = 0;
char StaleDataFlag = 0;
//boolean compassFlag = 0;

boolean PossibleMsg = 0;

boolean updateControls(void){
  //compassFlag = 0;
  //check if you have recived data in the last 1500 cycles
  if(Serial1.available() > 0){
    StaleDataCheck = STALEDATA;
    StaleDataFlag = 0;
  }
  else{
    StaleDataCheck--;
    if(StaleDataCheck < 0){
      StaleDataFlag = 1;
      MODE = 0;
      ReceivedData[0][0] = 0;
      ReceivedData[1][0] = 0;
      ReceivedData[2][0] = 0;
      ZReceivedData = 0;
    }
  }

  while(Serial1.available() > 0){ 
    //    digitalWrite(13, HIGH);
    //    Serial.print(micros());
    //    Serial.print(" ");
    char inByte = Serial1.read();
//     Serial.println("here");
    if(inByte == 'C' || inByte == 'G'){PossibleMsg = 1;}
    if(PossibleMsg){
      TempBuffer[i] = inByte;
      i++;
    }
    //Serial.print(inByte);

    if(i >= 50){
      i = 0; 
      return 0;
    }

    if(inByte == ';'){
      //Serial.println("got it");
      TempBuffer[i] = '\0';
      i = 0;
      //Serial.println(TempBuffer);
      PossibleMsg = 0;
      if(TempBuffer[0] == 'C'){
        
        long int Temp[5]; 
        i = 0;
        int Check = sscanf(TempBuffer ,"C%1d,%ld,%ld,%ld,%ld;", &Temp, &Temp[1], &Temp[2], &Temp[3], &Temp[4]);  
        
        //Serial.print("Check: ");
        //Serial.println(Check);
        if(Check == 5){ // valid data
          MODE = Temp[0]; // 0,1: received=forces; 2: received=rates for SetPoint
          
          ReceivedData[0][0] = (double)Temp[1]/DATATRANSFACTOR; 
          ReceivedData[1][0] = (double)Temp[2]/DATATRANSFACTOR;
          ReceivedData[2][0] = (double)Temp[3]/DATATRANSFACTOR; //omega
          ZReceivedData = (double)Temp[4]/DATATRANSFACTOR; //Zdot
          //Serial.println(ReceivedData[2][0]);
          return 1;
          //compassFlag = 1;        
          //Serial.flush();
          //Serial.print("got it");
          //        Serial.print(", ");
          //        Serial.print(DesiredForces[1][0]);
          //        Serial.print(", ");
          //        Serial.print(DesiredForces[2][0]);
          //        Serial.print(", ");
          //        Serial.println(ZDesiredForce);
        }
      }
      else if(TempBuffer[0] == 'G'){
        long Temp[5]; 
        float Gains[5];
        Serial.println("Gains");
        i = 0;
        int Check = sscanf(TempBuffer ,"G%ld,%ld,%ld,%ld,%ld;", &Temp, &Temp[1], &Temp[2], &Temp[3], &Temp[4]);  
        if(Check == 5){ // valid data
          for(int ii = 0; ii < 5; ii++){
            Gains[ii] = (float)Temp[ii]/DATATRANSFACTOR;  
            Serial.println(Gains[ii], 3);
          }
          Serial.println("Gains Changed");
          changeGains(Gains);
        }
    } 
  }
}

return 0;

}

//C2000,3455,4565,23;
