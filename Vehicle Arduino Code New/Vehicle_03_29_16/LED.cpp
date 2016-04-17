#include <Arduino.h>

char states[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long lastTime[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void LEDblink( int LED[], char channel, int period, int num){
  if(millis() - lastTime[channel] > (unsigned long)period){
    //Serial.println(lastTime[channel]);
    if(states[channel] == 0){
      for(int i = 0; i < num; i++){
        digitalWrite(LED[i], LOW); 
        //Serial.println(sizeof(LED));
      }
      states[channel] = 1;
    }
    else{
      for(int i = 0; i < num; i++){
        digitalWrite(LED[i], HIGH); 
      }
      states[channel] = 0;
    }
    lastTime[channel] = millis();
  } 
}

void LEDon( int LED[], int num ){
    for(int i = 0; i < num; i++){
      digitalWrite(LED[i], LOW); 
    }
}
void LEDoff( int LED[], int num ){
    for(int i = 0; i < num; i++){
      digitalWrite(LED[i], HIGH); 
    }
}



