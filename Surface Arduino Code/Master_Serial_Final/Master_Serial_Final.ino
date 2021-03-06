/* YourDuino SoftwareSerialExample1
   - Connect to another Arduino running "YD_SoftwareSerialExampleRS485_1Remote"
   - Connect this unit Pins 10, 11, Gnd
   - Pin 3 used for RS485 direction control
   - To other unit Pins 11,10, Gnd  (Cross over)
   - Open Serial Monitor, type in top window. 
   - Should see same characters echoed back from remote Arduino

   Questions: terry@yourduino.com 
*/

/*-----( Import needed libraries )-----*/
#define SerialTxControl 2   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

/*-----( Declare Variables )-----*/

char charIncomingController[50];
char charIncomingVehicle[50];
char charIncomingVehicle_Default[50] = "-1,0,0,0,0;\0";
char charIn; 
boolean msgRecivedComputer = 0;
boolean msgRecivedVehicle = 0;
int n1 = 0;
int n2 = 0;
int CurrData[5];

void setup()   /****** SETUP: RUNS ONCE ******/
{
  // Start the built-in serial port, probably to Serial Monitor
  Serial.begin(19200);
  pinMode(SerialTxControl, OUTPUT);    //set SSerialTxControl to output
  digitalWrite(SerialTxControl, RS485Receive);
  
  // Start the software serial port, to another device
  Serial2.begin(115200);
  digitalWrite(SerialTxControl, RS485Transmit);  // Enable RS485 Transmit
  
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{

  delayMicroseconds(5);

  if(msgRecivedComputer){
    if(msgRecivedVehicle){
      //Serial.println("Data: ");
      Serial.println(charIncomingVehicle);
    }
    else{
      Serial.println(charIncomingVehicle_Default);
    }
    msgRecivedComputer = 0;
    msgRecivedVehicle = 0;
    
  }
}
void serialEvent(){  
  while (Serial.available()) {
    charIncomingController[n1] = Serial.read();
//    Serial.print("n = ");
//    Serial.print(n);
//    Serial.print(" Char = ");
//    Serial.println(charIncomingController[n1]);
      n1++;
    if(charIncomingController[n1-1] == ';'){
      digitalWrite(SerialTxControl, RS485Transmit);  
      charIncomingController[n1] = '\0';
      Serial2.print(charIncomingController);
//      Serial.println("finalMsg ");
//      Serial.println(charIncomingController);
      n1 = 0;
      msgRecivedComputer = 1;
      delay(8);
      digitalWrite(SerialTxControl, RS485Receive);
    }  
    
    
  }
}

void serialEvent2(){ 
  //Serial.println("here");
  while (Serial2.available()){
    //char Serial2.read();
    charIncomingVehicle[n2] = Serial2.read();
    //Serial.println(charIncomingVehicle[n2]);
//    Serial.print("n = ");
//    Serial.print(n);
//    Serial.print(" Char = ");
//    Serial.println(charIncoming[n]);
    n2++; 
    if(charIncomingVehicle[n2-1] == ';'){
      charIncomingVehicle[n2] = '\0';
      //Serial.print(charIncomingVehicle);
//      Serial.println("finalMsg ");
//      Serial.println(charIncoming);
      n2 = 0;
      msgRecivedVehicle = 1;
    }  
    
       
  }
}


