//Features
//Enterprise
//Pricing
// Watch 1  Star 0  Fork 1 mikehoyer/UM7-Arduino
// Code  Issues 0  Pull requests 0  Pulse  Graphs
//Branch: master Find file Copy pathUM7-Arduino/UM7.cpp
//13ee375  on Apr 4, 2015
//@mikehoyer mikehoyer initial commit
//1 contributor
//RawBlameHistory     109 lines (103 sloc)  2.7 KB
#include "UM7.h"

#define DREG_EULER_PHI_THETA 0x70  // Packet address sent from the UM7 that contains roll,pitch,yaw and rates.
#define DREG_ACCEL_PROC_X 0x65  //processed accel
UM7::UM7() : state(STATE_ZERO){}  // Default constructor

bool UM7::encode(byte c){
  switch(state){
  case STATE_ZERO:
    if (c == 's'){
      state = STATE_S;    // Entering state S from state Zero
      //Serial.println(' ');
    } else {
      state = STATE_ZERO;
    }
    return false;
  case STATE_S:
    if (c == 'n'){
      state = STATE_SN;   // Entering state SN from state S
    } else {
      state = STATE_ZERO;
    }
    return false;
  case STATE_SN:
    if (c == 'p'){
      state = STATE_SNP;    // Entering state SNP from state SN.  Packet header detected. 
    } else {
      state = STATE_ZERO;
    }
    return false;
  case STATE_SNP:
    
    state = STATE_PT;     // Entering state PT from state SNP.  Decode packet type.
    packet_type = c;
    packet_has_data = (packet_type >> 7) & 0x01;
    packet_is_batch = (packet_type >> 6) & 0x01;
    batch_length    = (packet_type >> 2) & 0x0F;
    if (packet_has_data){
      if (packet_is_batch){
        data_length = 4 * batch_length; // Each data packet is 4 bytes long
        //Serial.println(data_length);
      } else {
        data_length = 4;
      }
    } else {
      data_length = 0;
    }  
    data_index = 0;
    return false;
  case STATE_PT:
    state = STATE_DATA;   // Next state will be READ_DATA.  Save address to memory. (eg 0x70 for a DREG_EULER_PHI_THETA packet)
    address = (byte)c;
    if(address == 137){
      state = STATE_ZERO;
    }
//    Serial.print("\nAddress ");
//    Serial.println((byte)address);
    data_index = 0;
    return false;
  case STATE_DATA:      //  Entering state READ_DATA.  Stay in state until all data is read.
//    Serial.print(' ');
//    Serial.print(c, HEX);

    data[data_index] = c;
    data_index++;
    if (data_index >= 4 * batch_length){   
      state = STATE_CHK1; //  Data read completed.  Next state will be CHK1
      return false;
    }
    
    return false;
  case STATE_CHK1:      // Entering state CHK1.  Next state will be CHK0
    state = STATE_CHK0;
    checksum1 = c;
    return false;
  case STATE_CHK0:        
    state = STATE_ZERO;   // Entering state CHK0.  Next state will be state Zero.
    checksum0 = c;
    return checksum();
  }


}

bool UM7::checksum(){
  checksum10  = checksum1 << 8; // Combine checksum1 and checksum0
  checksum10 |= checksum0;
  //Serial.println("checkSum");
  computed_checksum = 's' + 'n' + 'p' + packet_type + address;
  for (int i = 0; i < data_length; i++){
    computed_checksum += data[i];
  }
  //Serial.println(checksum10);
  //Serial.println(computed_checksum);
  if (checksum10 == computed_checksum){ //checksum10 == computed_checksum
    save();
    //Serial.println("\tCheck True");
    return true;
  } else {
    //Serial.println("\tCheck False");
    return false;
  }
}

void UM7::save(){
  //Serial.println(address);
  switch(address){

  case DREG_EULER_PHI_THETA :   // data[6] and data[7] are unused.
    if(packet_is_batch){
      roll_raw = data[0] << 8;  //Deg
      roll_raw |= data[1];
      roll = roll_raw/91.02222;
      
      pitch_raw = data[2] << 8;  //Deg
      pitch_raw |= data[3];
      pitch = pitch_raw/91.02222;
      
      yaw_raw = data[4] << 8; //Deg
      yaw_raw |= data[5];
      yaw = yaw_raw/91.02222;
      
      roll_rate_raw = data[8] << 8; //deg/sec
      roll_rate_raw |= data[9];
      roll_rate = roll_rate_raw/16.0;
      
      pitch_rate_raw = data[10] << 8;  //deg/sec
      pitch_rate_raw |= data[11];
      pitch_rate = pitch_rate_raw/16.0;
      
      yaw_rate_raw = data[12] << 8;  //deg/sec
      yaw_rate_raw |= data[13];
      yaw_rate = yaw_rate_raw/16.0;
      
      *((byte*)(&EulerTime)) = data[19];
      *((byte*)(&EulerTime) + 1) = data[18];
      *((byte*)(&EulerTime) + 2) = data[17];
      *((byte*)(&EulerTime) + 3) = data[16]; 
    }
    break;
    
    case DREG_ACCEL_PROC_X :
      if(packet_is_batch){
        
        *((byte*)(&ax)) = data[3];
        *((byte*)(&ax) + 1) = data[2];
        *((byte*)(&ax) + 2) = data[1];
        *((byte*)(&ax) + 3) = data[0];

        *((byte*)(&ay)) = data[7];
        *((byte*)(&ay) + 1) = data[6];
        *((byte*)(&ay) + 2) = data[5];
        *((byte*)(&ay) + 3) = data[4];

        *((byte*)(&az)) = data[11];
        *((byte*)(&az) + 1) = data[10];
        *((byte*)(&az) + 2) = data[9];
        *((byte*)(&az) + 3) = data[8];

        *((byte*)(&AccTime)) = data[15];
        *((byte*)(&AccTime) + 1) = data[14];
        *((byte*)(&AccTime) + 2) = data[13];
        *((byte*)(&AccTime) + 3) = data[12];
      }
    break;
    default :
    //Serial.println("Unknown Packet");
    break;
  }
}



