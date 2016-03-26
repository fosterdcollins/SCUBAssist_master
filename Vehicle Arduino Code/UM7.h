#ifndef UM7_H
#define UM7_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

class UM7{
public:
  short roll_raw, pitch_raw, yaw_raw, roll_rate_raw, pitch_rate_raw, yaw_rate_raw;
  float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, ax, ay, az, EulerTime, AccTime;
  UM7();
  
  bool encode(byte c);
  
private:

  int state;
  
  enum {STATE_ZERO,STATE_S,STATE_SN,STATE_SNP,STATE_PT,STATE_DATA,STATE_CHK1,STATE_CHK0};
  
  byte packet_type;
  byte address;
  bool packet_is_batch;
  byte batch_length;
  bool packet_has_data;
  byte data[30];
  byte data_length;
  byte data_index;
  byte data_index_curr;

  byte checksum1;   // First byte of checksum
  byte checksum0;   // Second byte of checksum

  unsigned short checksum10;      // Checksum received from packet
  unsigned short computed_checksum; // Checksum computed from bytes received
  
  bool checksum(void);
  
  void save(void);
};

#endif

