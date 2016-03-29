#ifndef TetherComms
#define TetherComms

#include "Arduino.h"
#include "Controller.h"

#define SURFACEBAUD 9600
#define STALEDATA 1500


extern float ReceivedData[3][1];
extern float ZReceivedData;
extern char StaleDataFlag;
extern int MODE;

boolean updateControls(void);
void sendData(int Heading, float Pitch, float Roll, int depth);

#endif
