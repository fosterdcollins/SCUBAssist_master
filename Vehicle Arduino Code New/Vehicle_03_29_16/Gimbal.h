#include <SBGC.h>
#include <SBGC_Arduino.h>

#define P_MAX 90
#define P_MIN -86

#define R_MAX 45
#define R_MIN -45

#define Y_MAX -90
#define Y_MIN 90

#define ROLL 0
#define PITCH 1
#define YAW 2




void setAnglesRel( float yaw, float pitch, float heading );
void setAnglesAbs( float yaw, float pitch);
void processGimbal( void );
void centerGimbal( void );
void initGimbal( float heading );
void powerOffGimbal(void);
void powerOnGimbal(void);
void requestAngles( void );
float getPitch( void );
float getYaw( void );
void zeroGimbal( float heading);

float getDiverX(void);
float getDiverY(void);
float getDiverZ(void);
boolean getValidDiver(void); 

void getRelativePose(float heading, float depth);
float getHeadingToDiver(void);
boolean updateVision(void);


