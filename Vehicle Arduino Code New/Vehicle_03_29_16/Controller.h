#define CONTROLTIME 0.05 //Seconds


float HeadingController(float HeadingDesired, float HeadingRateDesired, float heading, float HeadingRate, int mode); 
float DepthController(float DepthDesired, float Depth, float DepthRateDeisred, float currTime);
void PositionController(float XDesired, float XPos, float YDesired, float YPos, float yaw);
void PositionController2(float RDesired, float HeadingDesired, float XPos, float YPos, float yaw);

float getVXAuton( void );
float getVYAuton( void );

void resetDepthI( void );
void changeGains( float Gains[] );



