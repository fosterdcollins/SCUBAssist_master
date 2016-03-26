#define CONTROLTIME 0.05 //Seconds


float HeadingController(float HeadingDesired, float HeadingRateDesired, float heading, float HeadingRate); 
float DepthController(float DepthDesired, float Depth, float DepthRateDeisred, float currTime);
void resetDepthI( void );
void changeGains( float Gains[] );

