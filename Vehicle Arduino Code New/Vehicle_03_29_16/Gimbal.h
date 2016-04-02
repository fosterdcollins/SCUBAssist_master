#define P_MAX 90
#define P_MIN -86

#define R_MAX 45
#define R_MIN -45

#define Y_MAX -90
#define Y_MIN 90

void Set_rpy(float roll, float pitch, float yaw);
void centerGimbal(void);

float getDiverX(void);
float getDiverY(void);
float getDiverZ(void);
boolean getValidDiver(void); 

void getRelativePose(float heading, float depth);
float getHeadingToDiver(void);
boolean updateVision(void);
