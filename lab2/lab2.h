// Rice ELEC424 Lab2.h
// Simulated Quadrotor tasks
// Authors: Steven Arroyo and Lin Zhong

// Speed struct for motors
typedef struct
{
    unsigned char m1;
    unsigned char m2;
    unsigned char m3;
    unsigned char m4;
}MotorSpeeds;  

// this function needs to be executed once every 10 ms.  Priority 1.
unsigned char detectEmergency(void); 

// this function needs to be executed once every 100 ms.  Priority 2.
unsigned char refreshSensorData(void); 

// this function needs to be executed once every second.    Priority 3. 
// This function takes many cycles to complete!
// Care should be taken not to block other higher priority functions
unsigned char calculateOrientation(void); 

// this function needs to be executed once every second.    Priority 4.
// this function will return a struct with speeds for each motor.  Since the PWM's can
// be configured in different ways the speed will be either 0 or 1 (off or on).  You must scale this 
// value appropriately to a safe motor speed that you determined in the first part of the lab.
unsigned char updatePid(MotorSpeeds* p_motorSpeedsPtr);

// Run this low priority function when time is available.    Priority 5.
unsigned char logDebugInfo(void);

