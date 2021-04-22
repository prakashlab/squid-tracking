
/*Code to track a moving object using on-the-fly image analysis
  and Stepper motor control. See also the code to do the same using DC motor control (PID_ImageTracking_v3)
  Code accepts (X_com,Y_com) values from the Serial port of PC/Raspberry-pi
  and tracks this position.
  Code has two modes: OPEN LOOP and CLOSED LOOP
  Uses the Arduino PID library in Closed Loop Mode.
  Nomenclature: CW is +ve, CCW is -ve
  -by Deepak Krishnamurthy
  last mod: 2018-02-03
********************************************************/
/* Axis convention
********************************************************
X-axis - Along the plane of the wheel (+ve X axis Points Radially Outward)
Y axis: Optical axis; along rotational axis of the wheel (Orientation fixed by the Right-Hand Rule using the other two axes)
+ve Z- axis : anti-parallel to gravity vector 
********************************************************/
//=================================================================================
// HEADER FILES
//=================================================================================
#include <AccelStepper.h>
#include <stdlib.h>
//=================================================================================
// Mathematical constants
//=================================================================================
const double pi = 3.1415926535897;
//=================================================================================
// PIN Definitions
//=================================================================================
// Direction and Step outputs
//--------------------------------------------------
#define STEP_THETA 35
#define DIR_THETA 33
//--------------------------------------------------
//Theta axis microstepping pins
//--------------------------------------------------
//--------------------------------------------------
// Z- axis (Wheel) : anti-parallel to gravity vector 
//--------------------------------------------------
#define MS1_THETA 41
#define MS2_THETA 39
#define MS3_THETA 37

//-------------------------------------------------------------------------------
  // Set the initial microstepping
  //-------------------------------------------------------------------------------
bool ms1_X,ms2_X,ms3_X,ms1_Y,ms2_Y,ms3_Y,ms1_Z,ms2_Z,ms3_Z, ms1_Theta, ms2_Theta, ms3_Theta;          // State of microstepping pins

int  microSteps_X = 4, microSteps_Y = 4, microSteps_Theta = 1, microSteps_Z = 4;
int StepsPrev = 5000;

//============================================================================================================
// Create required number of instances of the AccelStepper Class based on the no:of Stepper motors to control
//============================================================================================================
    
// Theta axis stepper
AccelStepper stepperTHETA(AccelStepper::DRIVER, STEP_THETA, DIR_THETA);     

void setup()
{
  int maxManualSpeedTheta = 5000;
  //-------------------------------------------------------------------------------
  // Set up the DIRECTION pins as OUTPUT
  //-------------------------------------------------------------------------------

  pinMode(DIR_THETA, OUTPUT); digitalWrite(DIR_THETA, LOW);
  //-------------------------------------------------------------------------------
  // Set up the STEP pins as OUTPUT
  //-------------------------------------------------------------------------------

  pinMode(STEP_THETA, OUTPUT); digitalWrite(STEP_THETA, LOW);
  
  pinMode(MS1_THETA, OUTPUT); pinMode(MS2_THETA, OUTPUT); pinMode(MS3_THETA, OUTPUT);  
  
  
 
  //-------------------------------------------------------------------------------
  // actualise local variable ms1_X (resp. Y & Z) ms2.. ms3.. and write in the pin MS1.. MS2.. MS3..
  //-------------------------------------------------------------------------------
 
  setMS123(microSteps_Theta,'T');

  //-------------------------------------------------------------------------------
  // Set the initial speed and the MAX acceleration of all the steppers (Speeds > 1000 full-steps per second are unreliable)
  //-------------------------------------------------------------------------------
  int  stepperSpeedTheta = microstep_to_manualSpeed(microSteps_Theta, maxManualSpeedTheta); 
 

  int maxAcclTheta=5000;

 

  stepperTHETA.setMaxSpeed(stepperSpeedTheta);
  stepperTHETA.setAcceleration(maxAcclTheta);

  
  
  
}

void loop()
{
 if(stepperTHETA.distanceToGo()==0)
    {   
        StepsPrev = -(StepsPrev + 1000);
        stepperTHETA.move(StepsPrev);
    }

    stepperTHETA.run();
   
}

int microstep_to_manualSpeed(int microSteps, int maxManualSpeed)
{
  if (microSteps == 1)
  {
    return maxManualSpeed;          //The speed is maxManualSpeed
  }
  else if (microSteps == 2) 
  {
    return int(1.2*maxManualSpeed); //The speed has been multiplied by 1.2/2
  }
  else if (microSteps == 4)        
  {
    return int(2*maxManualSpeed);   //The speed has been multiplied by 2/4
  }
  else if (microSteps == 8)
  {
    return int(3*maxManualSpeed);   //The speed has been multiplied by 3/8
  }
  else if (microSteps == 16)
  {
    return int(5*maxManualSpeed);   //The speed has been multiplied by 5/16
  }
}

//-------------------------------------------------------------------------------
// Returns the number of fractional steps per 1 full step of the motor shaft
//-------------------------------------------------------------------------------

int MS123_to_microstepp(bool MS1, bool MS2, bool MS3)   //microSteppingLUT
{

  if(MS1==LOW && MS2==LOW && MS3 == LOW)
  {
    return 1;
  }
  else if(MS1==HIGH && MS2==LOW && MS3 == LOW)
  {
    return 2;
  }
  else if(MS1==LOW && MS2==HIGH && MS3 == LOW)
  {
    return 4;
  }
  else if(MS1==HIGH && MS2==HIGH && MS3 == LOW)
  {
    return 8;
  }
  else if(MS1==HIGH && MS2==HIGH && MS3 == HIGH)
  {
    return 16;
  }
}

//---------------------------------------------------------------------------------------------------------
// Update the pin and the msi variable for a given microstepping and a given motor
//---------------------------------------------------------------------------------------------------------
void setMS123(int microstep,char motor){
  bool MS1,MS2,MS3;
  if (microstep==1){
    MS1 = LOW; MS2 = LOW; MS3 = LOW;
  }
  else if (microstep==2){
    MS1 = HIGH; MS2 = LOW; MS3 = LOW;  
  }
  else if (microstep==4){
    MS1 = LOW; MS2 = HIGH; MS3 = LOW; 
  }
  else if (microstep==8){
    MS1 = HIGH; MS2 = HIGH; MS3 = LOW; 
  }
  else if (microstep==16){
    MS1 = HIGH; MS2 = HIGH; MS3 = HIGH;
  }

 
  
  microSteps_Theta=microstep;
  ms1_Theta = MS1; ms2_Theta = MS2; ms3_Theta = MS3;
  digitalWrite(MS1_THETA, ms1_Theta);digitalWrite(MS2_THETA, ms2_Theta);digitalWrite(MS3_THETA, ms3_Theta);
    
  

}
