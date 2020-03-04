/********************************************************
  uController code for gravity machine
  Axes convention:
  X (Linear stage): +ve radially outwards at 3'O Clock Position 
  Y (Linear stage): +ve Away from the camera
  Z (Linear stage): +ve upwards from gravity
  Theta (Rotational stage): CW +ve
  -by Deepak Krishnamurthy & François BENOIT du REY
********************************************************/
//=================================================================================
// HEADER FILES
//=================================================================================
#include <stdlib.h>


///--------------------------------------------------
// Serial communication
//--------------------------------------------------
# define CMD_LENGTH 11
# define DATA_TX_LENGTH 11

//=================================================================================
// Serial Comm Variables
//=================================================================================
bool isReceived=true;
volatile bool sendData = false;
//=================================================================================
// Triggering parameters (Tracking camera)
//=================================================================================
int fps = 120;
int numTimerCycles = 1000000/fps/timerPeriod;
volatile int counter_timer = -1;

volatile unsigned long prevMillisSend = 0, currMillisSend = 0, prevMillisRec = 0, currMillisRec = 0, currMillisStage=0, prevMillisStage = 0, MillisStick = 0, currMillis = 0, currMillisLight = 0;
volatile unsigned long timestamp_lastTrigger = 0;
bool startTriggering = false;

//=================================================================================
// Serial Communication
//=================================================================================
byte buffer_tx[DATA_TX_LENGTH];
byte buffer_rx[500];
int buffer_rx_ptr;
bool calculateCRC_tx = true;
//=================================================================================
// Send and Receive variables
//=================================================================================

float liquidLensSweepFrequency = 0;
int liquidLensOrder = 0, Homing = 0, Tracking = 0;
float Step_X = 0, Step_Y = 0, Step_Theta = 0;




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                          Beginning of SETUP
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void setup()
{
  
  //===============================================================================
  // Setup Serial Communication 
  //===============================================================================
  SerialUSB.begin (2000000);
  
  while(!SerialUSB); //Wait until connection is established
  buffer_rx_ptr=0;
  int a = 1;
  //SerialUSB.println(1,DEC); 
  SerialUSB.write(a);
  int init=0;
  
  while (init!=2)        // Wait for command 'a' to be sent from the host computer.
  {
    init=int(SerialUSB.read());
  }
  SerialUSB.write(init);


  delay(1000);

  // initialize timer
  Timer3.attachInterrupt(liquid_lens_handler_timer_500us);
  Timer3.start(timerPeriod); // Calls every 500 us
  startTriggering = true;

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                          Beginning of the main loop
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void loop()
{
  


  

  
 

  
  //-------------------------------------------------------------------------------
  // Serial sending block (Send data to computer)
  //-------------------------------------------------------------------------------
  // uController only sends data when image is triggered.
 
  if (sendData){
    
    buffer_tx[0] = byte(phase_code_lastTrigger%256);
    buffer_tx[1] = byte(phase_code_lastTrigger>>8);
    
    // When only open-loop position is available
//    CurrPos_X_code = CurrPos_X;  //right sens of the motor
    // When a closed-loop encoder is implemented
    // Testing
    x_EncoderTicks = 1234;
    CurrPos_X_code = x_EncoderTicks;  //right sens of the motor
    if (CurrPos_X_code>0){
      buffer_tx[2] = byte(int(0));
    }
    else{
      buffer_tx[2] = byte(int(1));
      CurrPos_X_code = -CurrPos_X_code;
    }
    buffer_tx[3] = byte(CurrPos_X_code>>24);
    buffer_tx[4] = byte(CurrPos_X_code>>16);
    buffer_tx[5] = byte(CurrPos_X_code>>8);
    buffer_tx[6] = byte(CurrPos_X_code%256);

    // CurrPos_Y_code= CurrPos_Y;  //right sens of the motor
    CurrPos_Y_code = y_EncoderTicks;

    if (CurrPos_Y_code>0){
      buffer_tx[7] = byte(int(0));
    }
    else{
      buffer_tx[7] = byte(int(1));
      CurrPos_Y_code = -CurrPos_Y_code;
    }
    buffer_tx[8]  = byte(CurrPos_Y_code>>24);
    buffer_tx[9]  = byte(CurrPos_Y_code>>16);
    buffer_tx[10] = byte(CurrPos_Y_code>>8);
    buffer_tx[11] = byte(CurrPos_Y_code%256);
    
    
    CurrPos_Theta_code=theta_EncoderTicks;
    if(CurrPos_Theta_code>0) 
    {
      buffer_tx[12] = byte(int(0));
      
    }
    else 
    {
      buffer_tx[12] = byte(int(1));
      CurrPos_Theta_code = -CurrPos_Theta_code;
    }
    buffer_tx[13] = byte(CurrPos_Theta_code>>24);
    buffer_tx[14] = byte(CurrPos_Theta_code>>16);
    buffer_tx[15] = byte(CurrPos_Theta_code>>8);
    buffer_tx[16] = byte(CurrPos_Theta_code%256);

    buffer_tx[17] = byte(ManualMode);

    // tracking trigger
    buffer_tx[18] = byte(!digitalRead(triggerTrack));
  
    
//    // 16 bit crc
//    calculateCRC_tx = false;
//    if(calculateCRC_tx) {
//      uint16_t crc = crc_ccitt(buffer_tx,24);
//      // uint16_t crc = crc_xmodem(buffer_tx,24);
//      buffer_tx[25] = byte(crc>>8);
//      buffer_tx[26] = byte(crc%256);
//    }
//    else{
//      buffer_tx[0] = 0;
//      buffer_tx[0] = 0;
//    }    

    SerialUSB.write(buffer_tx,DATA_TX_LENGTH);
   

    sendData=false;
  }
  //-------------------------------------------------------------------------------
  // Serial Receiving Block 
  //-------------------------------------------------------------------------------

  //Data reception: the data is read at the frequency of the computer
   
   if(SerialUSB.available())
   {
      buffer_rx_ptr=0;
      int cyclesElapsed = 0;
      while(buffer_rx_ptr < CMD_LENGTH ){ 
        buffer_rx[buffer_rx_ptr] = SerialUSB.read();
        buffer_rx_ptr++;
        // timeout:
        if(cyclesElapsed++>=10000)
          break;
      }

      //Data analysis: if the right message is read, lets compute the data and update arduino position
      isReceived=true;
      if (buffer_rx_ptr == CMD_LENGTH) {
 
        isReceived=true;

        // Holding freq fized (TESTING)
        liquidLensSweepFrequency = float(int(buffer_rx[0]) + int(buffer_rx[1]) *256)/100;
//        liquidLensVAmplitude = float(int(buffer_rx[2]) + int(buffer_rx[3]) *256)/1000*20/1.4;        //the amplitude is sent in mm, and is converted in V: 10V = 1.4 mm
//        liquidLensVOffset = float(int(buffer_rx[4]) + int(buffer_rx[5]) *256)/100;
        
        liquidLensOrder=int(buffer_rx[2]);
        
        if (liquidLensOrder==1 ){
          isYtracking=true;
        }
        else{
          isYtracking=false;
        }
        
        
        
        Homing = int(buffer_rx[3]);                                                                  
        Tracking=int(buffer_rx[4]);
        
        Step_X = int(buffer_rx[5])+(int(buffer_rx[6]) << 8);                                   // X steps to move  ATTENTION: We change the sens of the motor 
        if (Step_X>32767){
          Step_X=Step_X-65536;
        }
        Step_X = Step_X;                                                                   
         
        Step_Y = int(buffer_rx[7])+(int(buffer_rx[8]) << 8);
        if (Step_Y>32767){
          Step_Y=Step_Y-65536;
        }
        Step_Y = -Step_Y;
        

        Step_Theta = int(buffer_rx[9])+(int(buffer_rx[10]) << 8); 
        if (Step_Theta > 32767){
          Step_Theta = Step_Theta - 65536;
        }
        Step_Theta= Step_Theta;
        
      
      }
    }

   

  

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                             End of the main loop
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



// https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
/*
uint16_t crc_ccitt_update (uint16_t crc, uint8_t data)
{
  data ^= lo8 (crc);
  data ^= data << 4;
  return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}
*/

uint16_t crc_ccitt (byte* data_array, int data_length)
{
  uint16_t crc = 0xffff;
  for (int i=0; i <= data_length; i++){
    uint8_t data = data_array[i];
    data ^= (crc%256); // data ^= (crc&0xff);
    data ^= data << 4;
    crc = (((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3);
  }
  return crc;
}

/*
uint16_t crc_xmodem (byte* data_array, uint8_t data_length)
{
  uint16_t crc = 0x0;
  for (int i=0; i <= data_length; i++){
    data = data_array[i];
    int j;
    crc = crc ^ ((uint16_t)data << 8);
    for (j=0; j<8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}
*/

// Interrupt 

    
//-------------------------------------------------------------------------------
//                                 Timer camera trigger
//-------------------------------------------------------------------------------

void liquid_lens_handler_timer_500us(){
  
  // update phase (add *0.1 to debug )
  phase += 2*pi*(liquidLensSweepFrequency*timerPeriod/1000000);
  if (phase > 2*pi) {
    phase -= 2*pi;
  }
  // update phase_code
  phase_code = (int) round(65535*(phase)/(2*pi));


  /*
  liquidLensV = liquidLensVOffset + liquidLensVAmplitude*sin(phase);
  liquidLensVCode = liquidLensV*22.5828 - 551.0199;
  
  // translate phase into voltage and send it to the liquid lens driver
  if (false){
    Wire.beginTransmission(0x77); // transmit to device (0xEE)
    // Wire.write(byte(0xEE));
    Wire.write(byte(0x03));   // UserMode reg address
    Wire.write(byte(0x03));   // UserMode: ACTIVE = 1, SM = 1
    Wire.write(byte(liquidLensVCode<<6));   // OIS_LSB reg value 
    Wire.endTransmission();     // stop transmitting
  }
  if (false){
    Wire.beginTransmission(0x77); // transmit to device (0xEE)
    Wire.write(byte(0x08));   // LLV4 reg address
    Wire.write(byte(liquidLensVCode>>2));   // LLV4
    Wire.write(byte(0x02));   // Command
    Wire.endTransmission();     // stop transmitting
  }
  */
  
  if(startTriggering)
  {
    counter_timer++;
    if (counter_timer==numTimerCycles) 
    {
      counter_timer = 0;
      digitalWrite(triggerCamera,HIGH);
      phase_code_lastTrigger = phase_code;
      //timestamp_lastTrigger = millis();
      timestamp_lastTrigger = timestamp_lastTrigger+1; // repurposed as trigger counter
      sendData = true;
    }
    if (counter_timer==1) 
    {
      digitalWrite(triggerCamera,LOW);

    }
  }
  if(startTriggering_FL)
  {
    counter_timer_FL++;
    if (counter_timer_FL == numTimerCycles_FL) 
    {
      counter_timer_FL = 0;
      // Trigger FL camera
      digitalWrite(triggerCamera_FL,HIGH);
      digitalWrite(triggerLED,HIGH);
    }
    // We want this trigger signal to be longer duration.
    if (counter_timer_FL==500) 
    {
      digitalWrite(triggerCamera_FL,LOW);
      digitalWrite(triggerLED,LOW);
    }

      
  }
}

//-------------------------------------------------------------------------------
//                                 Hardware interrupt: set phase to 0
//-------------------------------------------------------------------------------

void HandleOptotuneSYNCInterrupt() {
  phase = pi/2.0;
}
