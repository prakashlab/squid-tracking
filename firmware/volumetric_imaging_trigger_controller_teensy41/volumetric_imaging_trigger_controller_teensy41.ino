IntervalTimer Timer_Simulated_Liquid_Lens_Trigger;

static const bool CONNECT_LIQUID_LENS_TRIGGER_IN_SOFTWARE = false;

static const int pin_liquid_lens_trigger_simulation = 0;
static const int pin_liquid_lens_trigger_in = 1;
static const int pin_camera_trigger = 2;
static const int pin_led = 13;

volatile bool liquid_lens_trigger_simulation_level = LOW;

// MCU - computer communication
static const int CMD_LENGTH = 4;
byte buffer_rx[500];
volatile int buffer_rx_ptr;

// MCU - computer communication command set
static const int SET_NUMBER_OF_PLANES_PER_VOLUME = 10;
static const int SET_NUMBER_OF_REQUESTED_VOLUMES = 11;
static const int SET_FREQUENCY_HZ = 12;
static const int SET_PHASE_DELAY = 13;
static const int START_TRIGGER_GENERATION = 14;
static const int STOP_TRIGGER_GENERATION = 15;

/*
// remote focus related variables
bool remote_focus_trigger_enabled = false; // set to true to listen to optotune trigger output, only set by the computer
uint16_t remote_focus_number_of_planes_per_volume; // make sure trigger interval x planes per volume < 1/liquid lens scanning frequency
uint32_t remote_focus_number_of_volumes_requested; // 0 means infinite 
uint32_t remote_focus_trigger_interval_us;
float remote_focus_liquid_lens_frequency;
float remote_focus_liquid_lens_phase_delay; // 0 to 1 for 0 to 90 degree
float remote_focus_delay_us_due_to_phase_lag;
*/
// remote focus related variables
bool remote_focus_trigger_enabled = false; // set to true to listen to optotune trigger output, only set by the computer
uint16_t remote_focus_number_of_planes_per_volume = 5; // make sure trigger interval x planes per volume < 1/liquid lens scanning frequency
uint32_t remote_focus_number_of_volumes_requested = 0; // 0 means infinite 
uint32_t remote_focus_trigger_interval_us = 5*40000;
float remote_focus_liquid_lens_frequency = 1;
float remote_focus_liquid_lens_phase_delay = 0; // 0 to 1 for 0 to 90 degree
float remote_focus_delay_us_due_to_phase_lag = 0;

long remote_focus_current_volume_number = 0;   // increament at the beginning of the volume
int remote_focus_current_plane_number = 0;

unsigned long remote_focus_timestamp_last_trigger_rising_edge;
unsigned long remote_focus_timestamp_last_volume_start_rising_edge;

volatile long remote_focus_timestamp_liquid_lens_trigger_rising_edge;
volatile bool remote_focus_liquid_lens_trigger_received = false;
bool remote_focus_volume_started = false;
bool remote_focus_camera_trigger_pin_state = false;

unsigned long current_time_us;

// trigger pulse length (active high)
static const int remote_focus_camera_trigger_pulse_length_us = 50;

bool tmp_debug = LOW;

void setup() 
{

  SerialUSB.begin(20000000);
  
  pinMode(pin_liquid_lens_trigger_simulation,OUTPUT);
  pinMode(pin_liquid_lens_trigger_in,INPUT);
  pinMode(pin_camera_trigger,OUTPUT);
  pinMode(pin_led,OUTPUT);
  
  Timer_Simulated_Liquid_Lens_Trigger.begin(ISR_Timer_Simulated_Liquid_Lens_Trigger,1000000);
  
  NVIC_SET_PRIORITY(IRQ_GPIO6789,9); // https://forum.pjrc.com/threads/62432-Teensy-4-1-counter-example-not-working
  attachInterrupt(digitalPinToInterrupt(pin_liquid_lens_trigger_in), ISR_liquid_lens_trigger_detected, RISING);
  
  delayMicroseconds(1500000);
  while (SerialUSB.available()) 
    SerialUSB.read();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  while (SerialUSB.available()) 
  { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) 
    {
        buffer_rx_ptr = 0;
        if(buffer_rx[0] == SET_FREQUENCY_HZ)
        {
          remote_focus_liquid_lens_frequency = float(uint16_t(buffer_rx[1])*256 + uint16_t(buffer_rx[2]))/1000;
        }
        else if(buffer_rx[0] == SET_PHASE_DELAY)
        {
          remote_focus_liquid_lens_phase_delay = float(int(buffer_rx[1])*256 + int(buffer_rx[2]))/65535;
          remote_focus_delay_us_due_to_phase_lag = ((1000000*1/float(remote_focus_liquid_lens_frequency))/4)*remote_focus_liquid_lens_phase_delay;
        }
        else if(buffer_rx[0] == SET_NUMBER_OF_PLANES_PER_VOLUME)
        {
          remote_focus_number_of_planes_per_volume = uint16_t(buffer_rx[1])*256 + uint16_t(buffer_rx[2]);
        }
        else if(buffer_rx[0] == SET_NUMBER_OF_REQUESTED_VOLUMES)
        {
          remote_focus_number_of_volumes_requested = uint32_t(buffer_rx[1])*65536 + uint32_t(buffer_rx[2])*256 + uint32_t(buffer_rx[3]);
        }
        else if(buffer_rx[0] == START_TRIGGER_GENERATION)
        {
          // tmp_debug = not tmp_debug; //@@@
          // digitalWrite(pin_led,HIGH); //@@@
          remote_focus_trigger_enabled = true;
          remote_focus_current_plane_number = 0;
          remote_focus_current_volume_number = 0;
          remote_focus_trigger_interval_us = uint32_t( (1000000*1/float(remote_focus_liquid_lens_frequency))/remote_focus_number_of_planes_per_volume );
          Timer_Simulated_Liquid_Lens_Trigger.update(remote_focus_trigger_interval_us);
        } 
        else if(buffer_rx[0] == STOP_TRIGGER_GENERATION)
        {
          // digitalWrite(pin_led,LOW); //@@@
          remote_focus_trigger_enabled = false;
        }
    }
  }

    // volumetric imaging
  current_time_us = micros();
  if(remote_focus_liquid_lens_trigger_received)
  {
    // send the camera trigger for plane 0
    if(remote_focus_volume_started == false && current_time_us-remote_focus_timestamp_liquid_lens_trigger_rising_edge>=remote_focus_delay_us_due_to_phase_lag)
    {
      digitalWrite(pin_camera_trigger,HIGH);
      digitalWrite(pin_led,HIGH);
      remote_focus_camera_trigger_pin_state = HIGH;
      remote_focus_volume_started = true;
      current_time_us = micros();
      remote_focus_timestamp_last_trigger_rising_edge = current_time_us;
      remote_focus_timestamp_last_volume_start_rising_edge = current_time_us;
      remote_focus_current_plane_number = 1;
      // check if requested number of planes have been started
      if(remote_focus_current_plane_number >= remote_focus_number_of_planes_per_volume)
      {
        remote_focus_liquid_lens_trigger_received = false; // to stop trigger for the current volume
        remote_focus_volume_started = false; // get ready for the next volume
      }
      remote_focus_current_volume_number = remote_focus_current_volume_number + 1;
    }

    // check if requested number of volumes have been started
    if(remote_focus_number_of_volumes_requested > 0 && remote_focus_current_volume_number >= remote_focus_number_of_volumes_requested)
      remote_focus_trigger_enabled = false; // this will prevent remote_focus_liquid_lens_trigger_received from being set to true - stop the acquisitiion

    // send remaining triggers for the current volume - if any
    if(remote_focus_volume_started && current_time_us - remote_focus_timestamp_last_trigger_rising_edge >= remote_focus_trigger_interval_us && remote_focus_camera_trigger_pin_state == LOW)
    {
      digitalWrite(pin_camera_trigger,HIGH);
      digitalWrite(pin_led,HIGH);
      remote_focus_camera_trigger_pin_state = HIGH;
      current_time_us = micros();
      remote_focus_timestamp_last_trigger_rising_edge = current_time_us;
      remote_focus_current_plane_number = remote_focus_current_plane_number + 1;
      if(remote_focus_current_plane_number >= remote_focus_number_of_planes_per_volume)
      {
        remote_focus_liquid_lens_trigger_received = false; // to stop trigger for the current volume
        remote_focus_volume_started = false; // get ready for the next volume
      }
    }
  }
  
  // turn the trigger pin to LOW
  if(remote_focus_camera_trigger_pin_state == HIGH && micros() - remote_focus_timestamp_last_trigger_rising_edge >= remote_focus_camera_trigger_pulse_length_us)
  {
    digitalWrite(pin_camera_trigger,LOW);
    digitalWrite(pin_led,LOW);
    remote_focus_camera_trigger_pin_state = LOW;
  }
  
}

void ISR_liquid_lens_trigger_detected()
{
  remote_focus_timestamp_liquid_lens_trigger_rising_edge = micros();
  if(remote_focus_trigger_enabled)
    remote_focus_liquid_lens_trigger_received = true;
}

void ISR_Timer_Simulated_Liquid_Lens_Trigger()
{
  liquid_lens_trigger_simulation_level = !liquid_lens_trigger_simulation_level;
  digitalWrite(pin_liquid_lens_trigger_simulation,liquid_lens_trigger_simulation_level);
  if(CONNECT_LIQUID_LENS_TRIGGER_IN_SOFTWARE)
  {
    if(remote_focus_trigger_enabled)
    {
      remote_focus_liquid_lens_trigger_received = true;
      remote_focus_timestamp_liquid_lens_trigger_rising_edge = micros();
    }
  }
}
