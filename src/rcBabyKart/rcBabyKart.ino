

// Arduino based remote control GoKart
// Allows for control using a standard RC TX/RX
// Arduino interprets the output of the receiver and controls
// larger motors using an external h-bridge
// Steering is accomplished using an external potentiometer 
// attached to the steering motor to feedback the current
// wheel direction

#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>
#include <EEPROMex.h>

#define CONFIG_VERSION 1

#define memoryBase 32
int configAdress=0;

// Example settings structure
struct rcCalibration {
    uint8_t version;   // This is for mere detection if they are your settings
    uint8_t calibrated;          // The variables of your settings
    uint16_t throttle_min;
    uint16_t throttle_max;
    uint16_t steering_min;
    uint16_t steering_max;
    uint16_t lights_min;
    uint16_t lights_max;
    uint16_t steering_feedback_min;
    uint16_t steering_feedback_max;
} calibration = { 
    CONFIG_VERSION,
    0,
    3000,
    0,
    3000,
    0,
    3000,
    0,
    1023,
    0
};

bool loadConfig() {
  EEPROM.readBlock(configAdress, calibration);
  return (calibration.version == CONFIG_VERSION);
}

void saveConfig() {
   EEPROM.writeBlock(configAdress, calibration);
}



//calibrate for 20 seconds
#define CALIBRATION_TIME      20000

//OUTPUTS (To H-Bridge)
#define STEERING_DIR_PIN       2
#define STEERING_PWM_PIN       3
#define THROTTLE_DIR_PIN       4
#define THROTTLE_PWM_PIN       5
#define LIGHTING_PIN           10

//INPUTS:
//Input from RC Receiver
#define THROTTLE_RC_IN_PIN     6
#define STEERING_RC_IN_PIN     7
#define LIGHTING_RC_IN_PIN     8


//Switches
#define TURBO_IN_PIN           11
#define LOCAL_CONTROL_PIN      12
#define LIGHTING_IN_PIN        12
#define ESTOP_IN_PIN           13

//Feedback for steering Servo (steering column position)
#define STEERING_FEEDBACK_PIN A0

//External Inputs used when manual override enabled
//For Drive-By-Wire Steering
#define STEERING_WHEEL_PIN    A2
#define GAS_PEDAL_PIN         A3

#define DEBUG 1

//deadband for steering, currently set to:
// 20*(5V/1024) = ~100mV
#define STEERING_DEADBAND 20

// Assign servo indexes
#define SERVO_THROTTLE        0
#define SERVO_STEERING        1
#define SERVO_AUX             2
#define SERVO_FRAME_SPACE     3

#define THROTTLE_FLAG         1
#define STEERING_FLAG         2
#define LIGHTING_FLAG         4


//The ESTOP Pin  must be held low by safety circuit
//if the circuit is opened the signal is pulled up
#define ESTOP (digitalRead(ESTOP_IN_PIN) == HIGH)
#define TURBO_PUSHED (digitalRead(TURBO_IN_PIN) == LOW)

//If manual override pin is pulled low it is active
#define LOCAL_CONTROL (digitalRead(LOCAL_CONTROL_PIN) == LOW)

volatile uint8_t bUpdateFlagsShared = 0;
volatile uint16_t throttleInPulseLengthShared = 0;
volatile uint16_t steeringInPulseLengthShared= 0;
volatile uint16_t lightsInPulseLengthShared = 0;
uint16_t throttleInPulseLengthStart = 0;
uint16_t steeringInPulseLengthStart = 0;
uint16_t lightsInPulseLengthStart = 0;


uint8_t turbo_active = 0;
uint32_t do_calibration = 0;
uint8_t config_ok = 0;

void calcThrottle();
void calcSteering();
void calcLighting();


void setup()
{
  Serial.begin(115200);
  Serial.println("RCBabyKart 1.0");
 Serial.println("a");
  EEPROM.setMemPool(memoryBase, EEPROMSizeUno); //Set memorypool base to 32, assume Arduino Uno board
  configAdress  = EEPROM.getAddress(sizeof(rcCalibration)); // Size of config object 
  Serial.println("b");
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 1 Servos + 9 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,7*2000);
  CRCArduinoFastServos::begin();
  PCintPort::attachInterrupt(THROTTLE_RC_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_RC_IN_PIN, calcSteering,CHANGE);
  PCintPort::attachInterrupt(LIGHTING_RC_IN_PIN, calcLighting,CHANGE);


  pinMode(THROTTLE_DIR_PIN, OUTPUT);
  pinMode(THROTTLE_PWM_PIN, OUTPUT);
  analogWrite(THROTTLE_PWM_PIN, 0);

  pinMode(LIGHTING_PIN, OUTPUT);
  digitalWrite(LIGHTING_PIN, LOW);
  pinMode(STEERING_DIR_PIN, OUTPUT);
  pinMode(STEERING_PWM_PIN, OUTPUT);
  analogWrite(STEERING_PWM_PIN, 255);

  //Emergeny Stop input
  //Pulled up internally; must be kept low by safety circuit
  pinMode(ESTOP_IN_PIN, INPUT);
  digitalWrite(ESTOP_IN_PIN, HIGH);
  
  //TO DO:
  //Temporary remove pull up for testing
  digitalWrite(ESTOP_IN_PIN, LOW);

  //Manual override pin active low
  pinMode(LOCAL_CONTROL_PIN, INPUT);
  digitalWrite(LOCAL_CONTROL_PIN, HIGH);

  pinMode(TURBO_IN_PIN, INPUT);
  digitalWrite(TURBO_IN_PIN, HIGH);

  do_calibration = 0;
  if (TURBO_PUSHED)
  {
    saveConfig();
    do_calibration = CALIBRATION_TIME;
  }
  loadConfig();


}

void printCalibration()
{
  //Log calibration Info every 500ms
  Serial.println(""); 
  Serial.print("CALIBRATION: Calibrated: "); 
  Serial.print(calibration.calibrated, DEC);
  Serial.println(""); 
  Serial.print("CALIBRATION: Throttle Min: "); 
  Serial.print(calibration.throttle_min, DEC); 
  Serial.print("\tMax:"); 
  Serial.print(calibration.throttle_max, DEC); 
  Serial.println(""); 
  Serial.print("CALIBRATION: Steering Min: "); 
  Serial.print(calibration.steering_min, DEC); 
  Serial.print("\tMax:"); 
  Serial.print(calibration.steering_max, DEC); 
  Serial.println(""); 
  Serial.print("CALIBRATION: Lights Min: "); 
  Serial.print(calibration.lights_min, DEC); 
  Serial.print("\tMax:"); 
  Serial.print(calibration.lights_max, DEC); 
  Serial.println(""); 
  Serial.print("CALIBRATION: Steering Fbk Min: "); 
  Serial.print(calibration.steering_feedback_min, DEC); 
  Serial.print("\tMax:"); 
  Serial.print(calibration.steering_feedback_max, DEC); 
  Serial.println(""); 
}

int lighting_state = 0;
int turbo_state = 0;

void loop()
{
  static int throttle_set;
  static int steering_set;
  static int steering_feedback;
  static int steering_delta;
  static uint16_t throttleInPulseLength = 0;
  static uint16_t steeringInPulseLength = 0;
  static uint16_t lightsInPulseLength = 0;
  static uint16_t safetyInPulseLength = 0;
  static uint8_t bUpdateFlags;



// check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    bUpdateFlags = bUpdateFlagsShared;

    if((bUpdateFlags & THROTTLE_FLAG) && (throttleInPulseLengthShared != 65536))
    {
      throttleInPulseLength = throttleInPulseLengthShared;
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      steeringInPulseLength = steeringInPulseLengthShared;
    }
  
    if(bUpdateFlags & LIGHTING_FLAG)
    {
      lightsInPulseLength = lightsInPulseLengthShared;
    }
   
    bUpdateFlagsShared = 0;
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }

  if (do_calibration)
  {
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      if (throttleInPulseLength < calibration.throttle_min)
      {
        calibration.throttle_min = throttleInPulseLength;
      }
      if (throttleInPulseLength > calibration.throttle_max)
      {
        calibration.throttle_max = throttleInPulseLength;
      }
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      if (steeringInPulseLength < calibration.steering_min)
      {
        calibration.steering_min = steeringInPulseLength;
      }
      if (steeringInPulseLength > calibration.steering_max)
      {
        calibration.steering_max = steeringInPulseLength;
      }
    }

    if(bUpdateFlags & LIGHTING_FLAG)
    {
      if (lightsInPulseLength < calibration.lights_min)
      {
        calibration.lights_min = lightsInPulseLength;
      }
      if (lightsInPulseLength > calibration.lights_max)
      {
        calibration.lights_max = lightsInPulseLength;
      }
    }

    steering_feedback = analogRead(STEERING_FEEDBACK_PIN);
    if (steering_feedback < calibration.steering_feedback_min)
    {
      calibration.steering_feedback_min = steering_feedback;
    }
    if (steering_feedback > calibration.steering_feedback_max)
    {
      calibration.steering_feedback_max = steering_feedback;
    }


    if ((do_calibration % 1000) == 0)
    {
      //Log calibration Info every 500ms
      Serial.println(""); 
      Serial.print("CALIBRATION: Throttle Min: "); 
      Serial.print(calibration.throttle_min, DEC); 
      Serial.print("\tMax:"); 
      Serial.print(calibration.throttle_max, DEC); 
      Serial.print("\tCur:"); 
      Serial.print(throttleInPulseLength, DEC); 
      Serial.println(""); 
      Serial.print("CALIBRATION: Steering Min: "); 
      Serial.print(calibration.steering_min, DEC); 
      Serial.print("\tMax:"); 
      Serial.print(calibration.steering_max, DEC); 
      Serial.print("\tCur:"); 
      Serial.print(steeringInPulseLength, DEC); 
      Serial.println(""); 
      Serial.print("CALIBRATION: Lights Min: "); 
      Serial.print(calibration.lights_min, DEC); 
      Serial.print("\tMax:"); 
      Serial.print(calibration.lights_max, DEC); 
      Serial.print("\tCur:"); 
      Serial.print(lightsInPulseLength, DEC);
      Serial.println(""); 
      Serial.print("CALIBRATION: Steering Feedback Min: "); 
      Serial.print(calibration.steering_feedback_min, DEC); 
      Serial.print("\tMax:"); 
      Serial.print(calibration.steering_feedback_max, DEC); 
      Serial.print("\tCur:"); 
      Serial.print(steering_feedback, DEC);
      Serial.println(""); 
    }
 
    do_calibration = do_calibration - 100;
    if (do_calibration)
    {
      delay(100);
    }
    else
    {
      calibration.calibrated = 1;
      saveConfig();
      Serial.println("Config Saved");
    }
  }
  else
  {

    if (ESTOP)
    {
      analogWrite(STEERING_PWM_PIN, 0);
      analogWrite(THROTTLE_PWM_PIN, 0);
    }
    else
    {
      if (LOCAL_CONTROL)
      {
        throttle_set = map(analogRead(GAS_PEDAL_PIN), 0,1023,-255,255);
        steering_set = analogRead(STEERING_WHEEL_PIN);
      }
      else
      {
        //Remote Control
        throttle_set = map(throttleInPulseLength,
                       calibration.throttle_min,
                       calibration.throttle_max,
                       -255,
                       255);

        steering_set = map(steeringInPulseLength,
                       calibration.steering_min,
                       calibration.steering_max,
                       0,
                       1023);

        if (!turbo_active)
        {
          throttle_set /= 2;
        }
      }
      
      
      //SET THROTTLE
      if (throttle_set >= 0)
      {
        digitalWrite(THROTTLE_DIR_PIN, HIGH);
      }
      else
      {
        digitalWrite(THROTTLE_DIR_PIN, LOW);
      }
      analogWrite(THROTTLE_PWM_PIN, abs(throttle_set));
      

      //SET STEERING
      steering_feedback = map(analogRead(STEERING_FEEDBACK_PIN),
                     calibration.steering_feedback_min,
                     calibration.steering_feedback_max,
                     0,
                     1023);

      if (abs(steering_set - steering_feedback) > STEERING_DEADBAND)
      {
        if(DEBUG)
        {
          //Serial.print("steering_set:"); 
          //Serial.println(steering_set);
          //Serial.print("steering_feedback:"); 
          //Serial.println(steering_feedback);
        }
        if (steering_set > steering_feedback)
        {
          digitalWrite(STEERING_DIR_PIN, HIGH);
        }
        else
        {
          digitalWrite(STEERING_DIR_PIN, LOW);
        }
        steering_delta = steering_set - steering_feedback;
        analogWrite(STEERING_PWM_PIN, map(abs(steering_delta), 0, 1023, 50,255));
      }
      else
      {
        //stop the steering motor
        analogWrite(STEERING_PWM_PIN, 0);
      }

      //SET LIGHTING
      if (lightsInPulseLength > ((calibration.lights_min + calibration.lights_max)/2))
      {
        lighting_state = 1;
      }
      else
      {
        lighting_state = 0;
      }
      digitalWrite(LIGHTING_PIN, lighting_state);

    }

    
    if (DEBUG)
    {
      printCalibration();
      Serial.print("OUTPUT: lighting_state:");
      Serial.print(lighting_state, DEC);
      Serial.println("");  
      Serial.print("OUTPUT: throttle_set(-255-255):");
      Serial.print(throttle_set, DEC);
      Serial.println("");      
      
      Serial.print("INPUT: e-stop in:"); 
      Serial.print(ESTOP, DEC);
      Serial.println(""); 
      Serial.print("INPUT: Local control:"); 
      Serial.print(LOCAL_CONTROL, DEC);
      Serial.println("");
      Serial.print("INPUT: Turbo Button"); 
      Serial.println(TURBO_PUSHED, DEC);
      Serial.println("");
      Serial.print("RC: throttle in (uSec):");
      Serial.println(throttleInPulseLength);
      Serial.print("RC: steering in (uSec):");
      Serial.println(steeringInPulseLength);
      Serial.print("RC: lights in (uSec):");
      Serial.println(lightsInPulseLength);     
      
      Serial.print("STEERING: steering feedback(0-1023):");
      Serial.println(steering_feedback, DEC);
      Serial.print("STEERING: set (0-1023):");
      Serial.println(steering_set, DEC);
      Serial.print("STEERING: delta:");
      Serial.println(steering_delta, DEC);
      

      Serial.print("A0 (0-1023):");
      Serial.println(analogRead(A0));
      Serial.print("A1 (0-1023):");
      Serial.println(analogRead(A1));
      Serial.print("A2 (0-1023):");
      Serial.println(analogRead(A2));
      Serial.print("A3 (0-1023):");
      Serial.println(analogRead(A3));
      delay(5000);
    }
    else
    {
      delay(500);
    }
    
    bUpdateFlags = 0;
  }
}


// simple interrupt service routine
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    throttleInPulseLengthStart = TCNT1;
  }
  else
  {
    throttleInPulseLengthShared = (TCNT1 - throttleInPulseLengthStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(PCintPort::pinState)
  {
    steeringInPulseLengthStart = TCNT1;
  }
  else
  {
    steeringInPulseLengthShared = (TCNT1 - steeringInPulseLengthStart)>>1;

    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcLighting()
{
  if(PCintPort::pinState)
  {
    lightsInPulseLengthStart = TCNT1;
  }
  else
  {
    lightsInPulseLengthShared = (TCNT1 - lightsInPulseLengthStart)>>1;
    bUpdateFlagsShared |= LIGHTING_FLAG;
  }
}
