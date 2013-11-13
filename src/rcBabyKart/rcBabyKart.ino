

// Arduino based remote control GoKart
// Allows for control using a standard RC TX/RX
// Arduino interprets the output of the receiver and controls
// larger motors using an external h-bridge
// Steering is accomplished using an external potentiometer 
// attached to the steering motor to feedback the current
// wheel direction

#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>

//OUTPUTS (To H-Bridge)
#define THROTTLE_DIR_PIN       4
#define THROTTLE_PWM_PIN       5
#define STEERING_DIR_PIN       2
#define STEERING_PWM_PIN       3
#define LIGHTING_PIN          12

//INPUTS:
//Input from RC Receiver
#define THROTTLE_RC_IN_PIN     6
#define STEERING_RC_IN_PIN     7
#define LIGHTING_RC_IN_PIN     8

//Switches
#define LOCAL_CONTROL_PIN      9
#define ESTOP_IN_PIN          10
#define TURBO_IN_PIN          11

//Feedback for steering Servo (steering column position)
#define STEERING_FEEDBACK_PIN A1

//External Inputs used when manual override enabled
//For Drive-By-Wire Steering
#define STEERING_WHEEL_PIN    A2
#define GAS_PEDAL_PIN         A3

#define DEBUG 1

//deadband for steering, currently set to:
// 20*(5V/1024) = ~100mV
#define STEERING_DEADBAND 20

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 3

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define LIGHTING_FLAG 4

//The ESTOP Pin  must be held low by safety circuit
//if the circuit is opened the signal is pulled up
#define ESTOP (digitalRead(ESTOP_IN_PIN) == HIGH)

//If manual override pin is pulled low it is active
#define LOCAL_CONTROL (digitalRead(LOCAL_CONTROL_PIN) == LOW)

volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unLightingInShared;
uint16_t unThrottleInStart;
uint16_t unSteeringInStart;
uint16_t unLightingInStart;

void setup()
{
  Serial.begin(9600);
  Serial.println("RCBabyKart 1.0");
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 1 Servos + 9 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,9*2000);
  CRCArduinoFastServos::begin();
  PCintPort::attachInterrupt(THROTTLE_RC_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_RC_IN_PIN, calcSteering,CHANGE);
  PCintPort::attachInterrupt(LIGHTING_RC_IN_PIN, calcLighting,CHANGE);
  PCintPort::attachInterrupt(SAFETY_RC_IN_PIN, calcSafety,CHANGE);

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
}

uint8_t turbo_active = 0;

void loop()
{
  static int throttle;
  static int steering_set;
  static int steering_feedback;
  static int steering_delta;
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unLightingIn;
  static uint8_t bUpdateFlags;


// check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
  
    if(bUpdateFlags & LIGHTING_FLAG)
    {
      unLightingIn = unLightingInShared;
    }
   
    bUpdateFlagsShared = 0;
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }

  if (ESTOP)
  {
    analogWrite(STEERING_PWM_PIN, 0);
    analogWrite(THROTTLE_PWM_PIN, 0);
  }
  else
  {
    if (LOCAL_CONTROL)
    {
      throttle = map(analogRead(GAS_PEDAL_PIN), 0,1023,-255,255);
      steering_set = analogRead(STEERING_WHEEL_PIN);
    }
    else
    {
      //Remote Control
      throttle = map(unThrottleIn, 1000,2000,-255,255);

      if (!turbo_active)
      {
        throttle /= 2;
      }

      //Map the steering value to a 0-1023 value for easy comparing to the value read with analogRead
      steering_set = map(unSteeringIn, 1000,2000,0,1023);
    }
    
    
    if (throttle >= 0)
    {
      digitalWrite(THROTTLE_DIR_PIN, HIGH);
    }
    else
    {
      digitalWrite(THROTTLE_DIR_PIN, LOW);
    }
    analogWrite(THROTTLE_PWM_PIN, abs(throttle));
    
 #if 0
    //Currently no steering feedback pot

    steering_feedback = analogRead(STEERING_FEEDBACK_PIN);

    if (abs(steering_set - steering_feedback) > STEERING_DEADBAND)
    {
      if(DEBUG)
      {
        Serial.print("steering_set:"); 
        Serial.println(steering_set);
        Serial.print("steering_feedback:"); 
        Serial.println(steering_feedback);
      }
      if (steering_set > steering_feedback)
      {
        digitalWrite(STEERING_DIR_PIN, HIGH);
      }
      else
      {
        digitalWrite(STEERING_DIR_PIN, LOW);
      }
      steering_delta = abs(steering_set - steering_feedback);
      analogWrite(STEERING_PWM_PIN, map(steering_delta, 0, 1024, 50,255));
    }
    else
    {
      //stop the steering motor
      analogWrite(STEERING_PWM_PIN, 0);
    }
#else
  //Simplified Steering Control
    if ((steering_set - 512) > 0)
    {
      digitalWrite(STEERING_DIR_PIN, HIGH);
    }
    else
    {
      digitalWrite(STEERING_DIR_PIN, LOW);
    }

    if (abs(steering_set - 512) > STEERING_DEADBAND)
    {
      analogWrite(STEERING_PWM_PIN, 128);
    }
#endif

    if (unLightingIn > 1700)
    {
      digitalWrite(LIGHTING_PIN, HIGH);
    }
    else
    {
      digitalWrite(LIGHTING_PIN, LOW);
    }
  }

  
  if (DEBUG)
  {
    Serial.print("e-stop in:"); 
    Serial.println(digitalRead(ESTOP_IN_PIN));
    Serial.print("Local control:"); 
    Serial.println(digitalRead(LOCAL_CONTROL_PIN));
    Serial.print("throttle in (uSec):");
    Serial.println(unThrottleIn);
    Serial.print("throttle val (-255-255):");
    Serial.println(throttle);
    Serial.print("steering in (uSec):");
    Serial.println(unSteeringIn);
    Serial.print("steering in (degrees):");
    Serial.println(map(unSteeringIn, 1000, 2000, -90, 90));
    Serial.print("steering in (0-1023):");
    Serial.println(steering_set);
    Serial.print("steering feedback (0-1023):");
    Serial.println(steering_feedback);
    Serial.print("aux in: (uSec):");
    Serial.println(unLightingIn);
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


// simple interrupt service routine
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(PCintPort::pinState)
  {
    unSteeringInStart = TCNT1;
  }
  else
  {
    unSteeringInShared = (TCNT1 - unSteeringInStart)>>1;

    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcLighting()
{
  if(PCintPort::pinState)
  {
    unLightingInStart = TCNT1;
  }
  else
  {
    unLightingInShared = (TCNT1 - unLightingInStart)>>1;
    bUpdateFlagsShared |= LIGHTING_FLAG;
  }
}

void calcSafety()
{
  if(PCintPort::pinState)
  {
    unSafetyInStart = TCNT1;
  }
  else
  {
    unSafetyInShared = (TCNT1 - unSafetyInStart)>>1;
    bUpdateFlagsShared |= SAFETY_FLAG;
  }
}
