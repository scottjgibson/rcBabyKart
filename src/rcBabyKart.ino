//Arduino based remote control GoKart
//Allows for control using a standard RC TX/RX
//Arduino interprets the output of the receiver and controls
//larger motors using an external h-bridge
//Steering is accomplished using an external potentiometer 
//attached to the steering motor to feedback the current
//wheel direction

#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>

//External Switches
#define ESTOP_IN_PIN 2
#define MANUAL_OVERRIDE_PIN 3

//Input from RC Receiver
#define THROTTLE_IN_PIN 5
#define STEERING_IN_PIN 6
#define AUX_IN_PIN 7

//Outputs to H-Bridges
#define THROTTLE_DIR_PIN 8
#define THROTTLE_PWM_PIN 9
#define STEERING_DIR_PIN 10
#define STEERING_PWM_PIN 11

//Feedback for steering "Servo"
#define STEERING_FEEDBACK_PIN A0


//External Inputs used when manual override enabled
#define STEERING_WHEEL_PIN A1
#define GAS_PEDAL_PIN A2

//deadband for steering, currently set to:
// 10*(5V/1024) = ~48mV
#define STEERING_DEADBAND 10

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 3

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;
uint16_t unThrottleInStart;
uint16_t unSteeringInStart;
uint16_t unAuxInStart;
uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("RCBaby 1.0");
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 1 Servos + 9 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,9*2000);
  CRCArduinoFastServos::begin();
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE);

  pinMode(THROTTLE_DIR_PIN, OUTPUT);
  pinMode(THROTTLE_PWM_PIN, OUTPUT);
  analogWrite(THROTTLE_PWM_PIN, 0);

  pinMode(STEERING_DIR_PIN, OUTPUT);
  pinMode(STEERING_PWM_PIN, OUTPUT);
  analogWrite(STEERING_PWM_PIN, 0);

  //Emergeny Stop input
  //Pulled up internally; must be kept low by safety circuit
  pinMode(ESTOP_IN_PIN, INPUT);
  digitalWrite(ESTOP_IN_PIN, HIGH);

  //Manual override pin active low
  pinMode(MANUAL_OVERRIDE_PIN, INPUT);
  digitalWrite(MANUAL_OVERRIDE_PIN, HIGH);
}

void loop()
{
  static int throttle;
  static int steering_set;
  static int steering_feedback;
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
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
  
    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }
   
    bUpdateFlagsShared = 0;
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }

  if (digitalRead(ESTOP_IN_PIN) != LOW)
  {
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      if (digitalRead(MANUAL_OVERRIDE_PIN) == LOW)
      {
        throttle = map(analogRead(GAS_PEDAL_PIN), 0,1023,-255,255);
      }
      else
      {
        throttle = map(unThrottleIn, 1500,2500,-255,255);
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
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      //Map the steering value to a 0-1023 value for easy comparing to the value read with analogRead
      steering_set = map(unSteeringIn, 1500,2500,0,1023);
    }

    steering_feedback = analogRead(STEERING_FEEDBACK_PIN);

    if (digitalRead(MANUAL_OVERRIDE_PIN) == LOW)
    {
      steering_set = analogRead(STEERING_WHEEL_PIN);
    }
    if ((steering_set - steering_feedback) > STEERING_DEADBAND)
    {
      if (steering_set > steering_feedback)
      {
        digitalWrite(STEERING_DIR_PIN, HIGH);
      }
      else
      {
        digitalWrite(STEERING_DIR_PIN, LOW);
      }

      //To Do: Set the steering speed based on how far we are from the set point
      //move the steering motor
      analogWrite(STEERING_PWM_PIN, 255);
    }
    else
    {
      //stop the steering motor
      analogWrite(STEERING_PWM_PIN, 0);
    }

    if(bUpdateFlags & AUX_FLAG)
    {
    }
  }
  else
  {
    analogWrite(STEERING_PWM_PIN, 0);
    analogWrite(THROTTLE_PWM_PIN, 0);
  }

  Serial.print("e-stop in:"); 
  Serial.println(digitalRead(ESTOP_IN_PIN));
  Serial.print("manual override:"); 
  Serial.println(digitalRead(MANUAL_OVERRIDE_PIN));
  Serial.print("throttle in (uSec):");
  Serial.println(unThrottleIn);
  Serial.print("throttle val (-255-255):");
  Serial.println(throttle);
  Serial.print("steering in (uSec):");
  Serial.print(unSteeringIn);
  Serial.print("steering in (degrees):");
  Serial.println(map(unSteeringIn, 1500, 2500, -90, 90));
  Serial.print("steering in (0-1023):");
  Serial.println(steering_set);
  Serial.print("steering feedback (0-1023):");
  Serial.println(steering_feedback);
  Serial.print("aux in: (uSec):");
  Serial.println(unAuxIn);

  delay(500);
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

void calcAux()
{
  if(PCintPort::pinState)
  {
    unAuxInStart = TCNT1;
  }
  else
  {
    unAuxInShared = (TCNT1 - unAuxInStart)>>1;
    bUpdateFlagsShared |= AUX_FLAG;  }
}
