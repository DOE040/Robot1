#include <SoftwareSerial.h>
#include <NewPing.h>
// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"

 
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1
 
// Integers for pulse counters
volatile unsigned long counter1    = 0;
volatile unsigned long counter1Old = 0;
volatile unsigned long counter2    = 0;
volatile unsigned long counter2Old = 0;


// Float for number of slots in encoder disk
float diskslots = 20;  // Change to match value of encoder disk
 
// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

const float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
const float cm_step = circumference / diskslots;  // CM per Step

/* typedef */ enum Movement { STAND_STILL, GO_FORWARD, GO_BACKWARD, SPIN_LEFT, SPIN_RIGHT };

Movement RequestedMove = STAND_STILL;

#define MOTORA_cwIN1 A2
#define MOTORAccwIN2 A3
#define MOTORB_cwIN3 A4
#define MOTORBccwIN4 A5
#define MOTORA_EN1    6
#define MOTORB_EN2   11

#define STOP    48
#define FORWARD 49
#define REVERSE 50
#define LEFT    51
#define RIGHT   52
#define INFO    53

unsigned int CurrentAction;

#define SPEED    200
#define SPEEDCMS  80.0

#define TRIGGER_PIN_FRONT  12
#define ECHO_PIN_FRONT     12
#define TRIGGER_PIN_RIGHT  13
#define ECHO_PIN_RIGHT     13
#define TRIGGER_PIN_LEFT    7
#define ECHO_PIN_LEFT       7
#define TRIGGER_PIN_BACK    8
#define ECHO_PIN_BACK       8
#define MAX_DISTANCE 400

NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarBack(TRIGGER_PIN_BACK, ECHO_PIN_BACK, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(ECHO_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);

//
// The hardware serial will be connected to the ESP32 WebSocket
// The software serial will be connected to the BlueTooth interface
//
#define WebSocket Serial
SoftwareSerial BlueTooth(4, 5); // RX, TX

// Define Variables

float duration1;          // First HC-SR04 pulse duration value
float duration2;          // Second HC-SR04 pulse duration value
float duration3;          // First HC-SR04 pulse duration value
float duration4;          // Second HC-SR04 pulse duration value
float distance1;          // Calculated distance in cm for First Sensor
float distance2;          // Calculated distance in cm for Second Sensor
float distance3;          // Calculated distance in cm for First Sensor
float distance4;          // Calculated distance in cm for Second Sensor
float soundsp;            // Speed of sound in m/s
float soundcm = 0.0343;   // Speed of sound in cm/ms
int iterations = 5;

unsigned long LastLoop,  PreviousLoop, LoopTime;
unsigned long LoopMin,   LoopAvg,      LoopMax;
unsigned long LoopCount, InitTime;

volatile float speedL;
volatile float speedR;

float SetpointL, ErrorL, PreviousErrorL, AccumulatedErrorL;
float SetpointR, ErrorR, PreviousErrorR, AccumulatedErrorR;

float Kp = 0.1;
float Ki = 0.1;
float Kd = 0.0;

String data;
int btVal;

// Function to convert from centimeters to steps
int CMtoSteps(float cm) {
 
  int result;  // Final calculation result
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result
}

// Interrupt Service Routines
 
// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 
 
// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 
 
// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  if(counter1 > counter1Old || counter2 > counter2Old)
  {
    BlueTooth.print("Motor Speed 1: "); 
    float rotation1 = ((counter1 - counter1Old) / diskslots) * 60.00;  // calculate RPM for Motor 1
    speedL = ((counter1 - counter1Old) / diskslots) * circumference;
    BlueTooth.print(speedL);  
    BlueTooth.print("[cm/s] - "); 
    BlueTooth.print(rotation1);  
    BlueTooth.print("[RPM] - "); 
    counter1Old = counter1;  //  reset counter to zero

    BlueTooth.print("Motor Speed 2: "); 
    float rotation2 = ((counter2 - counter2Old) / diskslots) * 60.00;  // calculate RPM for Motor 2
    speedR = ((counter2 - counter2Old) / diskslots) * circumference;
    BlueTooth.print(speedR);  
    BlueTooth.print("[cm/s] - "); 
    BlueTooth.print(rotation2);  
    BlueTooth.println("[RPM]"); 
    counter2Old = counter2;  //  reset counter to zero
  }
  else
  {
    speedL = 0.0;
    speedR = 0.0;
  }
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}
 
void setup() 
{  
  WebSocket.begin(9600);
  BlueTooth.begin(9600);
  analogReference(INTERNAL);
  pinMode(MOTORA_cwIN1, OUTPUT);
  pinMode(MOTORAccwIN2, OUTPUT);
  pinMode(MOTORB_cwIN3, OUTPUT);
  pinMode(MOTORBccwIN4, OUTPUT);
  //pinMode(EN1, OUTPUT);
  //pinMode(EN2, OUTPUT);
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    0);
  analogWrite(MOTORB_EN2,    0);

  WebSocket.setTimeout(1000);

  Timer1.initialize(1000000); // set timer for 1sec
  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);
  // Increase counter 2 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer

  InitTime  = micros();
  LastLoop  = InitTime;
  LoopMin   = 9999;
  LoopMax   =    0;

  SetpointL      = 0.0;
  ErrorL         = 0.0;
  PreviousErrorL = 0.0;
  SetpointR      = 0.0;
  ErrorR         = 0.0;
  PreviousErrorR = 0.0;
}

void loop()
{
  LoopCount++;
  PreviousLoop = LastLoop;
  LastLoop     = micros();
  //
  // micros() rolls over every 70 minutes.
  // when that happens, start counting again
  //
  if (LastLoop > PreviousLoop)
    LoopTime     = LastLoop - PreviousLoop;
  else
    LoopCount = 1;
  if (LoopCount > 1 && LoopTime < LoopMin) LoopMin = LoopTime;
  if (LoopTime > LoopMax)                  LoopMax = LoopTime;
  LoopAvg      = LastLoop / LoopCount;

  //
  // Save error values from previous loop to calculate differentials
  //
  PreviousErrorL = ErrorL;
  PreviousErrorR = ErrorR;
  //
  // The error is the difference between the setpoint and the measured speed
  //
  ErrorL = SetpointL - speedL;
  ErrorR = SetpointR - speedR;
  //
  // Errors are accumulated for the I-action of the controller
  // Multiply by the loop time to correct for irregular loop speed
  //
  AccumulatedErrorL += ErrorL * (LoopTime / 1000000.0);
  AccumulatedErrorR += ErrorR * (LoopTime / 1000000.0);
  //
  // Errors are differentiated for the D-action of the controller
  // Divide by the loop time to correct for irregular loop speed
  //
  float DiffErrorL = (ErrorL - PreviousErrorL) / (LoopTime / 1000000.0);
  float DiffErrorR = (ErrorR - PreviousErrorR) / (LoopTime / 1000000.0);
  //
  // Controller algorithm in three parts for debugging and educational purposes
  //
  float pPartL = Kp * ErrorL;
  float pPartR = Kp * ErrorR;
  float iPartL = Ki * AccumulatedErrorL;
  float iPartR = Ki * AccumulatedErrorR;
  float dPartL = Kd * DiffErrorL;
  float dPartR = Kd * DiffErrorR;
  //
  // Add up the parts to get the output signals of the controllers
  //
  float PWML = pPartL + iPartL + dPartL;
  float PWMR = pPartR + iPartR + dPartR;
  //
  // PWM range is 0...255. Prevent running out of bounds
  //
  if (PWML>255) PWML = 255;
  if (PWMR>255) PWMR = 255;
  if (PWML<0)   PWML = 0;
  if (PWMR<0)   PWMR = 0;
  //
  // When standing still do not accumulate errors and do not drive motors
  //
  if (RequestedMove == STAND_STILL)
  {
    AccumulatedErrorL = 0.0;
    AccumulatedErrorR = 0.0;
    ErrorL = 0.0;
    ErrorR = 0.0;
    PWML = 0;
    PWMR = 0;
  }
  else
  {
    BlueTooth.print("PWML = "); BlueTooth.print(PWML); BlueTooth.print(" PWMR = "); BlueTooth.println(PWMR);    
  }
  analogWrite(MOTORA_EN1,    PWML);
  analogWrite(MOTORB_EN2,    PWMR);

  duration1 = sonarFront.ping_median(iterations); // Measure duration for front sensor
  distance1 = (duration1 / 2) * soundcm;          // Calculate the distance
  if (distance1 == 0) distance1 = 401;
  duration2 = sonarBack.ping_median(iterations);  // Measure duration for back sensor
  distance2 = (duration2 / 2) * soundcm;          // Calculate the distance
  if (distance2 == 0) distance2 = 401;
  duration3 = sonarRight.ping_median(iterations); // Measure duration for right sensor
  distance3 = (duration3 / 2) * soundcm;          // Calculate the distance
  duration4 = sonarLeft.ping_median(iterations);  // Measure duration for left sensor
  distance4 = (duration4 / 2) * soundcm;          // Calculate the distance

  if (WebSocket.available())
  {  
      data = WebSocket.readStringUntil('\n');
//      data = WebSocket.read();
      BlueTooth.print(data);             
    
    // btVal = (data.toInt());
    btVal = data.charAt(data.length() - 1);
    BlueTooth.print("WebSocket Value ");
    BlueTooth.println(btVal);    
  }
  else if (BlueTooth.available())
  {  
//      data = BlueTooth.readStringUntil('\n');
      data = BlueTooth.read();
      //BlueTooth.print(str);             
    
    btVal = (data.toInt());
    BlueTooth.print("BlueTooth Value ");
    BlueTooth.println(btVal);    
  }
  else
  {
    btVal = -1;
  }

  if (distance1 < 30 && CurrentAction == FORWARD)
  {
    stoprobot();
    BlueTooth.println("STOPPED by sensor");
  }
  if (distance2 < 30 && CurrentAction == REVERSE)
  {
    stoprobot();
    BlueTooth.println("STOPPED by sensor");
  }
  switch (btVal) 
  {
      case FORWARD:                                
        BlueTooth.println("Forward");
        if (distance1 > 5)
          forward();
        else
          BlueTooth.println("BLOCKED by sensor");
        break;

      case REVERSE:                 
        BlueTooth.println("Reverse");
        reverse();
        break;

      case LEFT:         
       BlueTooth.println("Left");
       left();
        break;
        
      case RIGHT:                     
        BlueTooth.println("Right");
        right();
        break;
        
      case STOP:                                            
        BlueTooth.println("Stop");
        stoprobot();
        break;      

      case INFO:                                            
        BlueTooth.print("Distance front: ");
        BlueTooth.print(distance1);
        BlueTooth.println(" cm");

        BlueTooth.print("Distance back: ");
        BlueTooth.print(distance2);
        BlueTooth.println(" cm");

        BlueTooth.print("Distance right: ");
        BlueTooth.print(distance3);
        BlueTooth.println(" cm");

        BlueTooth.print("Distance left: ");
        BlueTooth.print(distance4);
        BlueTooth.println(" cm");
        
        
        BlueTooth.print("LoopCount: "); BlueTooth.print(LoopCount);       BlueTooth.println("");
        BlueTooth.print("LoopTime : "); BlueTooth.print(LoopTime/1000.0); BlueTooth.println("[ms]");
        BlueTooth.print("LoopMin  : "); BlueTooth.print(LoopMin/1000.0);  BlueTooth.println("[ms]");
        BlueTooth.print("LoopAvg  : "); BlueTooth.print(LoopAvg/1000.0);  BlueTooth.println("[ms]");
        BlueTooth.print("LoopMax  : "); BlueTooth.print(LoopMax/1000.0);  BlueTooth.println("[ms]");
        BlueTooth.print("Analog0  : "); BlueTooth.print(analogRead(A0));  BlueTooth.println("");
        BlueTooth.print("Analog1  : "); BlueTooth.print(analogRead(A1));  BlueTooth.println("");

        break;      

  }
 
}

void forward()
{
  CurrentAction = FORWARD;
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 1);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 1);
//  analogWrite(MOTORA_EN1,    SPEED);
//  analogWrite(MOTORB_EN2,    SPEED);
  RequestedMove = GO_FORWARD;
  SetpointL     = SPEEDCMS;
  SetpointR     = SPEEDCMS;
}

void reverse()
{
  CurrentAction = REVERSE;
  digitalWrite(MOTORA_cwIN1, 1);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 1);
  digitalWrite(MOTORBccwIN4, 0);
//  analogWrite(MOTORA_EN1,    SPEED);
//  analogWrite(MOTORB_EN2,    SPEED);
  RequestedMove = GO_BACKWARD;
  SetpointL     = SPEEDCMS;
  SetpointR     = SPEEDCMS;
}

void left()
{
  CurrentAction = LEFT;
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 1);
  digitalWrite(MOTORB_cwIN3, 1);
  digitalWrite(MOTORBccwIN4, 0);
//  analogWrite(MOTORA_EN1,    SPEED);
//  analogWrite(MOTORB_EN2,    SPEED);
  RequestedMove = SPIN_LEFT;
  SetpointL     = SPEEDCMS;
  SetpointR     = SPEEDCMS;
}

void right()
{
  CurrentAction = RIGHT;
  digitalWrite(MOTORA_cwIN1, 1);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 1);
//  analogWrite(MOTORA_EN1,    SPEED);
//  analogWrite(MOTORB_EN2,    SPEED);
  RequestedMove = SPIN_RIGHT;
  SetpointL     = SPEEDCMS;
  SetpointR     = SPEEDCMS;
}

void stoprobot()
{
  CurrentAction = STOP;
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 0);
//  analogWrite(MOTORA_EN1,    0);
//  analogWrite(MOTORB_EN2,    0);
  RequestedMove = STAND_STILL;
  SetpointL     = 0.0;
  SetpointR     = 0.0;
}
