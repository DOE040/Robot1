#include <SoftwareSerial.h>
#include <NewPing.h>
// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"

 
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1
 
// Integers for pulse counters
volatile unsigned int counter1 = 0;
volatile unsigned int counter2 = 0;
 
// Float for number of slots in encoder disk
float diskslots = 20;  // Change to match value of encoder disk
 
// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

const float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
const float cm_step = circumference / diskslots;  // CM per Step

// sketch_dec18b_remote_robot

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

#define SPEED 200

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
  if(counter1 > 0 || counter2 > 0)
  {
    BlueTooth.print("Motor Speed 1: "); 
    float rotation1 = (counter1 / diskslots) * 60.00;  // calculate RPM for Motor 1
    BlueTooth.print(rotation1);  
    BlueTooth.print(" RPM - "); 
    counter1 = 0;  //  reset counter to zero

    BlueTooth.print("Motor Speed 2: "); 
    float rotation2 = (counter2 / diskslots) * 60.00;  // calculate RPM for Motor 2
    BlueTooth.print(rotation2);  
    BlueTooth.println(" RPM"); 
    counter2 = 0;  //  reset counter to zero
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

  
  duration1 = sonarFront.ping_median(iterations); // Measure duration for first sensor
  distance1 = (duration1 / 2) * soundcm;          // Calculate the distance
  duration2 = sonarBack.ping_median(iterations); // Measure duration for first sensor
  distance2 = (duration2 / 2) * soundcm;          // Calculate the distance
  duration3 = sonarRight.ping_median(iterations); // Measure duration for first sensor
  distance3 = (duration3 / 2) * soundcm;          // Calculate the distance
  duration4 = sonarLeft.ping_median(iterations); // Measure duration for first sensor
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

  if (distance1 < 20 && CurrentAction == FORWARD)
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
        if (distance1 >= 400 || distance1 <= 2) {
          BlueTooth.println("Out of range");
        }
        else {
          BlueTooth.print(distance1);
          BlueTooth.println(" cm");
        }

        BlueTooth.print("Distance back: ");
        if (distance2 >= 400 || distance2 <= 2) {
          BlueTooth.println("Out of range");
        }
        else {
          BlueTooth.print(distance2);
          BlueTooth.println(" cm");
        }

        BlueTooth.print("Distance right: ");
        if (distance3 >= 400 || distance3 <= 2) {
          BlueTooth.println("Out of range");
        }
        else {
          BlueTooth.print(distance3);
          BlueTooth.println(" cm");
        }

        BlueTooth.print("Distance left: ");
        if (distance4 >= 400 || distance4 <= 2) {
          BlueTooth.println("Out of range");
        }
        else {
          BlueTooth.print(distance4);
          BlueTooth.println(" cm");
        }

        
        
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
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void reverse()
{
  CurrentAction = REVERSE;
  digitalWrite(MOTORA_cwIN1, 1);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 1);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void left()
{
  CurrentAction = LEFT;
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 1);
  digitalWrite(MOTORB_cwIN3, 1);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void right()
{
  CurrentAction = RIGHT;
  digitalWrite(MOTORA_cwIN1, 1);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 1);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void stoprobot()
{
  CurrentAction = STOP;
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    0);
  analogWrite(MOTORB_EN2,    0);
}
