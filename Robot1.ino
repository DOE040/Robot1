#include <SoftwareSerial.h>
// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"
#include <NewPing.h>
// Constants for Interrupt Pins
// Change values if not using Arduino Uno
 
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1
 
// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;
 
// Float for number of slots in encoder disk
float diskslots = 20;  // Change to match value of encoder disk
 

// sketch_dec18b_remote_robot

#define MOTORA_cwIN1 A2
#define MOTORAccwIN2 A3
#define MOTORB_cwIN3 A4
#define MOTORBccwIN4 A5
#define MOTORA_EN1    6
#define MOTORB_EN2   11
#define TRIGGER_PIN_FRONT  12
#define ECHO_PIN_FRONT     12
#define TRIGGER_PIN_RIGHT  13
#define ECHO_PIN_RIGHT     13
#define TRIGGER_PIN_LEFT    7
#define ECHO_PIN_LEFT       7
#define TRIGGER_PIN_BACK    8
#define ECHO_PIN_BACK       8
#define MAX_DISTANCE 400
#define SPEED 200

NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

//
// The hardware serial will be connected to the ESP32 WebSocket
// The software serial will be connected to the BlueTooth interface
//
#define WebSocket Serial
SoftwareSerial BlueTooth(4, 5); // RX, TX

// Define Variables

float duration1;          // First HC-SR04 pulse duration value
float duration2;          // Second HC-SR04 pulse duration value
float distance1;          // Calculated distance in cm for First Sensor
float distance2;          // Calculated distance in cm for Second Sensor
float soundsp;            // Speed of sound in m/s
float soundcm = 0.0343;   // Speed of sound in cm/ms
int iterations = 5;
unsigned long LastLoop, PreviousLoop, LoopTime;
unsigned long LoopMin,  LoopAvg,      LoopMax;
unsigned long LoopCount, InitTime;

String data;
int btVal;
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
  if(counter1 > 0)
  {
    BlueTooth.print("Motor Speed 1: "); 
    float rotation1 = (counter1 / diskslots) * 60.00;  // calculate RPM for Motor 1
    BlueTooth.print(rotation1);  
    BlueTooth.print(" RPM - "); 
    counter1 = 0;  //  reset counter to zero
  }
  if(counter2 > 0)
  {
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

  InitTime  = micros();
  LastLoop  = InitTime;
  LoopMin   = 9999;
  LoopMax   =    0;

  Timer1.initialize(1000000); // set timer for 1sec
  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);
  // Increase counter 2 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
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

  // Measure duration for first sensor
  duration1 = sonarFront.ping_median(iterations);
  // Calculate the distance
  distance1 = (duration1 / 2) * soundcm;

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

  if (distance1 < 6)
  {
    stoprobot();
  }
  switch (btVal) 
  {
      case 49:                                
        BlueTooth.println("Forward");
        if (distance1 > 5)
          forward();
        else
          BlueTooth.println("BLOCKED by sensor");
        break;

      case 50:                 
        BlueTooth.println("Reverse");
        reverse();
        break;

      case 51:         
       BlueTooth.println("Left");
       left();
        break;
        
      case 52:                     
        BlueTooth.println("Right");
        right();
        break;
        
      case 48:                                            
        BlueTooth.println("Stop");
        stoprobot();
        break;      

      case 53:                                            
        BlueTooth.print("Distance front: ");

        if (distance1 >= 400 || distance1 <= 2) {
          BlueTooth.println("Out of range");
        }
        else {
          BlueTooth.print(distance1);
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
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 1);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 1);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void reverse()
{
  digitalWrite(MOTORA_cwIN1, 1);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 1);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void left()
{
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 1);
  digitalWrite(MOTORB_cwIN3, 1);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void right()
{
  digitalWrite(MOTORA_cwIN1, 1);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 1);
  analogWrite(MOTORA_EN1,    SPEED);
  analogWrite(MOTORB_EN2,    SPEED);
}

void stoprobot()
{
  digitalWrite(MOTORA_cwIN1, 0);
  digitalWrite(MOTORAccwIN2, 0);
  digitalWrite(MOTORB_cwIN3, 0);
  digitalWrite(MOTORBccwIN4, 0);
  analogWrite(MOTORA_EN1,    0);
  analogWrite(MOTORB_EN2,    0);
}
