#include <SoftwareSerial.h>
#include <NewPing.h>

// sketch_dec18b_remote_robot

#define MOTORA_cwIN1  6
#define MOTORAccwIN2 11
#define MOTORB_cwIN3 10
#define MOTORBccwIN4  9
//#define EN1 6
//#define EN2 5
#define TRIGGER_PIN_FRONT  12
#define ECHO_PIN_FRONT     12
#define TRIGGER_PIN_RIGHT  13
#define ECHO_PIN_RIGHT     13
#define TRIGGER_PIN_LEFT    7
#define ECHO_PIN_LEFT       7
#define TRIGGER_PIN_BACK    8
#define ECHO_PIN_BACK       8
#define MAX_DISTANCE 400
#define SPEED 127

NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

//
// The hardware serial will be connected to the ESP32 WebSocket
// The software serial will be connected to the BlueTooth interface
//
#define WebSocket Serial
SoftwareSerial BlueTooth(2, 3); // RX, TX

// Define Variables

float duration1;          // First HC-SR04 pulse duration value
float duration2;          // Second HC-SR04 pulse duration value
float distance1;          // Calculated distance in cm for First Sensor
float distance2;          // Calculated distance in cm for Second Sensor
float soundsp;            // Speed of sound in m/s
float soundcm = 0.0343;   // Speed of sound in cm/ms
int iterations = 5;

String data;
int btVal;

void setup() 
{  
  WebSocket.begin(9600);
  BlueTooth.begin(9600);
  pinMode(MOTORA_cwIN1, OUTPUT);
  pinMode(MOTORAccwIN2, OUTPUT);
  pinMode(MOTORB_cwIN3, OUTPUT);
  pinMode(MOTORBccwIN4, OUTPUT);
  //pinMode(EN1, OUTPUT);
  //pinMode(EN2, OUTPUT);
  analogWrite(MOTORA_cwIN1, 0);
  analogWrite(MOTORAccwIN2, 0);
  analogWrite(MOTORB_cwIN3, 0);
  analogWrite(MOTORBccwIN4, 0);
  //analogWrite(EN1,63);
  //analogWrite(EN2,63);
  WebSocket.setTimeout(1000);
}

void loop()
{
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

  switch (btVal) 
  {
      case 49:                                
        BlueTooth.println("Forward");
        forward();
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
          BlueTooth.print("Out of range");
        }
        else {
          BlueTooth.print(distance1);
          BlueTooth.print(" cm");
        }
        break;      

  }
 
}

void forward()
{
  analogWrite(MOTORA_cwIN1, 0);
  analogWrite(MOTORAccwIN2, SPEED);
  analogWrite(MOTORB_cwIN3, 0);
  analogWrite(MOTORBccwIN4, SPEED);
}

void reverse()
{
  analogWrite(MOTORA_cwIN1, SPEED);
  analogWrite(MOTORAccwIN2, 0);
  analogWrite(MOTORB_cwIN3, SPEED);
  analogWrite(MOTORBccwIN4, 0);
}

void left()
{
  analogWrite(MOTORA_cwIN1, 0);
  analogWrite(MOTORAccwIN2, SPEED);
  analogWrite(MOTORB_cwIN3, SPEED);
  analogWrite(MOTORBccwIN4, 0);
}

void right()
{
  analogWrite(MOTORA_cwIN1, SPEED);
  analogWrite(MOTORAccwIN2, 0);
  analogWrite(MOTORB_cwIN3, 0);
  analogWrite(MOTORBccwIN4, SPEED);
}

void stoprobot()
{
  analogWrite(MOTORA_cwIN1, 0);
  analogWrite(MOTORAccwIN2, 0);
  analogWrite(MOTORB_cwIN3, 0);
  analogWrite(MOTORBccwIN4, 0);
}
