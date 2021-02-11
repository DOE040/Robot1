#include <SoftwareSerial.h>

// sketch_dec18b_remote_robot

#define MOTORA_cwIN1 6
#define MOTORAccwIN2 11
#define MOTORB_cwIN3 10
#define MOTORBccwIN4 9
//#define EN1 6
//#define EN2 5

#define SPEED 127
//
// The hardware serial will be connected to the ESP32 WebSocket
// The software serial will be connected to the BlueTooth interface
//
#define WebSocket Serial
SoftwareSerial BlueTooth(2, 3); // RX, TX

String data;
int btVal;

void setup() 
{  
  WebSocket.begin(115200);
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
  if (WebSocket.available())
  {  
      data = WebSocket.readStringUntil('\n');
//      data = WebSocket.read();
      BlueTooth.print(data);             
    
    btVal = (data.toInt());
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
