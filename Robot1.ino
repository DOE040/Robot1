#include <SoftwareSerial.h>

// sketch_dec18b_remote_robot

#define MOTORA_cwIN1 6
#define MOTORAccwIN2 11
#define MOTORB_cwIN3 10
#define MOTORBccwIN4 9
//#define EN1 6
//#define EN2 5

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
  digitalWrite(MOTORA_cwIN1, LOW);
  digitalWrite(MOTORAccwIN2, LOW);
  digitalWrite(MOTORB_cwIN3, LOW);
  digitalWrite(MOTORBccwIN4, LOW);
  //analogWrite(EN1,63);
  //analogWrite(EN2,63);
}

void loop()
{
  if (WebSocket.available())
  {  
//      data = BlueTooth.readStringUntil('\n');
      data = WebSocket.read();
      //WebSocket.print(str);             
    
    btVal = (data.toInt());
    WebSocket.print("WebSocket Value ");
    WebSocket.println(btVal);    
  }
  else if (BlueTooth.available())
  {  
//      data = BlueTooth.readStringUntil('\n');
      data = BlueTooth.read();
      //WebSocket.print(str);             
    
    btVal = (data.toInt());
    WebSocket.print("BlueTooth Value ");
    WebSocket.println(btVal);    
  }
  else
  {
    btVal = -1;
  }

  switch (btVal) 
  {
      case 49:                                
        WebSocket.println("Forward");
        forward();
        break;

      case 50:                 
        WebSocket.println("Reverse");
        reverse();
        break;

      case 51:         
       WebSocket.println("Left");
       left();
        break;
        
      case 52:                     
        WebSocket.println("Right");
        right();
        break;
        
      case 48:                                            
        WebSocket.println("Stop");
        stoprobot();
        break;      

  }
 
}

void forward()
{
  digitalWrite(MOTORA_cwIN1, LOW);
  digitalWrite(MOTORAccwIN2, HIGH);
  digitalWrite(MOTORB_cwIN3, LOW);
  digitalWrite(MOTORBccwIN4, HIGH);
}

void reverse()
{
  digitalWrite(MOTORA_cwIN1, HIGH);
  digitalWrite(MOTORAccwIN2, LOW);
  digitalWrite(MOTORB_cwIN3, HIGH);
  digitalWrite(MOTORBccwIN4, LOW);
}

void left()
{
  digitalWrite(MOTORA_cwIN1, LOW);
  digitalWrite(MOTORAccwIN2, LOW);
  digitalWrite(MOTORB_cwIN3, HIGH);
  digitalWrite(MOTORBccwIN4, LOW);
}

void right()
{
  digitalWrite(MOTORA_cwIN1, HIGH);
  digitalWrite(MOTORAccwIN2, LOW);
  digitalWrite(MOTORB_cwIN3, LOW);
  digitalWrite(MOTORBccwIN4, LOW);
}

void stoprobot()
{
  digitalWrite(MOTORA_cwIN1, LOW);
  digitalWrite(MOTORAccwIN2, LOW);
  digitalWrite(MOTORB_cwIN3, LOW);
  digitalWrite(MOTORBccwIN4, LOW);
}
