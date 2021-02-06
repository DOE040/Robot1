#include <SoftwareSerial.h>

// sketch_dec18b_remote_robot

#define IN1 12
#define IN2 11
#define IN3 10
#define IN4 9
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
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //pinMode(EN1, OUTPUT);
  //pinMode(EN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  //analogWrite(EN1,63);
  //analogWrite(EN2,63);
}

void loop()
{
 if (WebSocket.available())
 {  
//      data = BlueTooth.readStringUntil('\n');
      data = BlueTooth.read();
      //WebSocket.print(str);             
    
    btVal = (data.toInt());
    WebSocket.print("BlueTooth Value ");
    WebSocket.println(btVal);    


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
 if (BlueTooth.available())
 {  
//      data = BlueTooth.readStringUntil('\n');
      data = BlueTooth.read();
      //WebSocket.print(str);             
    
    btVal = (data.toInt());
    WebSocket.print("BlueTooth Value ");
    WebSocket.println(btVal);    


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
 
}

void forward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void reverse()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void stoprobot()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
