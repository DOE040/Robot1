#include<SoftwareSerial.h>

// sketch_dec18b_remote_robot

#define IN1 12
#define IN2 11
#define IN3 10
#define IN4 9
//#define EN1 6
//#define EN2 5

SoftwareSerial mySerial(2, 3); // RX, TX

String data;
int btVal;

void setup() 
{  
  Serial.begin(115200);
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
  mySerial.begin(9600);
}

void loop()
{
 if (Serial.available())
 {  
//      data = mySerial.readStringUntil('\n');
      data = Serial.read();
      //Serial.print(str);             
    
    btVal = (data.toInt());
//    Serial.print("BlueTooth Value ");
//    Serial.println(btVal);    


  switch (btVal) 
   {
      case 49:                                
        Serial.println("Forward");
        forward();
        break;

      case 50:                 
        Serial.println("Reverse");
        reverse();
        break;

      case 51:         
       Serial.println("Left");
       left();
        break;
        
      case 52:                     
        Serial.println("Right");
        right();
        break;
        
      case 48:                                            
        Serial.println("Stop");
        stoprobot();
        break;      

  }
 }
 if (mySerial.available())
 {  
//      data = mySerial.readStringUntil('\n');
      data = mySerial.read();
      //Serial.print(str);             
    
    btVal = (data.toInt());
    Serial.print("BlueTooth Value ");
    Serial.println(btVal);    


  switch (btVal) 
   {
      case 49:                                
        Serial.println("Forward");
        forward();
        break;

      case 50:                 
        Serial.println("Reverse");
        reverse();
        break;

      case 51:         
       Serial.println("Left");
       left();
        break;
        
      case 52:                     
        Serial.println("Right");
        right();
        break;
        
      case 48:                                            
        Serial.println("Stop");
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
