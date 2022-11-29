#include <Arduino.h>
#include <Servo.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Create the Player object
DFRobotDFPlayerMini player;
// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 3; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 2; // Connects to module's TX 
static const int baudRate=9600;
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

//Sound config---
//The number is to identify the song order
#define mouthSound 1  
#define leftEarSound 2
#define rightEarSound 3
#define noseSound 4
//---

//Infrared config---
//The number is to define digital pin
#define infraredMouth 8
#define infraredLeftEar 5
#define infraredRightEar 6
#define infraredNose 7

int infraredSensorList[4]={infraredNose,infraredMouth,infraredLeftEar,infraredRightEar};
//---

//Servo config---
//The number is to define digital pin
#define servoTailPin 9
#define servoHeadPin 10
Servo servoTail; 
Servo servoHead; 
//---


void setup() {
Serial.begin(baudRate);
softwareSerial.begin(baudRate);
for (int sensorCount = 0; sensorCount < 4; sensorCount++)
{
  pinMode(infraredSensorList[sensorCount], INPUT_PULLUP);
}

  //Initialize servo
  servoHead.attach(servoHeadPin);
  servoTail.attach(servoTailPin);
  //

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial)) {
   Serial.println("DFPlayer Mini connected");
    // Set volume to maximum (0 to 30).
    player.volume(10);
    //Stop player
    player.stop();
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

void moveTail(){
  for (int degree = 0; degree <=180; degree++)
  {
    servoTail.write(degree);
    delay(20); 
  }
  delay(200); 
  for (int degree = 180; degree >=0; degree--)
  {
    servoTail.write(degree);
    delay(20); 
  }
  
}

void moveHead(){
  for (int degree = 0; degree <=180; degree++)
  {
    servoHead.write(degree);
    delay(20); 
  }
  delay(200); 
  for (int degree = 180; degree >=0; degree--)
  {
    servoHead.write(degree);
    delay(20); 
  }
}

void movement(int timeIntervalInSeconds){
  unsigned long endTime = millis();
  unsigned long startTime = endTime;
  while ((endTime - startTime) <= (timeIntervalInSeconds*1000))
  { 
    moveHead();
    delay(200); 
    moveTail();
    endTime= millis();
  }
}

uint8_t  readFromInfraredSensor(int sensor){
  delay(200);
   return digitalRead(sensor);
}

void listenInfraredSensors(){
for (int sensorCount = 0; sensorCount < 4; sensorCount++)
{
  uint8_t  sensorData= readFromInfraredSensor(infraredSensorList[sensorCount]);

    if (sensorData==LOW)
    {
  switch (infraredSensorList[sensorCount])
  {
  case infraredNose:
      player.play(noseSound);
      //Add servo running time in seconds
      movement(5);
    break;
  
  case infraredLeftEar:
      player.play(leftEarSound);
      //Add servo running time in seconds
      movement(8);
    
    break;

  case infraredRightEar:
      player.play(rightEarSound);
      //Add servo running time in seconds
      movement(10);
    
    break;

  case infraredMouth:
      player.play(mouthSound);
      //Add servo running time in seconds
      movement(13);
    
    break;

  default:
    break;
  }
    }

}

}

void loop() {
listenInfraredSensors();
}