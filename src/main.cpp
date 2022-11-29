#include <Arduino.h>
#include <Servo.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Create the Player object
DFRobotDFPlayerMini player;
// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 2; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 3; // Connects to module's TX 
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
#define infraredMouth 4
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
for (int sensorCount = 1; sensorCount <= 4; sensorCount++)
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
    player.volume(30);
    //Stop player
    player.stop();
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

void moveTail(){
  servoTail.write(90);
  delay(200); 
  servoTail.write(0);
}

void moveHead(){
  servoHead.write(90);
  delay(200); 
  servoHead.write(0);
}

void movement(int timeIntervalInSeconds){
  unsigned long endTime = millis();
  unsigned long startTime = 0;
    while ((endTime - startTime) <= (timeIntervalInSeconds*1000))
  { 
    moveHead();
    moveTail();
    endTime= millis();
  }
}

uint8_t  readFromInfraredSensor(int sensor){
   return digitalRead(sensor);
}

void listenInfraredSensors(){
for (int sensorCount = 1; sensorCount <= 4; sensorCount++)
{
  uint8_t  sensorData= readFromInfraredSensor(infraredSensorList[sensorCount]);

  switch (infraredSensorList[sensorCount])
  {
  case infraredNose:
    if (sensorData==LOW)
    {
      player.play(noseSound);
      movement(40);
    }
    break;
  
  case infraredLeftEar:
    if (sensorData==LOW)
    {
      player.play(leftEarSound);
      movement(30);
    }
    break;

  case infraredRightEar:
    if (sensorData==LOW)
    {
      player.play(rightEarSound);
      movement(35);
    }
    break;

  case infraredMouth:
    if (sensorData==LOW)
    {
      player.play(mouthSound);
      movement(25);
    }
    break;

  default:
    break;
  }
}

}

void loop() {
listenInfraredSensors();
}