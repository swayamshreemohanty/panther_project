#include <Arduino.h>
#include <Servo.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Create the Player object
DFRobotDFPlayerMini player;
// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 2; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 3; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

//Sound config---
//The number is to identify the song order
#define mouthSound 1  
#define earSound 2
#define pithiSound 3
//---

//Infrared config---
//The number is to define digital pin
#define infraredMouth 4
#define infraredLeftEar 5
#define infraredRightEar 6
#define infraredPithi 7
int infraredSensorList[4]={infraredMouth,infraredLeftEar,infraredRightEar,infraredPithi};
//---

//Servo config---
//The number is to define digital pin
#define servoMouthPin 9
#define servoHeadPin 10
Servo servoMouth; 
Servo servoHead; 
//---




void setup() {
Serial.begin(9600);
for (int sensorCount = 1; sensorCount <= 4; sensorCount++)
{
  pinMode(infraredSensorList[sensorCount], INPUT);
}

  //Initialize servo
  servoHead.attach(servoHeadPin);
  servoMouth.attach(servoMouthPin);
  //

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial)) {
   Serial.println("DFPlayer Mini connected");
    // Set volume to maximum (0 to 30).
    player.volume(30);
    // // Play the first MP3 file on the SD card
    player.play(1);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

void moveMouth(){
  servoMouth.write(90);
  delay(200); 
  servoMouth.write(0);
}

void moveHead(){
  servoHead.write(90);
  delay(200); 
  servoHead.write(0);
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
  case infraredMouth:
    if (sensorData==HIGH)
    {
      player.play(mouthSound);
      moveMouth();
      moveHead();
    }
    break;
  
  case infraredLeftEar:
    if (sensorData==HIGH)
    {
      player.play(earSound);
    }
    break;

  case infraredRightEar:
    if (sensorData==HIGH)
    {
      player.play(earSound);
    }
    break;

  case infraredPithi:
    if (sensorData==HIGH)
    {
      player.play(pithiSound);
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