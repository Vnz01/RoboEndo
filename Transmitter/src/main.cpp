#include <SPI.h>
#include <RH_RF95.h>
#include <Arduino.h>

// Radio Communication Resources
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Button Pins Variables
const int joyXPin = A0;
const int joyYPin = A1;
const int lockPos = 13;
const int forward = 12;
const int backward = 11;
const int extraBtn = 10;
const int powerLED = 6;

// Latency
const int latency = 10;

// True state means unlocked, continue sending values
// False state means locked, position is stuck
int lockState = 1;
int forwState = 1;
int backState = 1;
int xtraState = 1;

// String Initialize
String joyX = "";
String joyY = "";
String XAndY = "500,500";
String totalData = "";


// On Interrupt sets the current state to the position it was in when it is pressed
void interruptLock() {
    lockState = digitalRead(lockPos);
}
void interruptForward() {
    forwState = digitalRead(forward);
}
void interruptBackward() {
    backState = digitalRead(backward);
}
void interruptXtra() {
    xtraState = digitalRead(extraBtn);
}

void setup() {
  Serial.begin(9600);

  // Radio Initializing
  Serial.println("RFM95 Transmitter Initializing...");
  while(!rf95.init()) {
    rf95.init();
    Serial.println("RFM95 initialization failed!");
    delay(1000);
  }
  Serial.println("RFM95 Transmitter Initialized");

  // Setup Pins and Interrupts
  pinMode(lockPos, INPUT_PULLUP);
  pinMode(forward, INPUT_PULLUP);
  pinMode(backward, INPUT_PULLUP);
  pinMode(extraBtn, INPUT_PULLUP);
  pinMode(powerLED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(lockPos), interruptLock, CHANGE);
  attachInterrupt(digitalPinToInterrupt(forward), interruptForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(backward), interruptBackward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(extraBtn), interruptXtra, CHANGE);

  // Find current state of buttons
  lockState = digitalRead(lockPos);
  forwState = digitalRead(forward);
  backState = digitalRead(backward);
  xtraState = digitalRead(extraBtn);
  digitalWrite(powerLED, HIGH);
}

void loop() {
  if(digitalRead(lockPos)){
    joyX = String(analogRead(joyXPin));
    joyY = String(analogRead(joyYPin));
    XAndY = joyX + "," + joyY;
  }

  totalData = "L" + String(lockState)+ "F" +String(forwState)+ "B" +String(backState)+ "X" + String(xtraState) + "P" + XAndY;

  rf95.send((uint8_t*)totalData.c_str(), totalData.length());
  rf95.waitPacketSent();

  // Fix if battery dies too quick, polling rate of analog data
  delay(latency);
  // Serial.println(totalData);
}