// #include <SPI.h>
// #include <RH_RF95.h>
#include <Arduino.h>

// Radio Communication Resources
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
// RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Button Pins Variables
const int joyXPin = A0;
const int joyYPin = A1;
const int lockPos = 13;
const int forward = 12;
const int backward = 11;
const int extraBtn = 10;

// True state means unlocked, continue sending values
// False state means locked, position is stuck
int lockState = 1;
int forwState = 1;
int backState = 1;
int xtraState = 1;

// // Track prev states for debouncing
// int lockPrevState = 1;
// int forwPrevState = 1;
// int backPrevState = 1;
// int xtraPrevState = 1;
// const unsigned long debounceInterval = 50;

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
  // Serial.println("RFM95 Transmitter Initializing...");
  // while(!rf95.init()) {
  //   rf95.init();
  //   Serial.println("RFM95 initialization failed!");
  //   delay(1000);
  // }
  // Serial.println("RFM95 Transmitter Initialized");

  // Setup Pins and Interrupts
  pinMode(lockPos, INPUT_PULLUP);
  pinMode(forward, INPUT_PULLUP);
  pinMode(backward, INPUT_PULLUP);
  pinMode(extraBtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lockPos), interruptLock, CHANGE);
  attachInterrupt(digitalPinToInterrupt(forward), interruptForward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(backward), interruptBackward, CHANGE);
  attachInterrupt(digitalPinToInterrupt(extraBtn), interruptXtra, CHANGE);

  // Find current state of buttons
  lockState = digitalRead(lockPos);
  forwState = digitalRead(forward);
  backState = digitalRead(backward);
  xtraState = digitalRead(extraBtn);

}

void loop() {
  if(digitalRead(lockPos)){
    // joyX = String(analogRead(joyXPin));
    // joyY = String(analogRead(joyYPin));
    // XAndY = joyX + "," + joyY;
    // rf95.send((uint8_t*)XAndY.c_str(), XAndY.length());
    // rf95.waitPacketSent();
    joyX = String(random(1000));
    joyY = String(random(1000));
    XAndY = joyX + "," + joyY;
  }

  // totalData = "LOCK:" + String(digitalRead(lockPos))+ " FORW:" +String(digitalRead(forward))+ " BACK:" +String(digitalRead(backward))+ " XTRA:" + String(digitalRead(extraBtn)) + " " + XAndY;
  totalData = "LOCK:" + String(lockState)+ " FORW:" +String(forwState)+ " BACK:" +String(backState)+ " XTRA:" + String(xtraState) + " " + XAndY;

  Serial.println(totalData);
  delay(100);
}



// const int batteryPin = A5; // Analog pin connected to BAT pin
// const int usbPin = 6;     // Digital pin connected to USB power input

// void setup() {
//   Serial.begin(9600);
//   pinMode(usbPin, INPUT_PULLDOWN);
// }

// float getBatteryVoltage() {
//   // Read the analog value and convert it to voltage
//   float voltage = analogRead(batteryPin) * (3.3 / 1023.0);
//   return voltage;
// }

// float getBatteryPercentage(float voltage, float minVoltage = 3.0, float maxVoltage = 4.2) {
//   // Calculate battery percentage based on voltage range
//   float percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
//   return constrain(percentage, 0, 100); // Ensure percentage is within 0-100 range
// }

// void loop() {
//   float batteryVoltage = getBatteryVoltage();
//   float batteryPercentage = getBatteryPercentage(batteryVoltage);
//   bool charging = digitalRead(usbPin) == HIGH;

//   Serial.print("Battery Voltage: ");
//   Serial.print(batteryVoltage);
//   Serial.print("V, ");
//   Serial.print("Battery Percentage: ");
//   Serial.print(batteryPercentage);
//   Serial.print("%, ");
//   Serial.print("Charging: ");
//   Serial.println(charging ? "Yes" : "No");

//   delay(1000); // Adjust delay as needed
// }
