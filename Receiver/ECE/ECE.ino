#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(9600);
  while (!Serial) ;

  Serial.println("RFM95 Receiver Initializing...");

  while(!rf95.init()) {
    rf95.init();
    Serial.println("RFM95 initialization failed!");
    delay(1000);
  }

  // rf95.setFrequency(915.0);  // Adjust frequency as needed

  Serial.println("RFM95 Receiver Initialized");
}

void loop() {
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    memset(buf,0,sizeof(buf));
    if (rf95.recv(buf, &len)) {
      Serial.println((char*)buf);
    } else {
      Serial.println("Receive failed");
    }
  }
}