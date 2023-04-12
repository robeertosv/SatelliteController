#include <SPI.h>
#include <LoRa.h>

#define ss 53
#define rst 49
#define dio0 48

void setup() {
  LoRa.setPins(ss);
  Serial.begin(115200);
  while (!Serial);

  if (!LoRa.begin(868050E3)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Receiver");
}

void loop() {
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
  
  }
}