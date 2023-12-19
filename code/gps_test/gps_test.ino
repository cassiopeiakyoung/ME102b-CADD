#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// The TinyGPSPlus object
TinyGPSPlus gps;
SoftwareSerial ss(16, 17);

// const int freq = 5000;
// const int pwmChannel = 0;
// const int resolution = 8;
// const int thresh = 2000;
// volatile bool buttonIsPressed = false;
// int state = 1;

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  Serial.println("Initialized.");
  delay(3000);
}

void loop() {
  // while(ss.available() == 0) {
  //   Serial.println("connecting...");
  //   delay(1000);
  // }
  updateSerial();

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo() {

  Serial.print("Location: ");

  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  }  
  else {
    Serial.print("INVALID");
    Serial.println();
  }
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    ss.write(Serial.read()); // Forward what Serial received to Software Serial Port
  }
  while (ss.available()) {
    Serial.write(ss.read()); // Forward what Software Serial received to Serial Port
  }
}
