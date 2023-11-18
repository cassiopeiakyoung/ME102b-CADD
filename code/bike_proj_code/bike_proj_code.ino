w // Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#define BTN 27

const char* ssid = "xtr";
const char* password = "cracklepop!";

WiFiServer server(80); // Port 80
Adafruit_MPU6050 mpu;

int hallPin1 = 15;
int hallPin2 = 14;
int totalAccel = 0;
int event = 1;
// event 1 = disarmed, event 2 = armed, event 3 = alarming
int wait30 = 30000;
bool detected1 = false;
bool detected2 = false;
String buttonState = "";
volatile bool buttonIsPressed = false;

void hall_detect_1()
{
  detected1 = true;
}

void hall_detect_2()
{
  detected2 = true;
}

void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
  buttonIsPressed = true;
}

void setup() {
  Serial.begin(115200);

//  // Connect to wifi.
//  Serial.println();
//  Serial.print("Connecting with ");
//  Serial.println(ssid);
//
//  WiFi.begin(ssid, password);
// 
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("Connected with WiFi.");
// 
//  // Start Web Server.
//  server.begin();
//  Serial.println("Web Server started.");
// 
//  // print ip
//  Serial.print("http://");
//  Serial.println(WiFi.localIP());

  attachInterrupt(BTN, isr, RISING);
  attachInterrupt(hallPin1, hall_detect_1, RISING);
  attachInterrupt(hallPin2, hall_detect_2, RISING);
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}


void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  totalAccel = pow((pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2)), 0.5);

//  if (wheel_turn()){
//    Serial.println("wheel turned");
//  }
//  if (totalAccel > 20){
//    Serial.println(totalAccel);
//  }
  
//////////////////////////////////////////////////////////
// README!!!!!
// Commenting out the wireless arming system code for now.
// Seems to only (for some reason) run the state machine
// code when a button on the website is pressed, i.e. it
// won't detect when accel/encoders are tripped when
// button is not clicked (!!). Will try working on it
// after A5 is turned in.
//////////////////////////////////////////////////////////


//  // If disconnected, try to reconnect every 30 seconds.
//  if ((WiFi.status() != WL_CONNECTED) && (millis() > wait30)) {
//    Serial.println("Trying to reconnect WiFi...");
//    WiFi.disconnect();
//    WiFi.begin(ssid, password);
//    wait30 = millis() + 30000;
//  } 
//  // Check if a client has connected..
//  WiFiClient client = server.available();
//  if (!client) {
//    return;
//  }
//   
////  Serial.print("New client: ");
////  Serial.println(client.remoteIP());
//   
//  // Espera hasta que el cliente envíe datos.
//  // while(!client.available()){ delay(1); }
//
//  /////////////////////////////////////////////////////
//  // Read the information sent by the client.
//  String req = client.readStringUntil('\r');
//  Serial.println(req);
//
//  // Make the client's request.
//  if (req.indexOf("check") != -1){
//    if (event == 2){
//      buttonState = "System is ARMED";
//    } else if (event == 3) {
//      buttonState = "System is ALARMING";
//    } else {
//      buttonState = "System is DISARMED";
//    }
//  }
//   
//  //////////////////////////////////////////////
//  //  WEB PAGE. ////////////////////////////
//  client.println("HTTP/1.1 200 OK");
//  client.println("Content-Type: text/html");
//  client.println(""); //  Important.
//  client.println("<!DOCTYPE HTML>");
//  client.println("<html>");
//  client.println("<head><meta charset=utf-8></head>");
//  client.println("<body><center><font face='Arial'>");
//  client.println("<h1>Servidor web con ESP32.</h1>");
//  client.println("<h2><font color='#009900'>Credit to example code: KIO4.COM - Juan A. Villalpando</font></h2>");
//  client.println("<br><br>");
//  client.println("<a href='on2'><button>Click to ARM security system</button></a>");
//  client.println("<a href='off2'><button>Click to DISARM security system</button></a>");
//  client.println("<a href='check'><button>Check armed state</button></a>");
//  client.println("<br><br>");
//  client.println(buttonState);
//  client.println("</font></center></body></html>");
//
////  Serial.print("Client disconnected: ");
////  Serial.println(client.remoteIP());
//  client.flush();
//  client.stop();
  

  switch (event){
    case 1:
//      if (req.indexOf("on2") != -1) { // when you press the button on the webpage
      if (CheckForButtonPress()){
        Serial.println("Button pressed - system armed\n");
        detected1 = false;
        detected2 = false;
        event = 2;
      }
      break;
    case 2:
//      Serial.println(totalAccel);
      if (totalAccel > 20 || wheel_turn()){
        // more than 2 g's total sensed by accelerometer
        // note: sensor by default includes force of gravity (aka 10 = 10 m/s^2)
        led_alarm_on();
        speaker_alarm_on();
        gps_location_start();
        Serial.println("");
        event = 3;
      }

//      if (req.indexOf("off2") != -1){ // when off button pressed
      if (CheckForButtonPress()){
        led_alarm_off();
        speaker_alarm_off();
        gps_location_stop();
        Serial.println("");
        event = 1;
      }
      break;
    case 3:
//      if (req.indexOf("off2") != -1){ // off button pressed
      if (CheckForButtonPress()){
        led_alarm_off();
        speaker_alarm_off();
        gps_location_stop();
        Serial.println("");
        event = 1;
      }    
      break;
  }
//  if (wheel_turn()){
//    Serial.println("turn");
//  }
}



bool wheel_turn()
{
  if (detected1 && detected2){
    detected1 = false;
    detected2 = false;
    return true;
  }
  return false;
}
void led_alarm_on()
{ 
  Serial.println("LEDs blinking on and off");
}
void speaker_alarm_on()
{
  Serial.println("Speaker alarming");
}
void gps_location_start()
{
  Serial.println("GPS begins transmitting location data");
}
void led_alarm_off()
{ 
  Serial.println("LED stops blinking on and off");
}
void speaker_alarm_off()
{
  Serial.println("Speaker stops alarming");
}
void gps_location_stop()
{
  Serial.println("GPS stops transmitting location data");
}

void pan_start()
{
  Serial.println("State 2->3: Begin spinning the rotating motor");
}
void pan_stop()
{
  Serial.println("State 3->2 or 3->1: Stop spinning the rotating motor");
}

bool CheckForButtonPress() {
  if (buttonIsPressed == true){
    buttonIsPressed = false;
    return true;
  }
  else {
    return false;
  }
}
