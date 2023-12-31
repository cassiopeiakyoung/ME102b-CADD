// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#define BTN 32
#define SPK 4
#define BIN_1 26
#define BIN_2 25
#define LED 13
#define LED_2 5
#define MOTOR_2 18

ESP32Encoder encoder;
ESP32Encoder encoder2;
// The TinyGPSPlus object
TinyGPSPlus gps;


//const char* ssid = "xtr";
//const char* password = "cracklepop!";

//WiFiServer server(80); // Port 80
Adafruit_MPU6050 mpu;

const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
int hallPin1 = 15;
int hallPin2 = 14;
int totalAccel = 0;
int wheel_revs = 0;
int event = 1;
// event 1 = disarmed, event 2 = armed, event 3 = alarming
int wait30 = 30000;
bool detected1 = false;
bool detected2 = false;
String buttonState = "";
volatile bool buttonIsPressed = false;

int omegaSpeed = 0;
int omegaDes = 7;
int omegaMax = 19;   // CHANGE THIS VALUE TO YOUR MEASURED MAXIMUM SPEED
int theta = 0;
int thetaDes = 0;
int thetaMax = 1500;
int D = 0;
int error = 0;
int dir = 1;

int Kp = 2;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
int Ki = 0.01;
int IMax = 0;
int SumError = 0;

//Setup interrupt variables ----------------------------
volatile int count = 0; // encoder count
volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

// setting PWM properties ----------------------------
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;


void hall_detect_1()
{
  detected1 = true;
}

void hall_detect_2()
{
  detected2 = true;
}


void wheel_turn()
{
  if (detected1 && detected2){
    detected1 = false;
    detected2 = false;
    wheel_revs += 1;
    Serial.println(wheel_revs);
  }
}


void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
  buttonIsPressed = true;
}

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(LED_2, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(SPK, pwmChannel);
  
  ////////////////////////////////////
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);

    // initilize timer
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 1500000, true); // 1500000 * 1 us = 1.5 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
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
  
//Serial.println(wheel_revs);

  ////////////////////////////////////////
  //motor stuff
  // READ ME
  // for some reason the direction change 
  // stuff doesnt work when this code is in 
  // the switch statement

  if (event == 3){
    if (interruptCounter) {
    portENTER_CRITICAL(&timerMux0);
    interruptCounter = false;
    portEXIT_CRITICAL(&timerMux0);
    
    totalInterrupts++;

    if (totalInterrupts%2 == 0) {
      thetaDes = 1;
    }
    if (totalInterrupts%2 == 1){
      thetaDes = thetaMax;
    }
  }
  
  if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);

      theta += count;
//      potReading = analogRead(POT); 
      

      //A6 CONTROL SECTION
      //CHANGE THIS SECTION FOR P AND PI CONTROL

      error = thetaDes - theta;
      int P = Kp * error;
      SumError += error;
      if (SumError > 100){
        SumError = 100;
      } else if (SumError < -100){
        SumError = -100;
      }

      D = Kp * (error + (SumError / 25));

      //END A6 CONTROL SECTION

      //Ensure that you don't go past the maximum possible command
      if (D > MAX_PWM_VOLTAGE) {
          D = MAX_PWM_VOLTAGE;
      }
      else if (D < -MAX_PWM_VOLTAGE) {
          D = -MAX_PWM_VOLTAGE;
      }

      //Map the D value to motor directionality
      //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
      if (D > 0) {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, D);
      }
      else if (D < 0) {
          ledcWrite(ledChannel_1, -D);
          ledcWrite(ledChannel_2, LOW);
      }
      else {
          ledcWrite(ledChannel_1, LOW);
          ledcWrite(ledChannel_2, LOW);
      }

      plotControlData();
  }
  }
  /////////////////////////////////

//  if (event == 3){
//    
//  }


  wheel_turn();

  switch (event){
    case 1:
//      if (req.indexOf("on2") != -1) { // when you press the button on the webpage
      if (CheckForButtonPress()){
        Serial.println("Button pressed - system armed\n");
        detected1 = false;
        detected2 = false;
//        Serial.println(event);
//        Serial.println("");
        digitalWrite(LED, HIGH);
        event = 2;

      }
      break;
    case 2:
//      Serial.println(totalAccel);

//      if (hall_detect_1){
//        Serial.println("hall 1 detect");
//      }
//      
//      if (hall_detect_2){
//        Serial.println("hall 2 detect");
//      }
      
      if (totalAccel > 15 || wheel_revs > 3){
        // more than 2 g's total sensed by accelerometer
        // note: sensor by default includes force of gravity (aka 10 = 10 m/s^2)
        digitalWrite(LED, LOW);
        led_alarm_on();
        speaker_alarm_on();
        gps_location_start();
//        spray_motor_on();
        wheel_revs = 0;
        spray_on();
        Serial.println("Spray on");
//        delay(1000); // remove later
        event = 3;
//        Serial.println(event);
//        Serial.println("");
      }

//      if (req.indexOf("off2") != -1){ // when off button pressed
      if (CheckForButtonPress()){
        Serial.println("Button pressed - system disarmed");
        digitalWrite(LED, LOW);
        led_alarm_off();
        speaker_alarm_off();
        spray_motor_off();
        gps_location_stop();
        spray_off();
        Serial.println("Spray off");
        Serial.println("");
        wheel_revs = 0;
        event = 1;
      }
      break;
    case 3:
//      if (req.indexOf("off2") != -1){ // off button pressed
      if (CheckForButtonPress()){
        Serial.println("Button pressed - system disarmed");
        digitalWrite(LED, LOW);
        led_alarm_off();
        speaker_alarm_off();
        gps_location_stop();
        spray_motor_off();
        Serial.println("Spray off");
        spray_off();
        wheel_revs = 0;
        Serial.println("");
        event = 1;
      }    
      break;
  }
//  if (wheel_turn()){
//    Serial.println("turn");
//  }
//  Serial.println(totalInterrupts);
}




void led_alarm_on()
{ 
  digitalWrite(LED_2, HIGH);
}
//void speaker_alarm_on()
//{
//  Serial.println("Speaker alarming");
//}
void gps_location_start()
{
  Serial.println("GPS begins transmitting location data");
}
void led_alarm_off()
{ 
  digitalWrite(LED_2, LOW);
}
//void speaker_alarm_off()
//{
//  Serial.println("Speaker stops alarming");
//}
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
void speaker_alarm_on() {
  ledcAttachPin(SPK, pwmChannel);
  ledcWriteTone(pwmChannel, 600);
}

void speaker_alarm_off() {
  ledcDetachPin(SPK);
  ledcWriteTone(pwmChannel, 0);
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

void displayInfo()
{

  Serial.print(F("Location: "));

  if (gps.location.isValid()){
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  }  
  else
  {
    Serial.print(F("INVALID"));
  }
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}
void plotControlData() {
  Serial.print("Position:");
  Serial.print(theta);
  Serial.print(" ");
  Serial.print("Desired_Position:");  
  Serial.print(thetaDes);
  Serial.print(" ");
  Serial.print("PWM_Duty:");
  Serial.println(D);
  Serial.print(" ");
}

//void spray_motor_on(){
//    if (interruptCounter) {
//      portENTER_CRITICAL(&timerMux0);
//      interruptCounter = false;
//      portEXIT_CRITICAL(&timerMux0);
//      
//      totalInterrupts++;
//  
//      if (totalInterrupts%2 == 0) {
//        dir = -1*dir;
//      }
//    }
//    if (deltaT) {
//      portENTER_CRITICAL(&timerMux1);
//      deltaT = false;
//      portEXIT_CRITICAL(&timerMux1);
//
//      omegaSpeed = count;
////      potReading = analogRead(POT); 
//      omegaDes = dir*omegaMax;
//
//      //A6 CONTROL SECTION
//      int error = omegaDes - omegaSpeed;
//      int P = Kp * error;
//      SumError += error;
//      if (SumError > 500){
//        SumError = 500;
//      } else if (SumError < -500){
//        SumError = -500;
//      }
//      
//      D = Kp * (error + (SumError / 100));
//      //CHANGE THIS SECTION FOR P AND PI CONTROL
//
////      D = map(omegaDes, -omegaMax, omegaMax, -NOM_PWM_VOLTAGE, NOM_PWM_VOLTAGE);  // REPLACE THIS LINE WITH P/PI CONTROLLER CODE
//
//      //END A6 CONTROL SECTION
//
//      //Ensure that you don't go past the maximum possible command
//      if (D > MAX_PWM_VOLTAGE) {
//          D = MAX_PWM_VOLTAGE;
//      }
//      else if (D < -MAX_PWM_VOLTAGE) {
//          D = -MAX_PWM_VOLTAGE;
//      }
//
//      //Map the D value to motor directionality
//      //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
//      if (D > 0) {
//          ledcWrite(ledChannel_1, LOW);
//          ledcWrite(ledChannel_2, D);
//      }
//      else if (D < 0) {
//          ledcWrite(ledChannel_1, -D);
//          ledcWrite(ledChannel_2, LOW);
//      }
//      else {
//          ledcWrite(ledChannel_1, LOW);
//          ledcWrite(ledChannel_2, LOW);
//      }
//      plotControlData();
//  }
//}

void spray_motor_off(){
    ledcWrite(ledChannel_1, LOW);
    ledcWrite(ledChannel_2, LOW);
}
void spray_on(){
  digitalWrite(MOTOR_2, HIGH);
}
void spray_off(){
  digitalWrite(MOTOR_2, LOW);
}
