// Juan A. Villalpando.
// KIO4.COM
// Enciende y apaga LED. Botones.

#include <WiFi.h>
const char* ssid = "apogee";
const char* password = "wifinowpls";

WiFiServer server(80); // Port 80

#define LED2  13    // LED2 is a Built-in LED.
String estado = "";
int wait30 = 30000; // time to reconnect when connection is lost.
const int trigPin = 5;
const int echoPin = 4;
// defines variables
long duration;
int distance;


void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

// Connect WiFi net.
  Serial.println();
  Serial.print("Connecting with ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected with WiFi.");
 
  // Start Web Server.
  server.begin();
  Serial.println("Web Server started.");
 
  // Esta es la IP
  Serial.print("This is IP to connect to the WebServer: ");
  Serial.print("http://");
  Serial.println(WiFi.localIP());
}
 
void loop() {
// If disconnected, try to reconnect every 30 seconds.

  
  //////////////////////////////////////////////
  // ultrasonic stuff

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  if ((WiFi.status() != WL_CONNECTED) && (millis() > wait30)) {
    Serial.println("Trying to reconnect WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    wait30 = millis() + 30000;
  } 
  // Check if a client has connected..
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
   
  Serial.print("New client: ");
  Serial.println(client.remoteIP());
   
  // Espera hasta que el cliente envíe datos.
  // while(!client.available()){ delay(1); }

  /////////////////////////////////////////////////////
  // Read the information sent by the client.
  String req = client.readStringUntil('\r');
  Serial.println(req);

  // Make the client's request.

  
  
//       if (req.indexOf("on2") != -1) {digitalWrite(LED2, HIGH); estado = "ON";}
//       if (req.indexOf("off2") != -1){digitalWrite(LED2, LOW); estado = "OFF";}
//     if (req.indexOf("consulta") != -1){
//         if (digitalRead(LED2)){estado = "LED2 now is ON";}
//         else {estado = "LED2 now is OFF";}
//          }
     
  //////////////////////////////////////////////
  //  WEB PAGE. ////////////////////////////
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  Important.
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<head><meta charset=utf-8>");
  client.println("<meta http-equiv='Content-Type' content='text/html; charset=UTF-8' />");
  client.println("<meta http-equiv='refresh' content='0.25' />");
  client.println("</head>");
  client.println("<body><center><font face='Arial'>");
  client.println("<h1>ESP32 web server.</h1>");
  //Censor-beep-sound-effect
  client.println("<h2><font color='#009900'>Thanks to my man Juan A. Villalpando at KIO4.COM i literally would have failed without him</font></h2>");
  client.println("<br><br>");

  if (distance < 60){
    client.println("<body style='background-color:red;'>");
    client.println("<h1><font color='#FFFFFF'>TOO CLOSE</h2>");  
    client.println("<embed src='/html/Censor-beep-sound-effect.mp3' loop='true' autostart='true' width='2' height='0'>");
  } else {
    client.println("<body style='background-color:green;'>");
    client.println("ya good G");
  }
  
  client.println("</font></center></body></html>");

  Serial.print("Client disconnected: ");
  Serial.println(client.remoteIP());
  client.flush();
  client.stop();

}
