 #include "application.h"
 #include "Spark-Websockets.h"
 #include "Particle.h"
 #include "softap_http.h"
 #include <string.h>

 WebSocketClient client;
 String routerIP = "192.168.0.15";
 int routerPort = 80;

 void onMessage(WebSocketClient client, char* message) {
   Serial.println("Data Arrived: " + String(message));
 }


 void onConnection(const char* url, ResponseCallback* cb, void* cbArg, Reader* body, Writer* result, void* reserved)
 {
   Serial.begin(9600);
   char* data = body->fetch_as_string();
   Serial.println("DATA: " + String(data));
   const char delimiter[2] = "\t";
   char* ssid = strtok(data, delimiter);
   char* pass = strtok(NULL, delimiter);
   char* ip = strtok(NULL, delimiter);
   routerIP = String(ip);
   Serial.println("URL: " + String(url));
   Serial.println("SSID: " + String(ssid));
   Serial.println("Pass: " + String(pass));
   Serial.println("IP: " + String(ip));
   if(String(url).compareTo(String("/wifi")) == 0  && String(data).compareTo(String("")) != 0)
   {
     cb(cbArg, 0, 200, "text/html", nullptr);
     Serial.println("\nmatch");
     WiFi.clearCredentials();
     WiFi.setCredentials(ssid, pass);
     WiFi.listen(false);
   }
   else
   {
     cb(cbArg, 0, 404, "text/html", nullptr);
   }
 }
 STARTUP(softap_set_application_page_handler(onConnection, nullptr));

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.
  pinMode(D7, OUTPUT);
  Serial.begin(9600);
  client.connect(routerIP.c_str(),routerPort);
  client.onMessage(onMessage);
  client.send("Hello World!");
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  client.monitor();
  delay(50);
    if(client.connected())
        Serial.println("Connected!");
    else
        Serial.println("Ah fuck!");
  delay(5000);
}
