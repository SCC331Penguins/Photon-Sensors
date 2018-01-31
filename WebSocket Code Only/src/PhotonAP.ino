 #include "application.h"
 #include "Spark-Websockets.h"
 #include "Particle.h"
 #include "softap_http.h"
 #include <string.h>

 char* sensorBits[8]; // Sensor Bits - Active or Inactive depending on received number
 WebSocketClient client;
 String routerIP = "192.168.0.15";
 int routerPort = 80;
 int counter = 0;

 void onMessage(WebSocketClient client, char* message)
 {
   Serial.println("\n---------------\nData Received: " + String(message));
   Serial.println("Looking for my ID...");
   char delimiter[2] = ",";
   char* var;
   while(true)
   {
     char* var = strtok(message, delimiter);
     if (strstr(var, System.deviceID().c_str()) != NULL)
     {
       Serial.println("Found ID match: ");
       Serial.println(var);
       delimiter[0] =' ';
       strtok(var, delimiter);
       char* configString = strtok(NULL, delimiter);
       delimiter[0] = ':';
       strtok(configString, delimiter);
       char* configNumber = strtok(NULL, delimiter);
       Serial.println(String("Number received: ") + String(atoi(configNumber)));
       intToBit(atoi(configNumber));
       break;
     }
  }
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
void setup()
{
  pinMode(D7, OUTPUT);
  Serial.begin(9600);
  client.connect(routerIP.c_str(),routerPort);
  client.onMessage(onMessage);
  delay(5000);
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  client.monitor();
  delay(5000);
  if(counter==0)
  {
    char deviceID[264];
    strcpy(deviceID, System.deviceID().c_str());
    client.send(deviceID);
  }
  counter++;
}


// Saves to sensorBits each number is a different sensor
void intToBit (int number){
  int bitsPosition[8] = { 128, 64, 32, 16, 8, 4, 2, 1 };
  for (int i = 0; i < 8; i++) {
    int result = bitsPosition[i] & number;
     if( result == bitsPosition[i]){
       sensorBits[i] = "Active";
       Serial.println(String("Bit at position ") + String(i) + String(" ") + String(sensorBits[i]));
     }
     else {
       sensorBits[i] = "Inactive";
       Serial.println(String("Bit at position ") + String(i) + String(" ") + String(sensorBits[i]));
     }
   }
}
