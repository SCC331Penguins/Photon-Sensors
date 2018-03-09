// Sensors
#include "MPU9150.h"
#include "Si1132.h"
#include "Si70xx.h"
#include "math.h"
// Web Sockets
#include "application.h"
#include "Spark-Websockets.h"
#include "Particle.h"
#include "softap_http.h"
#include <string.h>

//// Initialize application variables
#define RAD_TO_DEGREES 57.2957795131
#define DEG_TO_RADIANS 0.0174533
#define PI 3.1415926535
#define ACCEL_SCALE 2 // +/- 2g

String lineToSend = "";
int configNumber = 10;
// Delays
int SLEEP_DELAY = 30000; //adds a delay after publishing so that the following publishes print correctly (ms)
long PHOTON_SLEEP = 1800; // Seconds X2
int SENSORDELAY = 500;  //// 500; //3000; // milliseconds (runs x1)
//int EVENTSDELAY = 1000; //// milliseconds (runs x10)
//int OTAUPDDELAY = 7000; //// milliseconds (runs x1)
//int SLEEP_DELAY = 0;    //// 40 seconds (runs x1) - should get about 24 hours on 2100mAH, 0 to disable and use RELAX_DELAY instead
String SLEEP_DELAY_MIN = "15"; // seconds - easier to store as string then convert to int
String SLEEP_DELAY_STATUS = "OK"; // always OK to start with
int RELAX_DELAY = 5; // seconds (runs x1) - no power impact, just idle/relaxing


// Variables for the I2C scan
byte I2CERR, I2CADR;

//// ***************************************************************************
int I2CEN = D2;
int ALGEN = D3;
int LED = D7;

int SOUND = A0;
double SOUNDV = 0; //// Volts Peak-to-Peak Level/Amplitude

int POWR1 = A1;
int POWR2 = A2;
int POWR3 = A3;
double POWR1V = 0; //Watts
double POWR2V = 0; //Watts
double POWR3V = 0; //Watts

int SOILT = A4;
double SOILTV = 0; //// Celsius: temperature (C) = Vout*41.67-40 :: Temperature (F) = Vout*75.006-40

int SOILH = A5;
double SOILHV = 0; //// Volumetric Water Content (VWC): http://www.vegetronix.com/TechInfo/How-To-Measure-VWC.phtml

bool BMP180OK = false;
double BMP180Pressure = 0;    //// hPa
double BMP180Temperature = 0; //// Celsius
double BMP180Altitude = 0;    //// Meters

bool Si7020OK = false;
double Si7020Temperature = 0; //// Celsius
double Si7020Humidity = 0;    //// %Relative Humidity

bool Si1132OK = false;
double Si1132UVIndex = 0; //// UV Index scoring is as follows: 1-2  -> Low, 3-5  -> Moderate, 6-7  -> High, 8-10 -> Very High, 11+  -> Extreme
double Si1132Visible = 0; //// Lux
double Si1132InfraRd = 0; //// Lux

MPU9150 mpu9150;
bool ACCELOK = false;
int cx, cy, cz, ax, ay, az, gx, gy, gz;
double tm; //// Celsius

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//// ***************************************************************************
// ---- Websockets ----
char* sensorBits[8]; // Sensor Bits - Active or Inactive depending on received number
WebSocketClient client;
char* routerIP;
int routerPort = 80;
int counter = 0;


// ---- Sleep/Delay ----
bool isSleeping = false;
const int maxL = 1600;
int maxLineNumber = 1600; //changed
String savedLines[maxL]; // Estimate for an hour
int currentLineNumber = 0;

// ---- Motion -----

#define LED 7       // The photon LED 7 is used to provide a visual indication
                    // of detected motion. The LED will turn ON when motion is
                    // detected and turn OFF when no motion is detected
int inputPin = D6;  // Digital Pin D6 is used to read the output of the PIR
                    // sensor. D6 goes HIGH when motion is detected and LOW when
                    // there's no motion. When the sensor goes HIGH, it waits for
                    // short period before it goes LOW
int sensorState = LOW;        // Start by assuming no motion detected
int sensorValue = 0;          // Variable for reading the inputPin (D6) status

 void onMessage(WebSocketClient client, char* message)
 {
   Serial.println("Message");
   String msg = String(message);
   if(msg.indexOf(System.deviceID()) > 0){
     if(msg.indexOf("Sleep")> 0){
       String sleepString = getValue(msg, ',', 1); //up to 3600
       String seconds = getValue(sleepString, ':', 1); //up to 3600
       isSleeping = true;
       maxLineNumber = seconds.toInt(); // id:21412r21fvBasb,sleep:3600d
     } else {
     Serial.println("\n---------------\nData Received: " + msg);
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
         delimiter[0] = ' ';
         strtok(var, delimiter);
         char* configString = strtok(NULL, delimiter);
         delimiter[0] = ':';
         strtok(configString, delimiter);
         char* configNumber = strtok(NULL, delimiter);
         Serial.println(String("Number received: ") + String(atoi(configNumber)));
         intToBit(atoi(configNumber));
         writeConfigToEEPROM(atoi(configNumber));
         break;
       }
     }
   }
 }
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
 void onConnection(const char* url, ResponseCallback* cb, void* cbArg, Reader* body, Writer* result, void* reserved)
 {
   char* data = body->fetch_as_string();
   if(String(url).compareTo(String("/wifi")) == 0  && String(data).compareTo(String("")) != 0)
   {
     Serial.begin(9600);
     Serial.println("DATA: " + String(data));
     const char delimiter[2] = "\t";
     char* ssid = strtok(data, delimiter);
     char* pass = strtok(NULL, delimiter);
     char* ip = strtok(NULL, delimiter);
     Serial.println("URL: " + String(url));
     Serial.println("SSID: " + String(ssid));
     Serial.println("Pass: " + String(pass));
     Serial.println("IP: " + String(ip));
     writeIPToEEPROM( ip);
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
  // Motion
  pinMode(D7, OUTPUT);
  pinMode(D6, INPUT);
  Serial.println("started");
  // opens serial over USB
  Serial.begin(9600);
  // Set I2C speed
  // 400Khz seems to work best with the Photon with the packaged I2C sensors
  Wire.setSpeed(CLOCK_SPEED_400KHZ);
  Wire.begin();  // Start up I2C, required for LSM303 communication
  // diables interrupts
  noInterrupts();
  // initialises the IO pins
  setPinsMode();
  // initialises MPU9150 inertial measure unit
  initialiseMPU9150();

  // Websockets
  Serial.begin(9600);
  Serial.println(String("IP: ") + String(readIPFromEEPROM()));
  routerIP = readIPFromEEPROM();
  configNumber = readConfigFromEEPROM();

  routerIP = "192.168.0.133";              // EDIT THIS
  configNumber = 255;                   // EDIT THIS

  Serial.println(String("Config Number:") + String(configNumber));

  intToBit(configNumber);
  pinMode(D7, OUTPUT);
  client.connect(routerIP,8000,&printStatement);
  client.onMessage(onMessage);
  delay(5000);
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  sensorValue = digitalRead(inputPin);  // Reads sensor output connected to pin D6

  client.monitor();
  //delay(1000);
  if(counter==0)
  {
    char deviceID[264];
    strcpy(deviceID, System.deviceID().c_str());
    //client.send(deviceID);
  }
  counter++;
    // reset
  lineToSend = "{ \"SENSORID\": \""+System.deviceID()+"\",";
  lineToSend += "\"config\": " + String(configNumber) + ",";
  // Sensors
  digitalWrite(I2CEN, HIGH);
  digitalWrite(ALGEN, HIGH);

  readWeatherSi1132();
  readWeatherSi7020();

  if(sensorBits[0] == "Active"){
    //TILT
    String printable = "\"tiltX\": " + String(getXTilt(ax, az)) + ", \"tiltY\": "+ String(getYTilt(ay,az))+",";

    Serial.println(printable);
    lineToSend += printable;
  }
  else{
    lineToSend += "\"tiltX\": null,\"tiltY\": null,";
  }

  if(sensorBits[1] == "Active"){
    //MOTION
    String reading = motionDetection();
    Serial.println("Motion:"+reading);
    lineToSend += "\"motion\": " + reading + ",";
  }else{
    lineToSend += "\"motion\": null,";
  }

  if(sensorBits[2] == "Active"){
    //IF
    Serial.println("IF Reading:");
    Serial.println(Si1132InfraRd);
    lineToSend += "\"ir\": " + String(Si1132InfraRd) + ",";
  }else{
    lineToSend += "\"ir\": null,";
  }

  if(sensorBits[3] == "Active"){
    //UV
    Serial.println("UV Reading:");
    Serial.println(Si1132UVIndex);
    lineToSend += "\"uv\": " + String(Si1132UVIndex) + ",";
  }else{
    lineToSend += "\"uv\": null,";
  }

  if(sensorBits[4] == "Active"){
    //SOUND
    Serial.println("Sound Reading:");
    float sound = readSoundLevel();
    Serial.println(sound);
    lineToSend += "\"sound\": " + String(sound) + ",";
  }else{
    lineToSend += "\"sound\": null,";
  }

  if(sensorBits[5] == "Active"){
    //HUMIDITY
    Serial.println("Humidity Reading:");
    Serial.println(Si7020Humidity);
    lineToSend += "\"humidity\": " + String(Si7020Humidity) + ",";
  }else{
    lineToSend += "\"humidity\": null,";
  }

  if(sensorBits[6] == "Active"){
    //TEMPERATURE
    Serial.println("Temperature Reading:");
    Serial.println(Si7020Temperature);
    lineToSend += "\"temp\": " + String(Si7020Temperature) + ",";
  }else{
    lineToSend += "\"temp\": null,";
  }

  if(sensorBits[7] == "Active"){
    //LIGHT
    Serial.println("Light Reading:");
    if(Si1132Visible >= 0 || Si1132Visible <= 100.0){
      Serial.println(Si1132Visible);
      lineToSend += "\"light\": " + String(Si1132Visible)+ ",";
    }
    else{
      Serial.println("Reading too high");
      lineToSend += "\"light\": null,";
    }
  }else{
    lineToSend += "\"light\": null,";
  }
  time_t time = Time.now();
  lineToSend += "\"time\": "+String(time);
  lineToSend += "}";
  // send lineToSend
  if(isSleeping && currentLineNumber < maxLineNumber){ // cant do length
    savedLines[currentLineNumber] = lineToSend;
    currentLineNumber++;
  } else if (currentLineNumber == maxLineNumber){
    isSleeping = false;
    currentLineNumber = 0;
    sendAllLines();
  }
  if(!isSleeping){
    // send lineToSend
    char charBuf[lineToSend.length()+1];
    lineToSend.toCharArray(charBuf, lineToSend.length()+1);
    Serial.println(charBuf);
    client.send(charBuf);
    Serial.println("Sent");
  }

}
void sendAllLines(){
  for (int i=0; i < maxLineNumber; i++){
      char charBuf[savedLines[i].length()+1];
      savedLines[i].toCharArray(charBuf, savedLines[i].length()+1);
      client.send(charBuf);
      Serial.println("Sent");
      delay(10);
   }
}
char* stringToChar(String line){
  char charBuf[line.length()+1];
  line.toCharArray(charBuf, line.length()+1);
  Serial.println(charBuf);
  return charBuf;
}

/// --------------------- ///////////////////////// TODO FIX //////////////////
String motionDetection(){
  //motion detection code
  sensorValue = digitalRead(inputPin);  // Reads sensor output connected to pin D6

  if (sensorValue == HIGH)              // If the input pin is HIGH turn LED ON
  {
    digitalWrite(LED, HIGH);

    if (sensorState == LOW) //Checks if sensor state has changed from its previous state
    {
      Serial.println("Motion has been detected!");    // If yes,  prints new state and
      sensorState = HIGH;                    // preserves current sensor state
    }
  }
  else
  {
    digitalWrite(LED, LOW);                  // Turns LED OFF
    if (sensorState == HIGH) //Checks if sensor state has changed from its previous state
    {
      Serial.println("No motion detected!");      // if yes, prints new state
      sensorState = LOW;                    // preserves current sensor state
    }
  }
}

void printStatement(){
  Serial.println("CALLBACK-------");
  counter=0;
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

// Write to Memory, addr should be 0
void writeIPToEEPROM(char* IP)
{
  int addr = 0;
  // You can get and put simple values like int, long, bool, etc. using get and put directly
  for(int i = 0; IP[i] != '\0'; i++)
  {
    EEPROM.put(addr, IP[i]);
    addr += sizeof(char); //keep writing
  }
  char end =  '\0';
  EEPROM.put(addr,  end);
}

// Read From memory
char* readIPFromEEPROM()
{
  int addr = 0;
  char ip[16];
  char ch = 'a';
  // You can get and put simple values like int, long, bool, etc. using get and put directly
  while(ch != '\0')
  {
    EEPROM.get(addr, ch);
    ip[addr] = ch;
    addr += sizeof(char);
  }
  ip[addr]= '\0';
  char *ret = (char*) malloc(16);
  for(int i = 0; i < addr; ++i)
        ret[i] =ip[i];
  return ret;
}

void writeConfigToEEPROM(int configNumber)
{
  int addr = 18;
  EEPROM.put(addr, configNumber);
}

int readConfigFromEEPROM()
{
  int configNumber;
  int addr = 18;
  EEPROM.get(addr, configNumber);
  return configNumber;
}

// --- Set UP ---

void setPinsMode()
{
    pinMode(I2CEN, OUTPUT);
    pinMode(ALGEN, OUTPUT);
    pinMode(LED, OUTPUT);

    pinMode(SOUND, INPUT);

    pinMode(POWR1, INPUT);
    pinMode(POWR2, INPUT);
    pinMode(POWR3, INPUT);

    pinMode(SOILT, INPUT);
    pinMode(SOILH, INPUT);
}

void initialiseMPU9150()
{
  ACCELOK = mpu9150.begin(mpu9150._addr_motion); // Initialize MPU9150

  if (ACCELOK)
  {
      // Clear the 'sleep' bit to start the sensor.
      mpu9150.writeSensor(mpu9150._addr_motion, MPU9150_PWR_MGMT_1, 0);

      /// Set up compass
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x0F); //SelfTest
      mpu9150.writeSensor(mpu9150._addr_compass, 0x0A, 0x00); //PowerDownMode

      mpu9150.writeSensor(mpu9150._addr_motion, 0x24, 0x40); //Wait for Data at Slave0
      mpu9150.writeSensor(mpu9150._addr_motion, 0x25, 0x8C); //Set i2c address at slave0 at 0x0C
      mpu9150.writeSensor(mpu9150._addr_motion, 0x26, 0x02); //Set where reading at slave 0 starts
      mpu9150.writeSensor(mpu9150._addr_motion, 0x27, 0x88); //set offset at start reading and enable
      mpu9150.writeSensor(mpu9150._addr_motion, 0x28, 0x0C); //set i2c address at slv1 at 0x0C
      mpu9150.writeSensor(mpu9150._addr_motion, 0x29, 0x0A); //Set where reading at slave 1 starts
      mpu9150.writeSensor(mpu9150._addr_motion, 0x2A, 0x81); //Enable at set length to 1
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //overvride register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x67, 0x03); //set delay rate
      mpu9150.writeSensor(mpu9150._addr_motion, 0x01, 0x80);

      mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x04); //set i2c slv4 delay
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x00); //override register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x00); //clear usr setting
      mpu9150.writeSensor(mpu9150._addr_motion, 0x64, 0x01); //override register
      mpu9150.writeSensor(mpu9150._addr_motion, 0x6A, 0x20); //enable master i2c mode
      mpu9150.writeSensor(mpu9150._addr_motion, 0x34, 0x13); //disable slv4
    }
    else
    {
      Serial.println("Unable to start MPU5150");
    }
}

// --------- Sensors ---------

  void readMPU9150()
  {
      //// reads the MPU9150 sensor values. Values are read in order of temperature,
      //// compass, Gyro,!studio?x57 Accelerometer

      tm = ( (double) mpu9150.readSensor(mpu9150._addr_motion, MPU9150_TEMP_OUT_L, MPU9150_TEMP_OUT_H) + 12412.0 ) / 340.0;
      cx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);  //Compass_X
      cy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);  //Compass_Y
      cz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);  //Compass_Z
      ax = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
      ay = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
      az = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
      gx = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
      gy = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
      gz = mpu9150.readSensor(mpu9150._addr_motion, MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
  }


  //// returns air temperature and humidity
  int readWeatherSi7020()
  {
      Si70xx si7020;
      Si7020OK = si7020.begin(); //// initialises Si7020

      if (Si7020OK)
      {
          Si7020Temperature = si7020.readTemperature();
          Si7020Humidity = si7020.readHumidity();
      }

      return Si7020OK ? 2 : 0;
  }


  //// returns measurements of UV, InfraRed and Ambient light
  int readWeatherSi1132()
  {
      Si1132 si1132;
      Si1132OK = si1132.begin(); //// initialises Si1132

      if (Si1132OK)
      {
          Si1132UVIndex = si1132.readUV() / 100;
          Si1132Visible = si1132.readVisible();  ////0 to 1000 Lux
          Si1132InfraRd = si1132.readIR();
      }

      return Si1132OK ? 3 : 0;
  }

  //// returns accelaration along x-axis, should be 0-1g
  float getAccelX(float x)
  {
    return x/pow(2,15)*ACCEL_SCALE;
  }

  //// returns accelaration along z-axis, should be 0-1g
  float getAccelY(float y)
  {
    return y/pow(2,15)*ACCEL_SCALE;
  }

  //// returns accelaration along z-axis should be 0-1g
  float getAccelZ(float z)
  {
    return z/pow(2,15)*ACCEL_SCALE;
  }

  //// returns the vector sum of the acceleration along x, y and z axes
  //// in g units
  float getAccelXYZ(float x, float y, float z)
  {
    x = getAccelX(x);
    y = getAccelY(y);
    z = getAccelZ(z);

    return sqrt(x*x+y*y+z*z);
  }

  //// returns tilt along x axis in radians - uses accelerometer
  float getXTilt(float accelX, float accelZ)
  {
     float tilt = atan2(accelX,accelZ)*RAD_TO_DEGREES; //*RAD_TO_DEGREES;
     if(tilt < 0)
     {
        tilt = tilt+360.0;
     }

     return tilt;
  }

  //// returns tilt along y-axis in radians
  float getYTilt(float accelY, float accelZ)
  {
     float tilt = atan2(accelY,accelZ)*RAD_TO_DEGREES;
     if(tilt < 0)
     {
       tilt = tilt+360.0;
     }

     return tilt;
  }

  //returns sound level measurement in as voltage values (0 to 3.3v)
  float readSoundLevel()
  {
      unsigned int sampleWindow = 50; // Sample window width in milliseconds (50 milliseconds = 20Hz)
      unsigned long endWindow = millis() + sampleWindow;  // End of sample window

      unsigned int signalSample = 0;
      unsigned int signalMin = 4095; // Minimum is the lowest signal below which we assume silence
      unsigned int signalMax = 0; // Maximum signal starts out the same as the Minimum signal

      // collect data for milliseconds equal to sampleWindow
      while (millis() < endWindow)
      {
          signalSample = analogRead(SOUND);
          if (signalSample > signalMax)
          {
              signalMax = signalSample;  // save just the max levels
          }
          else if (signalSample < signalMin)
          {
              signalMin = signalSample;  // save just the min levels
          }
      }

      //SOUNDV = signalMax - signalMin;  // max - min = peak-peak amplitude
      SOUNDV = mapFloat((signalMax - signalMin), 0, 4095, 0, 3.3);

      //return 1;
      return SOUNDV;
  }
