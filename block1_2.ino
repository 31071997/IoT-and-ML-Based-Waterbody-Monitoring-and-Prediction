#include <WiFiClient.h>
#include <WebServer.h>*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <DFRobot_ESP_EC.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#define ONE_WIRE_BUS 13                
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire); 
DFRobot_ESP_EC ec;
Adafruit_ADS1115 ads;
TinyGPSPlus gps;
int trig = 13;
int echo = 26;
int rx = 16;
int tx = 17;
int samples = 10;
int flowsensor = 27;
int temperaturePin = 21;
float adc_resolution = 4096.0;
int pHSense = 25;
//int turbidityPin = 33;
int interval = 60000;
volatile int flow_frequency; // Measures flow sensor pulses
float l_minute;
float phValue;
unsigned long currentTime;
unsigned long cloopTime;
float temperature;
float duration;
int turbidity;
float distance;
String Location;
String lat1;
String lat2;
String lng1;
String lng2;
String lattitude;
String lngitude;
HardwareSerial GPS_Serial(1);

void flow () // Interrupt function
{
   flow_frequency++;
}

void setup() {
  Serial.begin(115200);
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  GPS_Serial.begin(9600, SERIAL_8N1, rx, tx);
  attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING);
  //sei();
  currentTime = millis();
  cloopTime = currentTime; 
}

void GPSinfo()
{
  if(GPS_Serial.available())
  {
    gps.encode(GPS_Serial.read()); 
  }
    
    /*Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print(", ");
    
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.print(gps.time.second());
    Serial.print(", ");
    Serial.begin(115200);
    Serial.print("lattitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("lngitude: ");
    Serial.println(gps.location.lng(), 6);*/
    Serial.print("satellite value: ");
    Serial.println(gps.satellites.value());
    /*Serial.end();
    
    String Location = String(gps.location.lat(), 6)+", "+String(gps.location.lng(), 6);
    Serial.print(", ");
   
    Serial.print(gps.location.lng(), 6);
    Serial.print(", ");
    String Location = String(gps.location.lat(), 6)+ String(gps.location.lng(), 6);
    return gps.location.lat();*/
    lattitude = String(gps.location.lat(), 6);
    lngitude = String(gps.location.lng(), 6);
}

void loop() {
  // put your main code here, to run repeatedly:
  depth();
  GPSinfo();
  flowRate();
  Cph();          
  //Serial.begin(115200);
  Serial.print("depth: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.print("temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("waterflow: ");
  Serial.print(l_minute, DEC); // Print litres/hour
  Serial.println(" L/Sec");
  Serial.print("pH= ");
  Serial.println(phValue);
  Serial.print("lattitude: ");
  Serial.println(lattitude);
  Serial.print("lngitude: ");
  Serial.println(lngitude, 6);
  /*Serial.print("turbidity: ");
  Serial.print(turbidity);*/
  //Serial.end();
  delay(100);
}

float depth(){
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = 0.017 * duration;
  if(distance < 2)
  {
    distance = 0; 
  }
  else if(distance > 400)
  {
    distance = 999999;
  }
  return distance; 
}

/*int turbidityFunc() {
  int val = analogRead(turbidityPin);
  turbidity = map(val, 0, 2800, 100, 1); 
    //text = (String)turbidity;
  Serial.print("turbidity: ");
  Serial.println(turbidity);
    //delay(1000);
  //Make the ESP32 always handle web clients
  //server.handleClient();
  return turbidity;
}*/

/*float GPSinfolng()
{
  if(GPS_Serial.available())
  {
    gps.encode(GPS_Serial.read()); 
  }
    
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print(", ");
    
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.print(gps.time.second());
    Serial.print(", ");
    Serial.begin(115200);
    Serial.print("lattitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("lngitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("satellite value: ");
    Serial.println(gps.satellites.value());
    Serial.end();
    String Location = String(gps.location.lat(), 6)+", "+String(gps.location.lng(), 6);
    Serial.print(", ");
   
    Serial.print(gps.location.lng(), 6);
    Serial.print(", ");
    String Location = String(gps.location.lat(), 6)+ String(gps.location.lng(), 6);
    return gps.location.lng();
}*/

void tempValue()
{
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0); 
}

void flowRate ()
{
  currentTime = millis();
  // Every second, calculate and print litres/hour
  if(currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    if(flow_frequency != 0){
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
    l_minute = (flow_frequency / 7.5); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
    l_minute = l_minute/60;
    //vol = vol +l_minute;
    flow_frequency = 0; // Reset Counter
  }
  else {
    l_minute = 0;
  }
 }
}

float ph (float voltage) {
  return 7 + ((2.5 - voltage) / 0.18);
}

void Cph () {
  int measurings=0;
  for (int i = 0; i < samples; i++)
  {
    measurings += analogRead(pHSense);
    delay(10);
  }
  float voltage = 3.3 / adc_resolution * measurings/samples;
  //Serial.print("pH= ");
  phValue = ph(voltage);
}
