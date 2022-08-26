#include "DHT.h"
#define DHTPIN 22
#define DHTTYPE DHT22
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#define WIFI_SSID "ZFB"
#define WIFI_PASSWORD "LifeIsBeautiful"
#define API_KEY "AIzaSyBb2RnM7yFhRmhCD11uqMtnX517PMM1MSs"
#define DATABASE_URL "wmap-b6030-default-rtdb.firebaseio.com/"
#define USER_EMAIL "zferdous35@gmail.com"
#define USER_PASSWORD "codeingismading"
#include "RTClib.h"
#include <Wire.h>
RTC_Millis rtc;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
unsigned long count = 0;
const float bucketSize = .3;
volatile unsigned long tipCount = 0;
volatile unsigned long contactTime = 0;
volatile float totalRainfall = 0;
long multiplier = 1572;
unsigned long mark = random(millis()+random(multiplier));
const byte windPin = 23;
const byte rainPin = 12;
volatile unsigned long startTime = 0;
volatile unsigned long bufferTime = 500;
unsigned long dataTimer = 0;
unsigned long inspectionTime = 60000;
volatile float pulseTime = 0;
int pulse = 0;
volatile bool start = true;
float temperature;
float humidity; 
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  Serial.begin(115200);
  rtc.begin(DateTime(__DATE__, __TIME__));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
  Serial.end();
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(5);
  dht.begin();
  pinMode(windPin, INPUT_PULLUP);
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(windPin, anemometerISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rainPin), rainGaugeISR, FALLING);
  sei();
  delay(inspectionTime);
  dataTimer = millis();
}
void anemometerISR() {
  unsigned long currentTime = millis();
  if(!start)
  {
    pulseTime = (float)(currentTime - startTime)/1000;
  }
  pulse++;
  startTime = currentTime;
  start = false;
}
void rainGaugeISR()
{
  if(millis() - contactTime > bufferTime)
  {
    tipCount++;
    contactTime = millis();
  }
}
float getAnemometerFreq(float pTime) 
{ 
  return (1/pTime); 
}
float getWindMS(float freq) 
{ 
  return (freq*0.83); 
}
void loop() {
  DateTime now = rtc.now();
  String date = String(now.day())+"/"+String(now.month())+"/"+String(now.year());
  String watch = String(now.hour())+":"+String(now.minute())+":"+String(now.second());
  String number = String(mark);
  String db_url = "/block2/"+number;
  if(pulse == 0)
  {
    pulseTime = 0;
  }
  unsigned long rTime = millis();
  if((rTime - dataTimer) >= inspectionTime)
  {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    detachInterrupt(windPin);
    detachInterrupt(rainPin);
    float aFreq = 0;
    if(pulseTime > 0.0) 
      aFreq = getAnemometerFreq(pulseTime);
    float wSpeedMS = getWindMS(aFreq);
    totalRainfall = tipCount * bucketSize;
    Serial.begin(115200);
    if (Firebase.ready())
    {
      FirebaseJson json;
      json.add("date", date);
      Serial.printf("Update node... %s\n", Firebase.updateNode(fbdo, db_url, json) ? "ok" : fbdo.errorReason().c_str());
      json.add("time", watch);
      Serial.printf("Update node... %s\n", Firebase.updateNode(fbdo, db_url, json) ? "ok" : fbdo.errorReason().c_str());
      json.add("windSpeed(mps)", wSpeedMS);
      Serial.printf("Update node... %s\n", Firebase.updateNode(fbdo, db_url, json) ? "ok" : fbdo.errorReason().c_str());
      json.add("rainfall(mm)", totalRainfall);
      Serial.printf("Update node... %s\n", Firebase.updateNode(fbdo, db_url, json) ? "ok" : fbdo.errorReason().c_str());
      json.add("temperature(C)", temperature);
      Serial.printf("Update node... %s\n", Firebase.updateNode(fbdo, db_url, json) ? "ok" : fbdo.errorReason().c_str());
      json.add("humidity", humidity);
      Serial.printf("Update node... %s\n", Firebase.updateNode(fbdo, db_url, json) ? "ok" : fbdo.errorReason().c_str());
      Serial.println();
      Serial.end();
    }
    mark = random(millis()+random(multiplier));
    start = true;
    pulse = 0;
    tipCount = 0; 
    attachInterrupt(digitalPinToInterrupt(windPin), anemometerISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rainPin), rainGaugeISR, FALLING);
    sei();
    dataTimer = millis();
  }
}
