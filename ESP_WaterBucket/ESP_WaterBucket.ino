/*
 * ESP Waterbucket
 * Optional Hardware setup: 
 *  - Embed an ESP8266 (or similar) into a generic solar charge controller such as "Renogy Wanderer 10 Amp 12V/24V" 
 *  - Find and solder a wire to the 5v or 3.3v rail inside your charge controller. 
 *      Hint: This can be found by probing the charge controller's own MCU or other known IC's VCC pin.
 *  - Find and solder a 1kR resistor to the signal pin of the charge controller's own Enter button. This will toggle the load.
 *  - Find and solder any appropriate resistor divider between the battery pin of the charge controller, ground, and A0.
 *      Hint: you will need to tweak values in ADC() depending on nominal system voltage and chosen ESP. I used  10kR and 330kR.
 *      !Hint: a 500kR trimmer potentiometer will also suffice and allow for some easy calibration. Just be sure to set it before power on.
 *      !!Hint: account for Voc (max open circuit voltage) since most LiFe batteries will go open circuit when fully charged.
 *  - Find and solder a wire to the gate of the load MOSSFET inside your charge controller.
 *  
 *  ESP will host an AP upon first boot. Connect to it and tell it your WiFi name/key.
 *
 *  Don't forget to "ESP8266 Sketch Data Upload" the html files in /data to flash.
 */

// Import required libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <FS.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

////////////////////////////////Setup////////////////////////////////
const char* http_username = "user"; //set your own web UI user and pasword for control access (this is not the wifi)
const char* http_password = "password"; //set your own web UI user and pasword (this is not the wifi)

#define MAX_ON_TIME 1000*60*61 //1 hour 1 minute
#define MIN_VOLTAGE 11.5

//if using the gen2 renogy mod.
//load detect is inverted, and voltage divider changed
#define GEN2

// set pin numbers
const int loadtogglepin = 14;     // the number of the pushbutton pin
const int loadreadpin =  12;       // the number of the LED pin

#define HISTORY_INTERVAL 60*5 //60*5 //sec
#define HISTORY_LENGTH 288 //288 every 5 mins covers 24 hours

////////////////////////////////Internal Use////////////////////////////////
Adafruit_BME280 bme; // I2C
AsyncWebServer server(80);
DNSServer dns;

float historicADC[HISTORY_LENGTH];
float historicTemp[HISTORY_LENGTH];
float historicRH[HISTORY_LENGTH];
float historicPressure[HISTORY_LENGTH];
unsigned long lastHistRec = HISTORY_INTERVAL * 1000;

// Define NTP Client to get time
const long utcOffsetInSeconds = 0;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

int autoAtTime = 0;
int autoForTime = 0;
int curtime = 0;

bool runOnce = true;

////////////////////////////////Get Values////////////////////////////////
//BME
String readBME280Temperature() {
  // Read temperature as Celsius (the default)
  float t = bme.readTemperature();
  // Convert temperature to Fahrenheit
  //t = 1.8 * t + 32;
  if (isnan(t)) {
    //Serial.println("BME280 fail");
    return "";
  }
  else {
    //Serial.println(t);
    return String(t);
  }
}

//BME
String readBME280Humidity() {
  float h = bme.readHumidity();
  if (isnan(h)) {
    //Serial.println("BME280 fail");
    return "";
  }
  else {
    //Serial.println(h);
    return String(h);
  }
}

//BME
String readBME280Pressure() {
  float p = bme.readPressure() / 100.0F;
  if (isnan(p)) {
    //Serial.println("BME280 fail");
    return "";
  }
  else {
    //Serial.println(p);
    return String(p);
  }
}

//Voltage
String ADC() {
  float l = 0;
  for (int i = 0; i < 25; i++) {
    l = l + analogRead(A0);
  }
  l = l / 25.0;
#ifdef GEN2
  float voltage = l * (26.66 / 1023.0); //gen 2 renogy (~10k / 330k)
#else
  float voltage = l * (28.15 / 1023.0);// gen1
#endif
  //Serial.println(voltage);
  return String(voltage);
}

//Load state
String getLoad() {
#ifdef GEN2
  if (digitalRead(loadreadpin) == HIGH) { //its on
    //Serial.println("ON");
    return String("ON");
  }
  if (digitalRead(loadreadpin) == LOW) { //its off
    //Serial.println("OFF");
    return String("OFF");
  }
#else
  if (digitalRead(loadreadpin) == LOW) { //its on
    //Serial.println("ON");
    return String("ON");
  }
  if (digitalRead(loadreadpin) == HIGH) { //its off
    //Serial.println("OFF");
    return String("OFF");
  }
#endif
}

//Scheduling
String getAutoAt() {
  //Serial.println(autoAtTime);
  return String(autoAtTime);
}
String getAutoFor() {
  //Serial.println(autoForTime);
  return String(autoForTime);
}
String getTime() {
  //Serial.println(curtime);
  return String(curtime);
}

//debug
String getQuality() {
  if (WiFi.status() != WL_CONNECTED)
    return String(-1);
  int dBm = WiFi.RSSI();
  if (dBm <= -100)
    return String(0);
  if (dBm >= -50)
    return String(100);
  int rssi = 2 * (dBm + 100);
  return String(rssi) + String("% (dBm:") + String(dBm) + String(")");
  //return String(ESP.getFreeHeap());
}

//get everything as one string
String getAll() {
  String CSV_String = readBME280Temperature();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + readBME280Humidity();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + readBME280Pressure();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + ADC();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + getLoad();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + getAutoAt();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + getAutoFor();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + getTime();
  CSV_String = CSV_String + String(",");
  CSV_String = CSV_String + getQuality();
  CSV_String = CSV_String + String(",");
  return CSV_String;
}

////////////////////////////////Safety////////////////////////////////
unsigned long lastOKVoltTime;
void checkLowVolts() {
  float curVolts = ADC().toFloat();
  if (curVolts <= MIN_VOLTAGE) {
    if ( millis() - lastOKVoltTime > 5000 ) {
      pressIFOn();
    }
  }
  else {
    lastOKVoltTime = millis();
  }
}

unsigned long lastOffTime = 0;
void safeTTimer() {
  // auto-off safety timer watch
  if (getLoad() == "ON") {
    if (millis() - lastOffTime > MAX_ON_TIME) {
      pressDown();
      lastOffTime = millis();
    }
  }
  else { //output is off
    lastOffTime = millis();
  }
}

////////////////////////////////Load Managment////////////////////////////////
unsigned long buttonDepressTime = 0;
unsigned long autoEnabledTime = 0;
unsigned long mobius = 0;

void pressIFOn() {
  if (getLoad() == "ON") {
    pressDown();
  }
}
void pressIFOff() {
  if (getLoad() == "OFF") {
    pressDown();
  }
}

void pressDown() {
  buttonDepressTime = millis();
  pinMode(loadtogglepin, OUTPUT); //D5 charger button
#ifdef GEN2
  digitalWrite(loadtogglepin, LOW);
#else
  digitalWrite(loadtogglepin, HIGH);
#endif
}
void holdDown() {
#ifdef GEN2
  digitalWrite(loadtogglepin, LOW);
#else
  digitalWrite(loadtogglepin, HIGH);
#endif
}


////////////////////////////////Load Control////////////////////////////////
String toggleLoad() {
  pressDown();
  //Serial.println("OK");
  return String("OK");
}

String setAuto() {
  eeWriteInt(0, autoAtTime);
  eeWriteInt(4, autoForTime);
  //Serial.println("OK");
  return String("OK");
}


////////////////////////////////Scheduling Managment////////////////////////////////
const char* PARAM_INPUT_1 = "at";
const char* PARAM_INPUT_2 = "for";
bool weWereInAuto = false;

void eeWriteInt(int pos, int val) {
  byte* p = (byte*) &val;
  EEPROM.write(pos, *p);
  EEPROM.write(pos + 1, *(p + 1));
  EEPROM.write(pos + 2, *(p + 2));
  EEPROM.write(pos + 3, *(p + 3));
  EEPROM.commit();
}
int eeGetInt(int pos) {
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  *(p + 1)  = EEPROM.read(pos + 1);
  *(p + 2)  = EEPROM.read(pos + 2);
  *(p + 3)  = EEPROM.read(pos + 3);
  return val;
}

////////////////////////////////SETUP////////////////////////////////
void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);

  EEPROM.begin(512);  //Initialize EEPROM

  pinMode(loadtogglepin, OUTPUT); //D5 charger button
  pinMode(loadreadpin, INPUT); //D6 load state input

  autoAtTime = eeGetInt(0);
  autoForTime = eeGetInt(4);

  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x77);
  if (!status) {
    Serial.println("Could not find BME280");
    //delay(5000);
    //ESP.restart();
    //while (1);
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS err");
    return;
  }

  timeClient.begin();

  //WiFiManager
  AsyncWiFiManager wifiManager(&server, &dns);
  wifiManager.autoConnect("ESP-WaterBucket-Disconnected");
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  ////////////////////////////////////////////Web Requests///////////////////////////////////////////////
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html");
  });    
  server.on("/simp", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/simp.html");
  });
  server.on("/dark", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/dark.html");
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/favicon.png", "image/png");
  });

  // get and return all in a csv format
  server.on("/getall", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getAll().c_str());
  });

  // get and return history in a csv format
  server.on("/getpast", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->addHeader("Server", "ESP Async Web Server");

    //begin building history message
    response->printf("%s,", String(HISTORY_LENGTH).c_str());
    response->printf("%s,", String(HISTORY_INTERVAL).c_str());
    response->print('\n');

    //append ADC history array
    for (int i = 0; i <= HISTORY_LENGTH - 1; i++) {
      response->printf("%s,", String(historicADC[i]).c_str());
    }
    response->print('\n');

    //append Temp history array
    for (int i = 0; i <= HISTORY_LENGTH - 1; i++) {
      response->printf("%s,", String(historicTemp[i]).c_str());
    }
    response->print('\n');

    //append RH history array
    for (int i = 0; i <= HISTORY_LENGTH - 1; i++) {
      response->printf("%s,", String(historicRH[i]).c_str());
    }
    response->print('\n');

    //append Pressure history array
    for (int i = 0; i <= HISTORY_LENGTH - 1; i++) {
      response->printf("%s,", String(historicPressure[i]).c_str());
    }
    response->print('\n');

    request->send(response);
  });

  //piecemeal requests for legacy
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBME280Temperature().c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBME280Humidity().c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBME280Pressure().c_str());
  });
  server.on("/adc", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", ADC().c_str());
  });
  /*server.on("/toggleload", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", toggleLoad().c_str());
  });*/
  server.on("/getload", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getLoad().c_str());
  });
  server.on("/getrssi", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getQuality().c_str());
  });
  server.on("/getautofor", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getAutoFor().c_str());
  });
  server.on("/getautoat", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getAutoAt().c_str());
  });
  server.on("/gettime", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getTime().c_str());
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/auto", HTTP_GET, [] (AsyncWebServerRequest * request) {
    if(!request->authenticate(http_username, http_password))
        return request->requestAuthentication();
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
    }

    if (inputParam == PARAM_INPUT_1) {
      autoAtTime = inputMessage.toInt();
    }
    if (inputParam == PARAM_INPUT_2) {
      autoForTime = inputMessage.toInt();
    }

    if (autoForTime < 0) autoForTime = 0;
    if (autoForTime > 60) autoForTime = 60;

    if (autoAtTime > 2359 || autoAtTime < 0) autoAtTime = 0;

    //  Serial.println(inputMessage);
    request->send(200, "text/html", setAuto());
  });

  // HTTP basic authentication
  server.on("/toggleload", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate(http_username, http_password))
        return request->requestAuthentication();
    //request->send(200, "text/plain", "Login Success!");
    request->send_P(200, "text/plain", toggleLoad().c_str());
  });



  ArduinoOTA.onError([](ota_error_t error) {
    (void)error;
    ESP.restart();
  });

  /* setup the OTA server */
  ArduinoOTA.begin();

  // Start server
  server.begin();
}

////////////////////////////////LOOOP////////////////////////////////
void loop() {
  ArduinoOTA.handle();

  //master button release
  if (millis() - buttonDepressTime >= 250) {
#ifdef GEN2
    digitalWrite(loadtogglepin, HIGH);
#else
    digitalWrite(loadtogglepin, LOW);
#endif
    if (millis() - buttonDepressTime >= 1000) {
      pinMode(loadtogglepin, INPUT); //change to input so human or dragon can press button
    }
  }
  else {
    holdDown();
  }

  //turn off load at astartup if its on for some reason
  if (millis() >= 5000 && runOnce) {
    runOnce = false;
    pressIFOn();
  }


  //machine loop
  if (millis() - mobius >= 1000) {
    mobius = millis();
    safeTTimer();
    checkLowVolts();

    //if (WiFi.status() != WL_CONNECTED) return;

    timeClient.update();

    int lehours = timeClient.getHours();
    int lemins = timeClient.getMinutes();
    char letime[5];
    sprintf(letime, "%02d%02d", lehours, lemins);
    curtime = atoi(letime);

    if (autoForTime > 0) {
      if (curtime == autoAtTime && weWereInAuto == false) {
        autoEnabledTime = millis();
        weWereInAuto = true;
        pressIFOff();
      }
    }

    if (millis() - autoEnabledTime >= autoForTime * 60 * 1000 && weWereInAuto == true) {
      weWereInAuto = false;
      pressIFOn();
    }
  }

  if (millis() - lastHistRec > HISTORY_INTERVAL * 1000) {
    lastHistRec = millis();
    //shift the array over by one, discard the oldest one
    for (int i = HISTORY_LENGTH - 1; i > 0; i--) {
      historicADC[i] = historicADC[i - 1];
      historicTemp[i] = historicTemp[i - 1];
      historicRH[i] = historicRH[i - 1];
      historicPressure[i] = historicPressure[i - 1];
    }

    //add a new measurement to the front of each array
    historicADC[0] = ADC().toFloat();
    historicTemp[0] = readBME280Temperature().toFloat();
    historicRH[0] = readBME280Humidity().toFloat();
    historicPressure[0] = readBME280Pressure().toFloat();

  }
  //restart a day keeps the
  /*
    #define oneDay 24*60*60*1000
    if (millis() > oneDay && digitalRead(loadreadpin) == HIGH ) {
    ESP.restart();
    }
  */

  //delay(1); //https://github.com/espressif/arduino-esp32/issues/4348
}
