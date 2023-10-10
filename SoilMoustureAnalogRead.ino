#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <PubSubClient.h>
//#include <Wire.h>
#include "credentials.h"

#define DateTimeOption//for getting time
#define MQTT_SERIAL_PUBLISH_CH "/icircuit/ESP32/serialdata/tx"
#define MQTT_SERIAL_RECEIVER_CH "/icircuit/ESP32/serialdata/rx"
#define mqtt_port 1883  // default
#define moisture_topic "sensor1/moisture"
#define converted_moisture_topic "sensor1/moisture_conv"
#define water_count_topic "sensor1/moisture_water_count"
#define wifi_rssi "sensor1/wifi_rssi1"

#define readCount 4 //the average counter, comment for single read if disabled, reads once.
// #define POWER 13 // for sensor power management, POWER is the power pin
#ifdef DateTimeOption  // timestamps in the serial output, for debug reasons.

#include "WiFiUdp.h"
#include "NTPClient.h"
#endif  

#ifdef DateTimeOption
const long utcOffsetInSeconds = 3600;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
#endif

/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/
int soil_pin = A0;    // select the input pin for the moisture sensor
//int ledPin = 13;      // select the pin for the LED
float rawSensorValue = 0;
float sensorValue=0;  // variable to store the value coming from the sensor
float slope = 2.48; // slope from linear fit
float intercept = -0.72; // intercept from linear fit

//float battVcc;


WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTT_USER,MQTT_PASSWORD)) {
      Serial.println("connected");
      Serial.print("ClientID: ");
      Serial.println(clientId);

      //Once connected, publish an announcement...
      client.publish("/icircuit/presence/ESP32/", "hello world");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_CH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte *payload, unsigned int length) {
    Serial.println("-------new message from broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:"); 
    Serial.write(payload, length);
    Serial.println();
}
# ifdef POWER
void powerOnPeripherals ()
  {
  Serial.println();Serial.print("Power on digital pin");Serial.println(POWER);
  digitalWrite (POWER, LOW);  // turn power ON
  delay (10); // give time to power up  
  Serial.println();Serial.print("Done Power on digital pin");Serial.println(POWER);
  }  // end of powerOnPeripherals

  void powerOffPeripherals ()
  {

  // put all digital pins into low stats
    Serial.println();Serial.print("Put all pins into low stats, including pin#");Serial.println(POWER);

  for (byte pin = 0; pin < 14; pin++)
    {
    digitalWrite (pin, LOW);
    pinMode (pin, OUTPUT);
    }
  Serial.println();Serial.print("Done Power off , including  digital pin");Serial.println(POWER);

  // except the power pin which has to be high
  // digitalWrite (POWER, HIGH);  // turn power OFF
  // and activation switch
  // pinMode (SWITCH, INPUT_PULLUP);
  }  // end of powerOffPeripherals
  
 #endif 
void setup() {
   Serial.begin(9600);
   Serial.setTimeout(2000);
   while(!Serial) { }
   Serial.println();

   Serial.println("Capacitive Moisture Sensor Control");
   //Serial.print("ledPin is: "); Serial.println(ledPin);
   Serial.setTimeout(500);// Set time out for 
   //pinMode(13,OUTPUT);

   //setup the wifi
   setup_wifi();
   #ifdef POWER
   int pwr = POWER;
   pinMode(pwr,OUTPUT);  
   #endif
   //pinMode(soil_pin, INPUT_PULLUP); //declare input pin
   //  pinMode(batt_pin, INPUT_PULLUP); //declare input pin
   //   analogReference(EXTERNAL); // set the analog reference to 3.3V
   
   //pinMode(ledPin, OUTPUT);
 
  
   #ifdef RTC
   timeClient.begin();
   #endif
   client.setServer(mqtt_server, mqtt_port);
   client.setCallback(callback);
   
   reconnect();
   
}


int counter = 0;
long lastMsg = 0;

void loop() {
  
    if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 10000) {
  lastMsg = now;
  float vol_water_cont;  
  // read the value from the sensor:
  
 #ifdef POWER
 powerOnPeripherals ();
 Serial.println(POWER);
 #endif
 
 #ifndef POWER
 int pw=13;
 Serial.println();Serial.print("Power on digital pin: ");Serial.println(pw);
 digitalWrite (pwr, HIGH);  // turn power ON
 #endif
  
   Serial.println();Serial.println("Digital pin activated. ");//

 #ifdef readCount
 int datanum = readCount;
 Serial.print("number of reads:  ");Serial.println (datanum);
 Serial.println("Calculating... ");
 for(int i=0; i<datanum; i++)
  {
    //delay(1000);
    Serial.println("Getting data from analog pin: ");Serial.print(soil_pin);Serial.print(" ; "); Serial.println(i);
    delay(10);
    //rawSensorValue=0;
    rawSensorValue = rawSensorValue + analogRead(soil_pin);
    //Serial.print(analogRead(soil_pin)); Serial.println();
    Serial.print("Sum:"); Serial.print(rawSensorValue);     Serial.print(","); 
   //delay(1000);
   // Serial.println(".");
    }
  Serial.println();
  Serial.println("average:");
    rawSensorValue = rawSensorValue / readCount;
    Serial.print(rawSensorValue);     Serial.print(",");
    #endif
   #ifdef POWER
   Serial.println("shutting down peripherals");
   powerOffPeripherals ();
   Serial.println("shut down peripherals");
   #endif
  //battVcc = analogRead(batt_pin);
  sensorValue=((rawSensorValue)/1023.0)*3.3;
  vol_water_cont = ((1.0/sensorValue)*slope)+intercept;
  // print the readings in the Serial Monitor
  Serial.println("");
  Serial.print("Soil Moisture Sensor Voltage: "); 
  Serial.print(sensorValue); //Serial.print(sensorValue); 
  Serial.println(" (V)");
  Serial.print("Theta_v: ");
  Serial.print(vol_water_cont);
  Serial.println(" cm^3/cm^3"); // cm^3/cm^3
  Serial.print("Sending Raw Data to nodered: "); 
  Serial.println(rawSensorValue); 
  client.publish(moisture_topic, String(rawSensorValue).c_str(), true);
  Serial.print("Sending Converted Data to nodered: "); 
  Serial.println(sensorValue);
  client.publish(converted_moisture_topic, String(sensorValue).c_str(), true);
  Serial.print("Sending Water Count to nodered: "); 
  Serial.println(vol_water_cont);
  client.publish(water_count_topic, String(vol_water_cont).c_str(), true);
  Serial.print("Sending Wifi RSSI to nodered: "); 
  long rssi = WiFi.RSSI();
  Serial.println(rssi);
  client.publish(wifi_rssi, String(rssi).c_str(), true);
 // Serial.print("Sending Battery Voltage to nodered: "); 
 // Serial.println(battVcc);
 // client.publish(vcc, String(battVcc).c_str(), true);
  
  #ifdef DateTimeOption
  timeClient.update();
  Serial.print(daysOfTheWeek[timeClient.getDay()]);
  Serial.print(", ");
  Serial.print(timeClient.getHours());
  Serial.print(":");
  Serial.print(timeClient.getMinutes());
  Serial.print(":");
  Serial.println(timeClient.getSeconds());
  //Serial.println(timeClient.getFormattedTime());
  
  delay(1000);
  #endif
  // turn the ledPin on
  //digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  //delay(sensorValue);
  // turn the ledPin off:
  //digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  delay(100);

  
  // Deep sleep mode for 30 seconds, the ESP8266 wakes up by itself when GPIO 16 (D0 in NodeMCU board) is connected to the RESET pin
  //Serial.println("I'm awake, but I'm going into deep sleep mode for 300 seconds");
  //ESP.deepSleep(300e6); 
  
  // Deep sleep mode until RESET pin is connected to a LOW signal (for example pushbutton or magnetic reed switch)
  //Serial.println("I'm awake, but I'm going into deep sleep mode until RESET pin is connected to a LOW signal");
  //ESP.deepSleep(0); 
  
      
  }
}
