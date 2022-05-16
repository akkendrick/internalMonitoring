#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <stdio.h>
#include "PubSubClient.h"
#include <ArduinoLowPower.h>
#include <RTCZero.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <hp_BH1750.h>  //  include the library

hp_BH1750 BH1750;       //  create the sensor

// Define wifi parameters
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int INTERVAL = 1;

// Define data storage variables
float bmeData[4];
float MQTTData[5];
#define Sensor_PWR 6

// Create an rtc object for tracking time
RTCZero rtc;

// MQTT properties
const char* mqtt_server = MQTT_address;  // IP address of the MQTT broker
const char* temp_topic = "indoor/conditions/temperature";
const char* humid_topic = "indoor/conditions/humidity";
const char* pressure_topic = "indoor/conditions/pressure";
const char* distance_topic = "indoor/conditions/distance";
const char* light_topic = "indoor/conditions/light";
const char* mqtt_username = MQTT_user; // MQTT username
const char* mqtt_password = MQTT_pass; // MQTT password
const char* clientID = "arduino"; // MQTT client ID

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;

// Connect to the computer running MQTT, 
// it is listening on port 1883 
PubSubClient client(mqtt_server, 1883, wifiClient);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

float lux;

/****************************************
* Auxiliary Functions
****************************************/

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging - Output the IP Address
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    delay(1000);
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    delay(1000);
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void getBMEData(float bmeData[4]) {

    float bmeTemp;
    float bmeHumid;
    float bmeAlt;
    float bmePressure;

    bool BMEstarted = false;
    
    // Power ON the BME280 board
    //pinMode(BME_PWR, OUTPUT);
    //digitalWrite(BME_PWR, HIGH);
    
    unsigned status;
    status = bme.begin();  

    int iter = 0;

    if (!status) {
      // Try restarting the BME 10 times
      while(iter < 10) {
        Serial.println("Stuck in BME Loop");
        // Try cycling BME280 power
        digitalWrite(Sensor_PWR, LOW);
        delay(50);
        digitalWrite(Sensor_PWR, HIGH);
        delay(50);
        iter++;

        // Update status each iteration
        status = bme.begin();
        if (status) {
          break;
        }
    }
 

    }
  
    // Check if the BME is connected
    if (status) {
        Serial.println("BME connected, getting measurements");

        bmeTemp = bme.readTemperature();
        bmeData[0] = bmeTemp;
  
        bmePressure = bme.readPressure() / 100.0F;
        bmeData[1] = bmePressure;
  
        bmeAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
        bmeData[2] = bmeAlt;
  
        bmeHumid = bme.readHumidity();
        bmeData[3] = bmeHumid;
      }
      else {
        Serial.println("BME not connected");

        bmeData[0] = 0;
        bmeData[1] = 0;
        bmeData[2] = 0;  
        bmeData[3] = 0;
      }
      

      //digitalWrite(BME_PWR, LOW);

}


float getLightData() {
    
    bool avail = BH1750.begin(BH1750_TO_GROUND);
    int iter = 0;

    if (!avail) {
      // Try restarting the Light sensor 10 times
      while(iter < 10) {
        Serial.println("Stuck in BH1750 Loop");
        // Try cycling power
        digitalWrite(Sensor_PWR, LOW);
        delay(100);
        digitalWrite(Sensor_PWR, HIGH);
        delay(100);
        iter++;

        // Update status each iteration
        bool avail = BH1750.begin(BH1750_TO_GROUND);
        if (avail) {
          break;
        }
    }
    }

    avail = BH1750.begin(BH1750_TO_GROUND);
    if (avail) {
      
      BH1750.start();
      lux=BH1750.getLux();  //  waits until a conversion finished
      Serial.print("BH1750 status:");
      Serial.println(avail); 
      Serial.print("The current lumens are:"); 
      Serial.println(lux);  
    }
    else {
      lux=0;
    }
 
  return lux;

}

bool sendMQTTData(float MQTTData[5]) {
  bool MQTTBool;
  bool MQTTfail;
  
  MQTTBool = client.publish(temp_topic, String(MQTTData[0]).c_str());
  MQTTfail = false;
  if (MQTTBool) {
    Serial.println("Temperature Data Sent");
  }
  else {
    Serial.println("Temperature data failed to send.");
    // Sleep and try again
    MQTTfail = true;
  }

  delay(100); // This delay ensures that BME data upload is all good
  MQTTBool = client.publish(humid_topic, String(MQTTData[3]).c_str());
  if (MQTTBool) {
    Serial.println("Humidity Data Sent");
  }
  else {
    Serial.println("Humidity data failed to send.");
    // Sleep and try again
    MQTTfail = true;
  }

  delay(100); // This delay ensures that BME data upload is all good
  MQTTBool = client.publish(pressure_topic, String(MQTTData[1]).c_str());
  if (MQTTBool) {
    Serial.println("Pressure Data Sent");
  }
  else {
    Serial.println("Pressure data failed to send.");
    // Sleep and try again
    MQTTfail = true;
  }

  delay(100); // This delay ensures that BME data upload is all good
  MQTTBool = client.publish(light_topic, String(MQTTData[4]).c_str());
  if (MQTTBool) {
    Serial.println("Light Data Sent");
  }
  else {
    Serial.println("Light data failed to send.");
    // Sleep and try again
    MQTTfail = true;
  }
  delay(100); // This delay ensures that BME data upload is all good
  ///////////////////////////////////////////////////

  return MQTTfail;
}


/****************************************
 * Main Functions
****************************************/
void setup() {
    Serial.begin(9600);
    delay(10); 
    
    rtc.begin(); // enable real time clock functionalities

    pinMode(LED_BUILTIN, OUTPUT);    
   

    Serial.println();

    // Power ON the Sensors
    pinMode(Sensor_PWR, OUTPUT);
    digitalWrite(Sensor_PWR, HIGH);

}

void loop() {

  float sleepTime;
  float US100_distance;
  float US100_distance1;
  float US100_distance2;
  float US100_distance3;
  float US100_time1;
  float US100_time2;
  float US100_time3;
  float US100_avgTime;  
  float accumulatedHeight;
  bool MQTTfail;

  // Power ON the Sensors
  pinMode(Sensor_PWR, OUTPUT);
  digitalWrite(Sensor_PWR, HIGH);

  connect_MQTT();
  
  // Get BME data
  ///////////////////////////////////////////////////
  getBMEData(bmeData);
  delay(100); // This delay ensures that BME data is all good
  printValues();
  ///////////////////////////////////////////////////

  // Get light sensor data
  ///////////////////////////////////////////////////
  lux = getLightData();

  ///////////////////////////////////////////////////
  // Publish all data to the MQTT Broker
  MQTTData[0] = bmeData[0];
  MQTTData[1] = bmeData[1];
  MQTTData[2] = bmeData[2];
  MQTTData[3] = bmeData[3];

  MQTTData[4] = lux; 
  MQTTfail = sendMQTTData(MQTTData);

  if (MQTTfail) {
    // Sleep for a short time if all is not good
    //sleepTime = 9000;
    sleepTime = 5000;

    Serial.println("Weather data failed to send");
  }
  else {
      Serial.println("Weather data sent!");

      // Sleep for a long time if all is  good
      sleepTime = 20000;
  }

  client.disconnect();  // disconnect from the MQTT broker
  
  WiFi.end(); //turn off wifi before sleep

  // Using built-in LED to indicate when it is awake
  digitalWrite(LED_BUILTIN, LOW);   
  digitalWrite(Sensor_PWR, LOW);

  Serial.println("Going to sleep");

  LowPower.sleep(int(sleepTime));

  delay(int(sleepTime));
  
  digitalWrite(LED_BUILTIN, HIGH);    


}
