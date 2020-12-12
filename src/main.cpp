
/**********************************************************************
 * MQTT client to read and publish weather data from Accurite family
 * of RF 433MHz sensors.
 * 
 * Arduino code to run on the ESP8266 or ESP32 to decode several 
 * Acurite devices Over The Air data stream.
 * 
 * Decoding protocol and prototype code from these sources:
 * Ray Wang (Rayshobby LLC) 
 *   http://rayshobby.net/?p=8998
 * Benjamin Larsson (RTL_433) 
 *   https://github.com/merbanan/rtl_433
 * Brad Hunting (Acurite_00592TX_sniffer) 
 *   https://github.com/bhunting/Acurite_00592TX_sniffer
 *
 * Tested on a WEMOS D1 MINI connected to a 433Mhz 
 * Superheterodyne Wireless Receiver Module
 * 
 * This works with these devices but more could be added
 *   Acurite Pro 5-in-1 (8 bytes)
 *    https://tinyurl.com/zwsl9oj
 *
 *   Acurite Ligtning Detector 06045M  (9 bytes)
 *    https://tinyurl.com/y7of6dq4
 * 
 *   Acurite Room Temp and Humidity 06044M (7 bytes)
 *    https://tinyurl.com/yc9fpx8q
 * 
 */

// MQTT setup
#include <ESP8266WiFi.h>          // required for wifi connection to MQTT broker
#include <PubSubClient.h>         // MQTT library
#include "../../../credentials.h" // My network and MQTT credential, this file not
                                  // distributed in the GIT repository.  You will
                                  // need to create you own file with the following
                                  // format:

/***************************
 * credentials.h file formate
#ifndef __CREDENTIALS_H
#define __CREDENTIALS_H
#define _MY_WIFI_SSID   ("YOURWIFISSID")
#define _MY_WIFI_PWD    ("YOURWIFIPASSWORD")
#define _MY_MQTT_SERVER ("YOURMQTTSERVERADDRESS")
#define _MY_MQTT_NAME   ("YOURMQTTUSERNAME")
#define _MY_MQTT_PWD    ("YOURMQTTPASSWORD")
#endif
*
*******************************/

// Update these with values correct for your network.
const char *ssid = _MY_WIFI_SSID;
const char *password = _MY_WIFI_PWD;
const char *mqtt_server = _MY_MQTT_SERVER;

WiFiClient espClient;              // wireless endpoint
PubSubClient client(espClient);    // MQTT endpoint
unsigned long lastMsgSentTime = 0; // timer to throttle message rate
#define PAYLOAD_BUFFER_SIZE (50)   // MQTT payload size max
char payload[PAYLOAD_BUFFER_SIZE]; // MQTT payload buffer
#define TOPIC_BUFFER_SIZE (50)
char topic[TOPIC_BUFFER_SIZE];  // MQTT topic string
static uint16_t batteryLow = 0;        // status of battery 0 = ok, 1 = low battery
static uint16_t sensorID = 0;          // Sensor ID returned from the sensor
static int16_t temperature = 0;        // temperature in units of 0.1 C
static int16_t humidity = 0;


// Connect ESP to you local wifi
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT subscribe callback, not used for this application but
// here if you need it
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // DEMO action to blind the WEMOS LED if a message received with the correct payload
  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because
                                    // it is active low on the ESP-01)
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off by making the voltage HIGH
  }
}

// Reconnect to MQTT broker if not connected
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "WEMOSTEMP-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), _MY_MQTT_NAME, _MY_MQTT_PWD))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("home/temperature", "INIT");
      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Setup for reading RF 433 MHz wireless radio
//#include <Arduino.h>
#include "rf433recv.h"
#include "decoder_accurite.h"

// Arduino framework setup call
void setup()
{
  Serial.begin(115200);
  Serial.println("Started.");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); // MQTT callback for subscribed messages

  pinMode(LED_BUILTIN, OUTPUT); // LED output

  setupRF433();
  attachRF433int();
  unsquelchRF433();
}

#include <algorithm>    // std::min
/*
 * Main Loop
 * Wait for received to be true, meaning a sync stream plus
 * all of the data bit edges have been found.
 * Convert all of the pulse timings to bits and calculate
 * the results.
 */
void loop()
{
  static uint16_t lastSensorIdSent = 0; // keep track of last msg id sent to avoid sending duplicates at a high rate

  if (receivedBitsRF433() == true)
  {
    // disable interrupt to avoid new data corrupting the buffer
    detachRF433int();

//define DISPLAY_BIT_TIMING
#ifdef DISPLAY_BIT_TIMING
    displayBitTiming();
#endif // DISPLAY_BIT_TIMING

    // decode received bits into a local buffer
    byte dataBytes[MAX_DATA_BYTES_RECV];
    uint16 recvByteCount = std::min(bytesRecvCntRF433(), (unsigned int)MAX_DATA_BYTES_RECV);
    bool bitdecodefail = decodeBitstreamRF433( dataBytes, recvByteCount);

    // after decoding bit stream reset receiver and turn on interrupts
    resetBitStreamRF433();
    // re-enable interrupt
    attachRF433int();

    // next move on to displaying and sending received data
    // Display the raw data received in hex
//#define DISPLAY_DATA_BYTES
#ifdef DISPLAY_DATA_BYTES
    if (bitdecodefail)
    {
      Serial.println("Data Byte Display : Decoding error.");
    }
    else
    {
      for (int i = 0; i < bytesReceived; i++)
      {
        PrintHex8(&dataBytes[i], 1);
        Serial.print(",");
      }
    }
#endif

    // extract temperature value
    if (bitdecodefail)
    {
      // do nothing and wait for next sequence of bits
      Serial.println("bitdecodefail : Decoding error.");
    }
    else if (recvByteCount == 7)
    {
      //Small Tower sensor with Temp and Humidity Only
      decode_Acurite_6044(dataBytes);
    }
    else if (recvByteCount == 8)
    {
      //5n1 tower sensor
      decode_5n1(dataBytes);
    }
    else if (recvByteCount == 9)
    {
      //Lightening detector
      decode_Acurite_6045(dataBytes);
    }

    // MQTT connection check
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();

    // if decoded msg is ok then send via mqtt
    if( !bitdecodefail)
    {
      // insert a wait time between receiving, decoding, and sending data to
      // limit double sends of RF data
      sensorID = acurite_txr_getSensorId(dataBytes[0], dataBytes[1]);
      unsigned long now = millis();
      // if sending a new reading from a new sensor OR been long enough between last msg sent
      if((sensorID != lastSensorIdSent) || ((now - lastMsgSentTime) > 250))
      {
        lastMsgSentTime = now;
        lastSensorIdSent = sensorID; // update last sensor id to this sensor being sent
        batteryLow = (((dataBytes[4] & 0x20) == 0x20) ? 1 : 0);
        temperature = acurite_getTemp_6044M(dataBytes[4], dataBytes[5]);
        humidity = acurite_getHumidity(dataBytes[3]);
        snprintf(payload, PAYLOAD_BUFFER_SIZE, "%u;%d;%d", batteryLow, temperature, humidity);
        sniprintf(topic, TOPIC_BUFFER_SIZE, "home/temperature/%04X", sensorID);
        Serial.print("Publish message: ");
        Serial.print(topic);
        Serial.print(" ");
        Serial.println(payload);
        client.publish(topic, payload);
      }
    }
  } // received(true)
} // loop()
