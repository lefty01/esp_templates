/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 by Andreas Loeffler
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


//#define DEBUG_ESP_SSL 1
#define DEBUG 1
#define DEBUG_MQTT 1

const char* VERSION = "0.0.1";
#define MQTTDEVICEID "ESP-01_MQTTSSL_01"

#include "debug_print.h"
#include "wifi_mqtt_creds.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

String ipAddr;
String dnsAddr;
String rssi;

const unsigned max_wifi_wait_seconds = 30;
const int maxMqttRetry = 5;
bool isWifiAvailable = false;
bool isMqttAvailable = false;


#define EVERY_SECOND 1000
unsigned long sw_timer_clock;

const char* MQTT_TOPIC_STATE     = MQTTDEVICEID "/state";
const char* MQTT_TOPIC_SET       = MQTTDEVICEID "/set";
const char* MQTT_TOPIC_UPTIME    = MQTTDEVICEID "/uptime";


BearSSL::X509List   server_cert(server_crt_str);
BearSSL::X509List   client_crt(client_crt_str);
BearSSL::PrivateKey client_key(client_key_str);

BearSSL::WiFiClientSecure net;

PubSubClient mqttClient(net);


int setupWifi() {
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Connecting to wifi");

  WiFi.begin(wifi_ssid, wifi_pass);

  unsigned retry_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    DEBUG_PRINT(".");
    retry_counter++;

    if (retry_counter > max_wifi_wait_seconds) {
      DEBUG_PRINTLN(" TIMEOUT!");
      return 1;
    }
  }
  ipAddr  = WiFi.localIP().toString();
  dnsAddr = WiFi.dnsIP().toString();

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINTLN("IP address: ");
  DEBUG_PRINTLN(ipAddr);
  DEBUG_PRINTLN("DNS address: ");
  DEBUG_PRINTLN(dnsAddr);

  return 0;
}

int mqttConnect()
{
  DEBUG_PRINT("Attempting MQTT connection...");
  String connect_msg = "CONNECTED ";
  int rc = 0;

  connect_msg += VERSION;

  // Attempt to connect
  if (mqttClient.connect(MQTTDEVICEID, mqtt_user, mqtt_pass,
			 MQTT_TOPIC_STATE, 1, 1, "OFFLINE")) {
    DEBUG_PRINTLN("connected");

    // Once connected, publish an announcement...
    mqttClient.publish(MQTT_TOPIC_STATE, connect_msg.c_str(), true);

    // subscriptions ...
    mqttClient.subscribe(MQTT_TOPIC_SET);
  }
  else {
    DEBUG_PRINT("failed, mqttClient.state = ");
    DEBUG_PRINTLN(mqttClient.state());
    DEBUG_PRINT_MQTTSTATE(mqttClient.state());
    rc = 1;
  }
  delay(1000);
  return rc;
}


void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  DEBUG_PRINT("Message arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("] ");

  char value[length+1];
  memcpy(value, payload, length);
  value[length] = '\0';

  DEBUG_PRINTLN(value);

  if (0 == strcmp(MQTT_TOPIC_SET, topic)) {
    DEBUG_PRINTLN("set topic");

    if (0 == memcmp("on", payload, 2)) {
      DEBUG_PRINTLN("set ON");
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (0 == memcmp("off", payload, 3)) {
      DEBUG_PRINTLN("set OFF");
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}


void setup() {

  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("setup begin...");
  DEBUG_PRINTHEX(ESP.getChipId());

  pinMode(LED_BUILTIN, OUTPUT);    // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off (low active)

  DEBUG_PRINTLN("starting setupWifi()");

  isWifiAvailable = setupWifi() ? false : true;

  if (isWifiAvailable) {
    // Use WiFiClientSecure class to create TLS connection ****
    DEBUG_PRINTLN("setTrustAnchors()");
    net.setTrustAnchors(&server_cert);
    //  DEBUG_PRINTLN("allowSelfSignedCerts()");
    //  net.allowSelfSignedCerts();
    DEBUG_PRINTLN("setClientRSACert() client cert and key");
    net.setClientRSACert(&client_crt, &client_key);

    DEBUG_PRINTLN("set mqtt host/port and callback");
    mqttClient.setServer(mqtt_host, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    DEBUG_PRINTLN("calling mqttConnect()");
    isMqttAvailable = mqttConnect() ? false : true;
  }
  if (isMqttAvailable) {
    isMqttAvailable = mqttClient.publish(MQTT_TOPIC_STATE, "setup done");
  }
  DEBUG_PRINTLN("setup_done");
}


void loop() {

  // if (isWifiAvailable) ArduinoOTA.handle();

  if (isWifiAvailable && (false == (isMqttAvailable = mqttClient.loop()))) {
    DEBUG_PRINTLN("mqtt connection lost ... try re-connect");
    isMqttAvailable = mqttConnect() ? false : true;
  }

  if ((millis() - sw_timer_clock) > EVERY_SECOND * 5) {
    sw_timer_clock = millis();

    if (isMqttAvailable) {
      char up[16];
      sprintf(up, "%ld", sw_timer_clock);
      isMqttAvailable = mqttClient.publish(MQTT_TOPIC_UPTIME, up);
    }
  }
}

