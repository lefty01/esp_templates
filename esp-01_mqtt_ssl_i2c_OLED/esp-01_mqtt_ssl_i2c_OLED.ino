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
 *
 * NOTE: SSD1306Wire.h from https://github.com/lefty01/esp8266-oled-ssd1306
 *       which is a fork of git@github.com:ThingPulse/esp8266-oled-ssd1306.git
 *       but contains a fix for (typo?) ESP8266 ARCH define (2021-01-15)
 *
 */



//#define DEBUG_ESP_SSL 1
//#define DEBUG 1
#define DEBUG_MQTT 1

const char* VERSION = "0.0.1";
#define MQTTDEVICEID "ESP-01_I2C_OLED"

#include "debug_print.h"
#include "wifi_mqtt_creds.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <SSD1306Wire.h>

#define SDA_PIN 1 // TX
#define SCL_PIN 3 // RX


String ipAddr;
String dnsAddr;
String rssi;

const unsigned max_wifi_wait_seconds = 30;
const int maxMqttRetry = 5;
bool isWifiAvailable = false;
bool isMqttAvailable = false;


#define EVERY_SECOND 1000
#define EVERY_MINUTE EVERY_SECOND * 60
unsigned long sw_timer_clock;
unsigned long sw_timer2_clock;

const char* MQTT_TOPIC_STATE     = MQTTDEVICEID "/state";
const char* MQTT_TOPIC_SET       = MQTTDEVICEID "/set";
const char* MQTT_TOPIC_UPTIME    = MQTTDEVICEID "/uptime";
const char* MQTT_TOPIC_RSSI      = MQTTDEVICEID "/rssi";
const char* MQTT_TOPIC_LIGHT     = MQTTDEVICEID "/light";
const char* MQTT_TOPIC_TEMP      = MQTTDEVICEID "/temp";
const char* MQTT_TOPIC_VOLT      = MQTTDEVICEID "/voltage";


BearSSL::X509List   server_cert(server_crt_str);
BearSSL::X509List   client_crt(client_crt_str);
BearSSL::PrivateKey client_key(client_key_str);

BearSSL::WiFiClientSecure net;

PubSubClient mqttClient(net);

SSD1306Wire display(0x3C, SDA_PIN, SCL_PIN);



void drawTextCenter(const char *text, int yPos, bool clear=true) {
  if (clear) display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.drawStringMaxWidth(display.getWidth()/2, yPos, 128, text);
  display.display();
}


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
  DEBUG_PRINTLN("SETUP BEGIN ...");
  DEBUG_PRINT("esp chipid: ");
  DEBUG_PRINTLNHEX(ESP.getChipId());

  pinMode(LED_BUILTIN, OUTPUT);    // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off (low active)

  // oled setup
  display.init();
  display.flipScreenVertically();
  display.setContrast(200);
  display.clear();
  drawTextCenter("Hello World", 0);

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

  drawTextCenter(ipAddr.c_str(), 0);

  DEBUG_PRINTLN("SETUP DONE");
}


void loop() {

  // if (isWifiAvailable) ArduinoOTA.handle();

  if (isWifiAvailable && (false == (isMqttAvailable = mqttClient.loop()))) {
    DEBUG_PRINTLN("mqtt connection lost ... try re-connect");
    isMqttAvailable = mqttConnect() ? false : true;
  }


  char up[16];
  sprintf(up, "%ld", sw_timer_clock);

  if ((millis() - sw_timer2_clock) > EVERY_MINUTE) {
    sw_timer2_clock = millis();
    if (isMqttAvailable) {
      rssi = WiFi.RSSI();
      isMqttAvailable = mqttClient.publish(MQTT_TOPIC_UPTIME, up);
      isMqttAvailable = mqttClient.publish(MQTT_TOPIC_RSSI, rssi.c_str());
    }
  }

  if ((millis() - sw_timer_clock) > EVERY_SECOND) {
    sw_timer_clock = millis();

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawStringMaxWidth(display.getWidth()/2, 0, 128, "UPTIME");
    display.drawLine(5, 20, display.getWidth()-5, 20);
    display.drawRect(20, 32, 88, 15);

    display.setFont(ArialMT_Plain_10);
    display.drawStringMaxWidth(display.getWidth()/2, 32, 128, up);

    display.display();
  }
}
