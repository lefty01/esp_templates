#ifndef _wifi_mqtt_creds_h_
#define _wifi_mqtt_creds_h_

#ifndef NOMQTTCERTS
#include "server_mqtt.crt.h"
#include "client.crt.h"
#include "client.key.h"
#endif

const char* wifi_ssid = "WIFI_SSID";
const char* wifi_pass = "*********";

const char*    mqtt_host = "yourhost.de";
const char*    mqtt_user = "username";
const char*    mqtt_pass = "password";
const unsigned mqtt_port = 8883;

const char*    mqtt_host_int = "192.168.1.123";
const char*    mqtt_user_int = "username";
const char*    mqtt_pass_int = "password";
const unsigned mqtt_port_int = 1883;


#endif
