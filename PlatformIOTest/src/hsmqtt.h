#ifndef HSMQTT_H
#define HSMQTT_H

// Funktio määrittelyt ennen ifdef:ä, jotta stubeja voi kutsua vaikka ei käytössä
//void hslora_setup();

#ifdef SEND_DATA_WIFI_3_ENABLED
#include <WiFi.h>
#include "main.h"
#include <PubSubClient.h>

void mqttsetup();
void mqttloop();
boolean mqttsend(const char* topic, const char* payload);

#endif //SEND_DATA_WIFI_3_ENABLED
#endif //HSMQTT_H