#include <arduino.h>
#include "tasksdefine.h"
#include "hsmqtt.h"
#include "main.h"

#ifdef SEND_DATA_WIFI_3_ENABLED
#define LISTEN_TOPIC     "Davis/reboot" 
const char* mqtt_server = "95.216.211.87";
//const char* mqtt_server = "fvh.uei.fi";
const char* mqttUser = "uei";
const char* mqttPassword = "uei2020";

/* create an instance of PubSubClient client */
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void mqttreceivedCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if( strcmp(topic,"David/reset") == 0 ) { 
    /* we got '1' -> on */
    if ((char)payload[0] == '1') {
      Serial.println("ESP reset via MQTT!");
      ESP.restart();
    } else {
   
    }
  }

}

void mqttsetup() {
  /* configure the MQTT server with IPaddress and port */
  mqttClient.setServer(mqtt_server, 1883);
  /* this receivedCallback function will be invoked 
  when client received subscribed topic */
  mqttClient.setCallback(mqttreceivedCallback);
}

void mqttconnect() {
  /* Loop until reconnected */
 // while (!mqttClient.connected()) {
    Serial.print("MQTT connecting ...");
    /* mqttClient ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword )) {
      Serial.println("connected");
      /* subscribe topic with default QoS 0*/
      mqttClient.subscribe(LISTEN_TOPIC);
    } else {
      Serial.print("failed, status code =");
      Serial.print(mqttClient.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      //delay(5000);
    }
  //}
}

void mqttloop() {
  /* if client was disconnected then try to reconnect again */
  if (!mqttClient.connected()) {
    mqttconnect();
  }
  /* this function will listen for incomming 
  subscribed topic-process-invoke receivedCallback */
  mqttClient.loop();  
}

boolean mqttsend(const char* topic, const char* payload) {
    /* if client was disconnected then try to reconnect again */
  if (!mqttClient.connected()) {
    mqttconnect();
  }
  /* publish the message */
  return( mqttClient.publish(topic, payload) );
}

#endif //SEND_DATA_WIFI_3_ENABLED