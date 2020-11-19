#include <Arduino.h>
#include "hsvictron.h"
#include "main.h"

/* 
VE.Direct pinout and spec
TX RX communication in RS232 levels ( adapter to needed TTL level is needed )
____=____
|1 2 3 4|  ( as seen in controller )
---------
1 = GND White
2 = RX  Yellow
3 = TX  Black
4 = 5V ( max 10mA and 20mA in short burst ) Red
*/

int setupVictron()
{
#ifdef READ_VICTRON_ENABLED
    Serial1.begin(19200, SERIAL_8N1, 13, 14); //rx,tx Victron
    pinMode(21, OUTPUT);    // sets the digital pin as output ( victron load on/off )
    pinMode(22, OUTPUT);    // sets the digital pin as output ( victron load on/off )
    Serial.println("Set relay ON");
    digitalWrite(21, HIGH); //Vihrä
    delay(50);            // wait 50ms
    digitalWrite(21, LOW);
    Serial.println("Using Serial1 for ESP to Victorn communication.");
    Serial1.setTimeout(100);
    return (0);
#endif
    return (-1);
}
char buf[80];

float floatFromBuffer(String val) {
 val.toCharArray(buf, sizeof(buf));
 return atof(buf);
}
int intFromBuffer(String val) {
 val.toCharArray(buf, sizeof(buf));
 return atoi(buf);
}

int readVictron()
{
  #ifdef READ_VICTRON_ENABLED
  //The device transmits blocks of data at 1 second intervals. Each field is sent using the following format:
  DataOut.victronData.msg_type=read_victron;
  DataOut.victronData.msg_ver=0;
  //DataOut.victronData.mainVoltage_V = floatFromBuffer("1234");
  //DataOut.victronData.mainVoltage_V=12.34;
  //DataOut.victronData.panelVoltage_VPV=56.78;
  String label, val;
  while(Serial1.available()) {
       Serial1.read(); // read old buffers away
   }
  int maxdelay=100;
  while(!Serial1.available()) { //Wait data to become available
      delay(10);
      maxdelay--;
      if(maxdelay<1) {
        Serial.println("Timeout in Read Victron");
        break;
        }
      };
  while(Serial1.available()) {
        Serial.println("Read Victron:");
        label = Serial1.readStringUntil('\t');    // this is the actual line that reads the label from the MPPT controller
        val = Serial1.readStringUntil('\n');  // this is the line that reads the value of the label

     if (label =="V") {
         DataOut.victronData.mainVoltage_V = floatFromBuffer(val);
         Serial.print("Main voltage: ");
         Serial.print(val);
         Serial.print(" float: ");
         Serial.println(DataOut.victronData.mainVoltage_V);
     }
     else if (label =="VPV") {
         DataOut.victronData.panelVoltage_VPV = floatFromBuffer(val);
         Serial.print("Panel voltage: ");
         Serial.println(val);
     }
     else if (label =="PPV") {
         DataOut.victronData.panelPower_PPV = floatFromBuffer(val);
         Serial.print("Panael power: ");
         Serial.println(val);
     }
     else if (label =="I") {
         DataOut.victronData.batteryCurrent_I = floatFromBuffer(val);
         Serial.print("Battery current: ");
         Serial.println(val);
     }
     else if (label =="H19") {
         DataOut.victronData.yieldTotal_H19 = floatFromBuffer(val);
         Serial.print("Yield total: ");
         Serial.println(val);
     }
     else if (label =="H20") {
         DataOut.victronData.yieldToday_H20 = floatFromBuffer(val);
         Serial.print("Yield today: ");
         Serial.println(val);
     }
     else if (label =="H21") {
         DataOut.victronData.maxPowerToday_H21 = floatFromBuffer(val);
         Serial.print("Max power today: ");
         Serial.println(val);
     }
     else if (label =="H22") {
         DataOut.victronData.yieldYesterday_H22 = floatFromBuffer(val);
         Serial.print("Yield yday: ");
         Serial.println(val);
     }
     else if (label =="H23") {
         DataOut.victronData.maxPowerYesterday_H23 = floatFromBuffer(val);
         Serial.print("Yday max power: ");
         Serial.println(val);
     }
     else if (label =="ERR") {
         DataOut.victronData.errorCode_ERR = intFromBuffer(val);
         Serial.print("Error code: ");
         Serial.println(val);
     }
     else if (label =="CS") {
         DataOut.victronData.stateOfOperation_CS = intFromBuffer(val);
         Serial.print("State of operation: ");
         Serial.println(val);
     }

   }
   #endif
   return(0);
}