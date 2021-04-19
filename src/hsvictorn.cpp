#include <Arduino.h>
#include "hsvictron.h"
#include "main.h"

/* 
VE.Direct pinout and spec
Victron TX RX communication in TTL 5V levels ( adapter to needed TTL 3,3V level is needed )
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
    Serial1.begin(19200, SERIAL_8N1, 13, 14); //rx,tx 
    Serial2.begin(19200, SERIAL_8N1, 19, 21); //rx,tx 
    pinMode(21, OUTPUT);    // sets the digital pin as output ( bistate relay load on )
    pinMode(22, OUTPUT);    // sets the digital pin as output ( bistate relay load off )
    Serial.println("Set relay ON");
    digitalWrite(21, HIGH); //Vihrä
    delay(50);            // wait 50ms
    digitalWrite(21, LOW);
    Serial.println("Using Serial1 for ESP to Victorn MPPT communication.");
    Serial1.setTimeout(100);
    Serial.println("Using Serial2 for ESP to Victorn inverter communication.");
    Serial2.setTimeout(100);
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
  /* Message format: <Newline><Field-Label><Tab><Field-Value>
  Fields in MPPT:
    V: Main channel / battery mV
    VPV: Panel voltage mV
    PPV: Panel power W
    I: Battery current mA
    IL: Load current ( in small ones )
    LOAD: Load on/off ( in small ones )
    Relay: Relay state?
    OR: off reason
    H19: 0.01 kWh Yield total
    H20: Yield today
    H21: Max power W today
    H22: Yield yesterday
    H23: Max power W yesterday
    ERR: Error code
    FW: Firmware (16bit)
    PID: Product id
    SER#:  Serialnumber

  Fields in Phoenix Inverter:
    V: Main channel / battery V in mV
    Relay: Relay state?
    OR: off reason
    CS:  State of operation
    FW: Firmware (16bit)
    PID: Product id
    SER#:  Serialnumber
    MODE: Device mode
    AC_OUT_V: AC output voltage 0.01 V
    AC_OUT_I: AC output current 0.1 A
    AC_OUT_S: AC output apparent power VA (in some models )
    WARN: Warning reason ( can be multible  ) send out as 8bit
        Low voltage 1 -> 1
        High voltage 2 -> 2
        Low temo 32 -> 4
        High temp 64 -> 8
        Overload 256 -> 16
        DC-ripple 512 -> 32
        Low V AC out 1024 -> 64
        High V AC out 2048 -> 128
  */

  int sensorStatus = 0;
  #ifdef READ_VICTRON_ENABLED
  //The device transmits blocks of data at 1 second intervals. Each field is sent using the following format:
  DataOut.victronData.msg_type=read_victron;
  DataOut.victronData.msg_ver=1;
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
        Serial.println("Timeout in Read MPPT Victron");
        sensorStatus=1;
        break;
        }
      };

  while(Serial1.available()) {
        Serial.println("Read Victron MPPT:");
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

  while(Serial2.available()) {
       Serial2.read(); // read old buffers away
   }

  while(!Serial2.available()) { //Wait data to become available
      delay(10);
      maxdelay--;
      if(maxdelay<1) {
        Serial.println("Timeout in Read inverter Victron");
        if( sensorStatus==1) sensorStatus=3; // Both reads faild
        else sensorStatus=2; // only inverter read fail
        break;
        }
      };

  while(Serial2.available()) {
        Serial.println("Read Victron inverter:");
        label = Serial2.readStringUntil('\t');    // this is the actual line that reads the label from the MPPT controller
        val = Serial2.readStringUntil('\n');  // this is the line that reads the value of the label
        Serial.print(label);
        Serial.print(": ");
        Serial.print(val);
   }

   #endif
   
   return(sensorStatus);
}