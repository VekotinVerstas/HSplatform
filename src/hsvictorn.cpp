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
char buf[16];

float floatFromBuffer(String val) {
 val.toCharArray(buf, sizeof(buf));
 return strtof(buf, NULL); //atof(buf);
}

uint8_t uint8FromBuffer(String val) {
    val.toCharArray(buf, sizeof(buf));
    return((uint8_t)strtoul(buf, NULL, 10)); 
}

uint16_t uint16FromBuffer(String val) {
    val.toCharArray(buf, sizeof(buf));
    return((uint16_t)strtoul(buf, NULL, 10)); 
}

int32_t int32FromBuffer(String val) {
    val.toCharArray(buf, sizeof(buf));
    return((int32_t)strtol(buf, NULL, 10)); 
}

int intFromBuffer(String val) {
 val.toCharArray(buf, sizeof(buf));
 return( (int)strtoul(buf, NULL, 10) ); //atoi(buf);
}

uint8_t select8bitfrom16bit( uint16_t var, uint16_t bits ) {
    uint8_t newvar=0;
    uint8_t pos=0;
    for(int bit=0; bit<16; bit++) {
        if((bits >> bit) & 1) { // if this is one of the wanted bits
            if((var >> bit) & 1) newvar|(1 << pos); // Copy vanted bit to newvar
            pos++;
        }
    }
    return(newvar);
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
  DataOut.victronData.msg_ver=2;
  //DataOut.victronData.mainVoltage_V = floatFromBuffer("1234");
  //DataOut.victronData.mainVoltage_V=12.34;
  //DataOut.victronData.panelVoltage_VPV=56.78;
  String label, val;
  while(Serial1.available()) {
       Serial1.read(); // read old buffers away
   }
  int maxdelay=200;
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
         DataOut.victronData.mainVoltage_V = uint16FromBuffer(val);
         Serial.print("Main voltage: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.mainVoltage_V);
     }
     else if (label =="VPV") {
         DataOut.victronData.panelVoltage_VPV = uint16FromBuffer(val);
         Serial.print("Panel voltage: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.panelVoltage_VPV);
     }
     else if (label =="PPV") {
         DataOut.victronData.panelPower_PPV = uint16FromBuffer(val);
         Serial.print("Panael power: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.panelPower_PPV);
     }
     else if (label =="I") {
         DataOut.victronData.batteryCurrent_I = floatFromBuffer(val);
         Serial.print("Battery current: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.batteryCurrent_I);
     }
     else if (label =="H19") {
         DataOut.victronData.yieldTotal_H19 = floatFromBuffer(val);
         Serial.print("Yield total: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.yieldTotal_H19);         
     }
     else if (label =="H20") {
         DataOut.victronData.yieldToday_H20 = floatFromBuffer(val);
         Serial.print("Yield today: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.yieldToday_H20);     }
     else if (label =="H21") {
         DataOut.victronData.maxPowerToday_H21 = floatFromBuffer(val);
         Serial.print("Max power today: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.maxPowerToday_H21);
     }
     else if (label =="H22") {
         DataOut.victronData.yieldYesterday_H22 = floatFromBuffer(val);
         Serial.print("Yield yday: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.yieldYesterday_H22);
     }
     else if (label =="H23") {
         DataOut.victronData.maxPowerYesterday_H23 = floatFromBuffer(val);
         Serial.print("Yday max power: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.maxPowerYesterday_H23);
     }
     else if (label =="ERR") {
         DataOut.victronData.errorCode_ERR = intFromBuffer(val);
         Serial.print("Error code: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.errorCode_ERR);
     }
     else if (label =="CS") {
         DataOut.victronData.stateOfOperation_CS = intFromBuffer(val);
         Serial.print("State of operation: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.stateOfOperation_CS );
     }
  }

  while(Serial2.available()) {
       Serial2.read(); // read old buffers away
   }

  maxdelay=200;
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
        label = Serial2.readStringUntil('\t'); // read the label
        val = Serial2.readStringUntil('\n');   // read the value of the label
        Serial.print(label);
        Serial.print(": ");
        Serial.print(val);
        if (label =="V") { 
         DataOut.victronData.p_V = uint16FromBuffer(val);
         Serial.print("Inverter main voltage: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_V);
        }
	    else if (label =="AC_OUT_V") {
         DataOut.victronData.p_AC_OUT_V = uint16FromBuffer(val);
         Serial.print("Inverter AC out voltage: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_AC_OUT_V);
        }
	    else if (label =="AC_OUT_I") {
         DataOut.victronData.p_AC_OUT_I = uint8FromBuffer(val);
         Serial.print("Inverter AC out curret: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_AC_OUT_I);
        }
	    else if (label =="AC_OUT_S") {
         DataOut.victronData.p_AC_OUT_S = uint16FromBuffer(val);
         Serial.print("Inverter AC out power: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_AC_OUT_S);
        }
	    else if (label =="WARN") {
         DataOut.victronData.p_WARN = uint8FromBuffer(val);
         Serial.print("Inverter warn: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_WARN);
        }
	    else if (label =="AR") {
         //uint16_t 16bitval = = uint8FromBuffer(val);
         //if(16bitval & (1<<N))    
         DataOut.victronData.p_AR = uint8FromBuffer(val);
         Serial.print("Inverter alarm reason: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_AR);
        }
	    else if (label =="CS") {
         DataOut.victronData.p_CS = uint8FromBuffer(val);
         Serial.print("Inverter state of operation: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_CS);
        }
	    else if (label =="MODE") {
         DataOut.victronData.p_AC_OUT_I = uint8FromBuffer(val);
         Serial.print("Inverter mode: ");
         Serial.print(val);
         Serial.print(" after cast: ");
         Serial.println(DataOut.victronData.p_AC_OUT_I);
        }
   }

   #endif
   
   return(sensorStatus);
}