#include <Arduino.h>
#include "hsvictron.h"
#include "main.h"
#include <HardwareSerial.h>

HardwareSerial MPPT(1);
HardwareSerial Phoenix(2);
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
    MPPT.begin(19200, SERIAL_8N1, 13, 14); //rx,tx 
    Phoenix.begin(19200, SERIAL_8N1, 34, 21); //rx,tx
    pinMode(21, OUTPUT);    // sets the digital pin as output ( bistate relay load on )
    pinMode(22, OUTPUT);    // sets the digital pin as output ( bistate relay load off )
    Serial.println("Set relay ON");
    digitalWrite(21, HIGH); //Vihrä
    delay(50);            // wait 50ms
    digitalWrite(21, LOW);
    Serial.println("Using Serial1 for ESP to Victorn MPPT communication.");
    MPPT.setTimeout(100);
    Serial.println("Using Serial2 for ESP to Victorn inverter communication.");
    Phoenix.setTimeout(100);
    return (0);
#endif
    return (-1);
}

float floatFromBuffer(String val) {
    char buf[16];
 val.toCharArray(buf, sizeof(buf));
 return strtof(buf, NULL); //atof(buf);
}

uint8_t uint8FromBuffer(String val) {
    char buf[16];
    val.toCharArray(buf, sizeof(buf));
    return((uint8_t)strtoul(buf, NULL, 10)); 
}

uint16_t uint16FromBuffer(String val) {
    char buf[16];
    val.toCharArray(buf, sizeof(buf));
    return((uint16_t)strtoul(buf, NULL, 10)); 
}

int16_t int16FromBuffer(String val) {
    char buf[16];
    val.toCharArray(buf, sizeof(buf));
    return((int16_t)strtol(buf, NULL, 10)); 
}

int32_t int32FromBuffer(String val) {
    char buf[16];
    val.toCharArray(buf, sizeof(buf));
    return((int32_t)strtol(buf, NULL, 10)); 
}

int intFromBuffer(String val) {
    char buf[16];
 val.toCharArray(buf, sizeof(buf));
 return( (int)strtoul(buf, NULL, 10) ); //atoi(buf);
}

uint8_t select8bitfrom16bit( uint16_t var, uint16_t bits ) {
    uint8_t newvar=0;
    uint8_t pos=0;
    for(int bit=0; bit<16; bit++) {
        if((bits >> bit) & 1) { // if this is one of the wanted bits
            if((var >> bit) & 1) newvar=newvar|(1 << pos); // Copy vanted bit to newvar
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
  String label, val;
  while(MPPT.available()) {
       Serial.print(MPPT.read()); // read old buffers away
   }
  int maxdelay=200;
  while(!MPPT.available()) { //Wait data to become available
      delay(10);
      maxdelay--;
      if(maxdelay<1) {
        Serial.println("Timeout in Read Victron MPPT");
        sensorStatus=1;
        break;
        }
      };

delay(50);
  if(sensorStatus<1) {
    Serial.println("Read Victron MPPT");
    MPPT.read(); // Read start linefeed away
    delay(10);
  while(MPPT.available()) {    
    Serial.flush();
    label = MPPT.readStringUntil('\t');    // this is the actual line that reads the label from the MPPT controller
    val = MPPT.readStringUntil('\n');  // this is the line that reads the value of the label
        Serial.print(label);
        Serial.print(": ");
    Serial.println(val);
    Serial.flush();

     if (label =="V") {
         DataOut.victronData.mainVoltage_V = uint16FromBuffer(val);
         Serial.print("Main voltage: ");
         Serial.println(DataOut.victronData.mainVoltage_V);
     }
     else if (label =="VPV") {
         DataOut.victronData.panelVoltage_VPV = (uint16FromBuffer(val)/10);
         Serial.print("Panel voltage: ");
         Serial.println(DataOut.victronData.panelVoltage_VPV*10);
     }
     else if (label =="PPV") {
         DataOut.victronData.panelPower_PPV = uint16FromBuffer(val);
         Serial.print("Panael power: ");
         Serial.println(DataOut.victronData.panelPower_PPV);
     }
     else if (label =="I") {
         DataOut.victronData.batteryCurrent_I = (int16FromBuffer(val)/10);
         Serial.print("Battery current: ");
         Serial.println(DataOut.victronData.batteryCurrent_I*10);
     }
     else if (label =="H19") {
         DataOut.victronData.yieldTotal_H19 = uint16FromBuffer(val);
         Serial.print("Yield total: ");
         Serial.println(DataOut.victronData.yieldTotal_H19);         
     }
     else if (label =="H20") {
         DataOut.victronData.yieldToday_H20 = uint16FromBuffer(val);
         Serial.print("Yield today: ");
         Serial.println(DataOut.victronData.yieldToday_H20);
         }
     else if (label =="H21") {
         DataOut.victronData.maxPowerToday_H21 = uint16FromBuffer(val);
         Serial.print("Max power today: ");
         Serial.println(DataOut.victronData.maxPowerToday_H21);
     }
     else if (label =="H22") {
         DataOut.victronData.yieldYesterday_H22 = uint16FromBuffer(val);
         Serial.print("Yield yday: ");
         Serial.println(DataOut.victronData.yieldYesterday_H22);
     }
     else if (label =="H23") {
         DataOut.victronData.maxPowerYesterday_H23 = uint16FromBuffer(val);
         Serial.print("Yday max power: ");
         Serial.println(DataOut.victronData.maxPowerYesterday_H23);
     }
     else if (label =="ERR") {
         DataOut.victronData.errorCode_ERR = uint8FromBuffer(val);
         Serial.print("Error code: ");
         Serial.println(DataOut.victronData.errorCode_ERR);
     }
     else if (label =="CS") {
         DataOut.victronData.stateOfOperation_CS = uint8FromBuffer(val);
         Serial.print("State of operation: ");
         Serial.println(DataOut.victronData.stateOfOperation_CS );
     }
    delay(5);
  }
  }
  while(Phoenix.available()) {
       Phoenix.read(); // read old buffers away
   }

  maxdelay=200;
  while(!Phoenix.available()) { //Wait data to become available
      delay(10);
      maxdelay--;
      if(maxdelay<1) {
        Serial.println("Timeout in read Phoenix inverter");
        if( sensorStatus==1) sensorStatus=3; // Both reads faild
        else sensorStatus=2; // only inverter read fail
        break;
        }
      };
delay(50);

if(sensorStatus<2) {
  Serial.println("Read Phoenix inverter:");
  Phoenix.read(); // Read start linefeed away
  while(Phoenix.available()) {
        Serial.flush();
        label = Phoenix.readStringUntil('\t');    // this is the actual line that reads the label from the MPPT controller
        val = Phoenix.readStringUntil('\n');  // this is the line that reads the value of the label
        Serial.print(label);
        Serial.print(": ");
        Serial.print(val);
        if (label =="V") { 
         DataOut.victronData.p_V = uint16FromBuffer(val);
         Serial.print("Inverter main voltage: ");
         Serial.println(DataOut.victronData.p_V);
        }
	    else if (label =="AC_OUT_V") {
         DataOut.victronData.p_AC_OUT_V = uint16FromBuffer(val);
         Serial.print("Inverter AC out voltage: ");
         Serial.println(DataOut.victronData.p_AC_OUT_V);
        }
	    else if (label =="AC_OUT_I") {
         DataOut.victronData.p_AC_OUT_I = uint8FromBuffer(val);
         Serial.print("Inverter AC out curret: ");
         Serial.println(DataOut.victronData.p_AC_OUT_I);
        }
	    else if (label =="AC_OUT_S") {
         DataOut.victronData.p_AC_OUT_S = uint16FromBuffer(val);
         Serial.print("Inverter AC out power: ");
         Serial.println(DataOut.victronData.p_AC_OUT_S);
        }
	    else if (label =="WARN") {
         DataOut.victronData.p_WARN = select8bitfrom16bit(uint16FromBuffer(val), 0b0000111101100011);
         Serial.print("Inverter warn: ");
         Serial.println(DataOut.victronData.p_WARN);
        }
	    else if (label =="AR") {
         //if(16bitval & (1<<N))    
         DataOut.victronData.p_AR = select8bitfrom16bit(uint16FromBuffer(val), 0b0000111101100011);
         Serial.print("Inverter alarm reason: ");
         Serial.println(DataOut.victronData.p_AR);
        }
	    else if (label =="CS") {
         DataOut.victronData.p_CS = uint8FromBuffer(val);
         Serial.print("Inverter state of operation: ");
         Serial.println(DataOut.victronData.p_CS);
        }
	    else if (label =="MODE") {
         DataOut.victronData.p_AC_OUT_I = uint8FromBuffer(val);
         Serial.print("Inverter mode: ");
         Serial.println(DataOut.victronData.p_AC_OUT_I);
        }
   }
  }
   #endif
   
   return(sensorStatus);
}
