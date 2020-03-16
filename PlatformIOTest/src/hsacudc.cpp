#include <Arduino.h>
#include "hsacudc.h"
#include "main.h"
// Plus vieressä lähetys tx (ruskea)
// Miinus vieressä RX ohjaus (valkoinen)
int setupAcuDC()
{
#ifdef READ_ACUDC
    Serial1.begin(19200, SERIAL_8N1, 13, 2); // Victron
    pinMode(14, OUTPUT);    // sets the digital pin 14 as output ( victron load on/off )
    //while(true){
    digitalWrite(14, LOW); // sets the digital pin 13 off
    //delay(10000);            // waits for a second
    //digitalWrite(14, LOW);  // sets the digital pin 13 off
    //delay(10000); 
    //}
    Serial.println("Using Serial1 for ESP to Victorn communication.");
    Serial1.setTimeout(100);
    return (0);
#endif
    return (-1);
}
char buf2[80];

float floatFromBuffer2(String val) {
 val.toCharArray(buf2, sizeof(buf2));
 return atof(buf2);
}
int intFromBuffer2(String val) {
 val.toCharArray(buf2, sizeof(buf2));
 return atoi(buf2);
}

#include <ModbusMaster.h>
#define RXD2 34 //13
#define TXD2 19//15
#define RXenable 22//14

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission() {
  digitalWrite(RXenable, HIGH);
}

void postTransmission() {
  digitalWrite(RXenable, LOW);
}

void modbusSetup() {
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  pinMode(RXenable, OUTPUT);
  digitalWrite(RXenable, LOW);

  // Modbus slave ID 1
  node.begin(2, Serial2);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

float modbusReadFloat(uint16_t addr) {
  uint8_t result;
  uint16_t data[2];
  float val;

  result = node.readHoldingRegisters(addr, 2);
  if (result != node.ku8MBSuccess) {
    if ( result == 0x2e ) {
      Serial.println("Modbus read failed: Timeout");
    }
    else {
      Serial.print("Modbus read failed: ");
      Serial.println(result, HEX);
    }
    return (-1);
  }

  data[0] = node.getResponseBuffer(1);
  data[1] = node.getResponseBuffer(0);
  memcpy(&val, data, 4);
  return (val);
}

uint32_t modbusReadRunTime() {
  uint8_t result;
  result = node.readHoldingRegisters(0x0280, 2);
  if (result != node.ku8MBSuccess) {
    Serial.print("Modbus read failed: ");
    Serial.println(result, HEX);
    return (0);
  }
  return ((uint32_t)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
}

int readAcuDC()
{
  #ifdef READ_ACUDC
  //The device transmits blocks of data at 1 second intervals. Each field is sent using the following format:
  // Serial.setTimeout(); // ms default 1000
  DataOut.acudcData.msg_type=0x3a;
  DataOut.acudcData.msg_ver=0x2C;
  DataOut.acudcData.volt = modbusReadFloat(0x0200)*10;
  DataOut.acudcData.amp = modbusReadFloat(0x0202)*10;
  DataOut.acudcData.watt = modbusReadFloat(0x0204)*10;
  DataOut.acudcData.runTime = modbusReadRunTime()*10;
  
  #endif
  return(0);
}
