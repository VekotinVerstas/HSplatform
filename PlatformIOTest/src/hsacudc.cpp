#include <Arduino.h>
#ifdef READ_ACUDC
#include "hsacudc.h"
#include "main.h"
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

#endif

int setupAcuDC()
{
#ifdef READ_ACUDC
    Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
    pinMode(RXenable, OUTPUT);
    digitalWrite(RXenable, LOW);
    // Modbus slave ID 1
    node.begin(2, Serial2);
    // Callbacks allow us to configure the RS485 transceiver correctly
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    return (0);
#endif
    return (-1);
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
  Serial.print( "AcuDC volt: ");
  Serial.println(DataOut.acudcData.volt);
  Serial.print( "AcuDC amp: ");
  Serial.println(DataOut.acudcData.amp);
  Serial.print( "AcuDC watt: ");
  Serial.println(DataOut.acudcData.watt);
  Serial.print( "AcuDC runtime: ");
  Serial.println(DataOut.acudcData.runTime);
  
  #endif
  return(0);
}
