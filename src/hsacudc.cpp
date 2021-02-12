#include "tasksdefine.h"
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
bool readOk=true;

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
  if (result != node.ku8MBSuccess) 
  {
    Serial.print("Modbus address ");
    Serial.print(addr);
    Serial.print(" read failed");
    if ( result == 0x2e ) {
      Serial.println(": Timeout");
      readOk=false;
      return(0);

    }
    else 
    {
      Serial.print(": Error ");
      Serial.println(result, HEX);
      readOk=false;
      return(0);
     }
  }

  data[0] = node.getResponseBuffer(1);
  data[1] = node.getResponseBuffer(0);
  memcpy(&val, data, 4);
  return (val);
}

uint32_t modbusReadUint32(uint16_t addr) 
{
  uint8_t result;
  
  result = node.readHoldingRegisters(addr, 4);
  if (result != node.ku8MBSuccess) 
  {
    Serial.print("Modbus address ");
    Serial.print(addr);
    Serial.print(" read failed");
    if ( result == 0x2e ) {
      Serial.println(": Timeout");
      readOk = false;
      return(0);
    }
    else 
    {
      Serial.print(": Error ");
      Serial.println(result, HEX);
      readOk = false;
      return (0);
    }
  }
  return (((long)node.getResponseBuffer(0))<<16 ) | (node.getResponseBuffer(1));
}

#endif

int setupAcuDC()
{
#ifdef READ_ACUDC
    //Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    pinMode(RXenable, OUTPUT);
    digitalWrite(RXenable, LOW);
    // Modbus slave ID 2
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
  
  /* Clear AcuDC data
  node.writeSingleCoil(290,0xA0);
  node.writeSingleCoil(291,0xA0);
  node.writeSingleCoil(292,0xA0);
  node.writeSingleCoil(295,0xA0);
  node.writeSingleCoil(296,0xA0);
  node.writeSingleCoil(297,0xA0);*/
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  //node.setTransmitBuffer(0, 9600);
  //node.setTransmitBuffer(1, highWord(i));
  //Serial.print("Update baud reate to 9600:" );
  //Serial.println(node.writeMultipleRegisters(258, 1));

  DataOut.acudcData.msg_type=0x3a;
  DataOut.acudcData.msg_ver=0x2C;
  DataOut.acudcData.volt = modbusReadFloat(0x0200);
  DataOut.acudcData.amp = modbusReadFloat(0x0202);
  DataOut.acudcData.watt = modbusReadFloat(0x0204)*1000;
  DataOut.acudcData.runTime = modbusReadUint32(0x0280);
  DataOut.acudcData.inAh = modbusReadUint32(0x0308);
  DataOut.acudcData.outAh = modbusReadUint32(0x030A);
  DataOut.acudcData.inEnergy = modbusReadUint32(0x0300);
  DataOut.acudcData.outEnergy = modbusReadUint32(0x0302);
  if( !readOk ) return(-1);
  Serial.print( "AcuDC volt: ");
  Serial.println(DataOut.acudcData.volt);
  Serial.print( "AcuDC amp: ");
  Serial.println(DataOut.acudcData.amp);
  Serial.print( "AcuDC watt: ");
  Serial.println(DataOut.acudcData.watt);
  Serial.print( "AcuDC runtime: ");
  Serial.println(DataOut.acudcData.runTime);
  Serial.print( "AcuDC inAh: ");
  Serial.println(DataOut.acudcData.inAh);
  Serial.print( "AcuDC outAh: ");
  Serial.println(DataOut.acudcData.outAh);
  Serial.print( "AcuDC inEnergy: ");
  Serial.println(DataOut.acudcData.inEnergy);
  Serial.print( "AcuDC outEnergy: ");
  Serial.println(DataOut.acudcData.outEnergy);
  
  #endif
  return(0);
}
