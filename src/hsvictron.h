#ifndef HSVICTORN_H
#define HSVICTORN_H
#include "tasksdefine.h"

int setupVictron();
int readVictron();

#ifdef READ_VICTRON_ENABLED
struct t_VictronDATA { 
  //MPPT
  uint8_t msg_type;
  uint8_t msg_ver;
  uint16_t mainVoltage_V;      // mV
  uint16_t panelVoltage_VPV;   // mV/10
  uint16_t panelPower_PPV;     // W
  int16_t batteryCurrent_I;   // mA/10
  uint16_t yieldTotal_H19;     // 0.01 kWh
  uint16_t yieldToday_H20;     // 0.01 kWh
  uint16_t maxPowerToday_H21;  // W
  uint16_t yieldYesterday_H22; // 0.01 kWh
  uint16_t maxPowerYesterday_H23; // W
  uint8_t errorCode_ERR; //was int
  uint8_t stateOfOperation_CS; //was int

  //Phoenix inverter
  uint16_t p_V;      // mV
  uint16_t p_AC_OUT_V;
  uint16_t p_AC_OUT_S;
  uint8_t p_AC_OUT_I;
  uint8_t p_WARN; // Same as ar but for now can be multiple bits
  uint8_t p_AR; // alarm convert to 8 bit
  uint8_t p_CS; // convert to 8 bit
  uint8_t p_MODE; 
};

#endif //READ_VICTORN_ENABLED

#endif // HSVICTORN_H