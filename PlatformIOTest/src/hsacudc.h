#ifndef HSACUDC_H
#define HSACUDC_H
#include "tasksdefine.h"

int setupAcuDC();
int readAcuDC();

#ifdef READ_ACUDC
struct t_AcudcDATA { 
  uint8_t msg_type;
  uint8_t msg_ver;
  float volt;
  float amp;
  float watt;
  uint32_t runTime;
  uint32_t inEnergy;
  uint32_t outEnergy;
  uint32_t inAh;
  uint32_t outAh;
};

#endif //READ_ACUDC

#endif // HSACUDC_H