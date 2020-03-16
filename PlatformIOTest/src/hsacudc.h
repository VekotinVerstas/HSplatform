#ifndef HSACUDC_H
#define HSACUDC_H
#include "tasksdefine.h"

int setupAcuDC();
int readAcuDC();

#ifdef READ_ACUDC
struct t_AcudcDATA { 
  uint8_t msg_type;
  uint8_t msg_ver;
  uint16_t volt;
  uint16_t amp;
  uint16_t watt;
  uint16_t runTime;
};

#endif //READ_ACUDC

#endif // HSACUDC_H