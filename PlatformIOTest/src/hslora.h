#ifndef HSLORA_H
#define HSLORA_H

// Funktio määrittelyt ennen ifdef:ä, jotta stubeja voi kutsua vaikka ei käytössä
void hslora_setup();

#ifdef SEND_DATA_LORA_2_ENABLED
#include <lmic.h>
#include <hal/hal.h>
extern RTC_DATA_ATTR byte bootCount;

typedef struct t_LORA_OUT
{
    uint8_t msg_type;
    uint8_t msg_ver;
} LORA_OUT;

//MDummy message to emulate 26 byte (AQBURK) payload ( bat counter probably has much shorter
//static PROGMEM u1_t STATICMSG[26] = {0x24, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//static osjob_t sendjob;

#endif //SEND_DATA_LORA_2_ENABLED

#endif //HSLORA_H