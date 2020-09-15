#ifndef HSOTA_H
#define HSOTA_H

extern const char OTAname[];
extern const char OTApw[];

// Funktio määrittelyt ennen ifdef:ä, jotta stubeja voi kutsua vaikka ei käytössä
void hsota_setup();
void hsota_loop();

#ifdef OTA_ENABLED
#include <ArduinoOTA.h>

#endif //OTA_ENABLED
#endif //HSOTA_H