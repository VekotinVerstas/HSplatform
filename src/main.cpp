#include <Arduino.h>
// tässä määritellään mitkä taskit käytössä
#include "settings.h"
#include <esp_system.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>

#include "hsacudc.h"
#include "hslora.h"
#include "hsmqtt.h"
#include "main.h"

#if defined(SEND_DATA_WIFI_3_ENABLED) || defined(OTA_ENABLED) || defined(SYNCRONIZE_NTP_TIME_7_ENABLED)
#define WIFI_REQUIRED
#endif

#ifdef WIFI_REQUIRED
// wifi käytössä
// tähän wifin vaatimat includet ja vakio/muuttujamääritykset
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiType.h>
WiFiMulti wifiMulti;
bool softAPconnected=false;

#endif //WIFI_REQUIRED

bool clear_to_sleep = true; // LoRa send can prevent sleep
struct DATA_OUT DataOut; // result in static memory

// next start time of each task, initiated in startup
RTC_DATA_ATTR time_t next_run_time[MAX_TASK_COUNT];
bool printStatus[MAX_TASK_COUNT];
RTC_DATA_ATTR byte bootCount = 0; // This is going to overflow
//RTC_DATA_ATTR uint64_t Mics = 0;

#define UNDEFINED_TIME -1

#ifdef READ_TEMP_HUM_BME280_5_ENABLED
#include <hsbme280.h>
HSbme280 HSBME280Sensor;
RTC_DATA_ATTR HSbme280Data HSBME280SensorData;
#endif //READ_TEMP_HUM_BME280_5_ENABLED

#ifdef READ_HTU21D_6_ENABLED
#include <hshtu21d.h>
HShtu21d HShtu21dSensor;
RTC_DATA_ATTR HShtu21dData HSHTU21DSensorData;
#endif //READ_HTU21D_6_ENABLED

#ifdef OTA_ENABLED
#include "hsota.h"                 // tässä viittaus tiedostoon, jossa  ota-päivitykseen liittyvät funktiot
#endif //OTA_ENABLED

#include "hsdavis.h" //READ_WEATHER_DAVIS_8_ENABLED

#ifdef SYNCRONIZE_NTP_TIME_7_ENABLED
#include "hstimesync.h"
HSTimeSync TimeSync;
// see https://forum.arduino.cc/index.php?topic=595802.0 for code
const char *ntpServer = "pool.ntp.org";
#endif //SYNCRONIZE_NTP_TIME_7_ENABLED

#ifdef READ_EXTERNAL_VOLTAGE_9_ENABLED
//t_EXTERNAL_VOLTAGE_OUT externalVoltageData;
//float externalVoltage; // result in static memory
#endif

#ifdef WIFI_REQUIRED
bool connectWifi()
{
  if (softAPconnected )
  {
    return false; // softap not really connected
  }
  if( wifiMulti.run() == WL_CONNECTED )
  {
    return true; // already connected
  }
  wifiMulti.addAP(AP1_SSID, AP1_PWD);
#ifdef AP2_SSID
  wifiMulti.addAP(AP2_SSID, AP2_PWD);
#endif
#ifdef AP3_SSID
  wifiMulti.addAP(AP3_SSID, AP3_PWD);
#endif
#ifdef AP4_SSID
  wifiMulti.addAP(AP4_SSID, AP4_PWD);
#endif

  Serial.println("Connecting Wifi...");
  if (wifiMulti.run() == WL_CONNECTED)
  {
    Serial.println("WiFi connected!");
    IPAddress local = WiFi.localIP();
    IPAddress gatew = WiFi.gatewayIP();

    Serial.printf( (char *)"Connected to:    %s\r\n", WiFi.SSID().c_str());
    Serial.printf( (char *)"Signal strength: %ddBm\r\n", WiFi.RSSI());
    Serial.printf( (char *)"Local IP:        %d.%d.%d.%d\r\n", local[0], local[1], local[2], local[3]);
    Serial.printf( (char *)"Gateway IP:      %d.%d.%d.%d\r\n", gatew[0], gatew[1], gatew[2], gatew[3]);
    return true;
  }
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  if( WiFi.softAP(softAPname, softAPpw) )
  {
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Soft AP IP address: ");
    Serial.println(IP);
    // Print ESP8266 Local IP Address
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    softAPconnected = true;
  }
  else 
  {
    Serial.println("Unable to create local AP. Restarting...");
    ESP.restart();
  }
  return false;
}
#endif // WIFI_REQUIRED

#include "hsacudc.h"

//END OF TASK  SPECIFIC DEFINITIONS

void printLocalTime()
{
  time_t now;
  time(&now);
  struct tm *timeinfo;
  timeinfo = localtime(&now);
  Serial.printf("%s\r\n", asctime(timeinfo));
  delay(2); // 26 bytes @ 115200 baud is less than 2 ms
}
/*
void updateTime(uint64_t elapsedTime)
{ // elapsedTime in us
  if (elapsedTime == 0)
    Mics += micros();
  else
    Mics += elapsedTime;
  if (Mics > 1000000)
  {
    Mics = Mics % 1000000;
    now += Mics / 1000000;
  }
}
*/

bool time_to_run_task(int task_number)
{
  time_t now;
  time(&now);
  if ((next_run_time[task_number] <= now))
  {
    Serial.printf("Task %d was due %ld secs ago (now %ld, due %ld)\r\n", task_number, now - next_run_time[task_number], now, next_run_time[task_number]);
    return true;
  }
  else
  {
    //Print status only every 10secs
    if((next_run_time[task_number] - now)%10==0)
    {
      if(printStatus[task_number])
      {
      Serial.printf("Task %d is due after %ld secs\r\n", task_number, next_run_time[task_number] - now);
      printStatus[task_number]=false;
      }
    }
    else printStatus[task_number]=true;
    return false;
  }
};

void schedule_next_task_run(int task_number, time_t to_next_run_sec, bool from_now)
{
  time_t now_local;
  time(&now_local);
  if (from_now)
  {
    next_run_time[task_number] = now_local + to_next_run_sec;
  }
  else
  {
    if (next_run_time[task_number] < 1)
    {
      next_run_time[task_number] = now_local; // if undefined/0, initiate
    }
    next_run_time[task_number] += to_next_run_sec;

    if (next_run_time[task_number] < now_local)
    { //do not let tasks accumulate
      next_run_time[task_number] = now_local;
    }
  }
}

/* 
yleiskysymyksiä 
- voisko loralla saada reaaliajan ,jos odotetaan loraviestejä niin silloin nukkumaan ei voi mennä kovin usein. OTAn yhteydessä voisi hakea ajan wifillä.
- missä säilytetään data sleepin aikana, esim. RTC_DATA_ATTR int bootCount = 0; , RTC-muisti (4k) ei dynaaminen, mutta struct onnistuu https://esp32.com/viewtopic.php?t=1536
- tässä mallissa ei oikein moniajoa, vaan esim. wifi skannaus varaa koko devicen, onko lopulta ongelma

taskin ominaisuudet:
- taskeilla oltava ajojärjestys (nyt loopissa oleva järjestys) , jos riippuvat toisistaan, esim. jos bat-counter toimii vain lämpimänä yönä, pitää lämpö ja valo sensoroida ennen 
- frequency (esim. 300 s), taski voi määritellä seuraavan ajoajan itselleen muun sensoridata (pitää olla luettuna) perusteella, esim. harvemmin, jos jännite alhaalla
- taski voi määrittää myös toisen taskin seuraavan ajoajan, esim. pax-counter onnistuneen skannauksen jälkeen "käskee" lähettämään datan

esimerkkitaskeja:
read_battery_voltage - jos käytetään erilaisia akkuja (lion, lyijy 6v jne) niin tulos voisi olla vakioitu luokkiin low/normal/high
read_temp_hum_bme280 - lukee bme280 sensorilta lämpötilan ja kosteuden globaaliin muuttujaan/tietorakenteeseen
read_chimney_temp
read_water_temp
read_water_distance - luetaan etäisyys veteen eli 
read_lightness_xyz - lukee tietyllä sensorilla valoisuuden globaaliin muuttujaan
listen_bat_sounds - kuuntelee annetun ajan UÄ-sensorilla lepakkojen aktiivisuutta 
scan_pax_wifi - skannaa wifi-laitteet
scan_pax_ble - skannaa ble-laitteet
scan_battery_charger - nykyinen Vasikkasaaren Victronin luku voisi olla oma moduulinsa, sieltä pukkaan rivejä tasaiseen tahtiin

set_power_switches - tämä voisi muuttaa relekytkinten asentoa sensoriarvojen perusteella, esim. jos jännite alhaalla otetaan jostain virta pois, tai ohjataan tuuletinta lämpötilan/kosteuden perusteella

send_sensordata_lora kasaa sensoroidut datat ja lähettää paketin loralla
send_sensordata_wifi ... wifillä
*/

time_t time_of_earliest_run() // loop all tasks and return earliest run time
{
  int next_task_to_run = -1;

  time_t earliest_run_time = LONG_MAX;
  for (int i = 0; i < MAX_TASK_COUNT; i++)
  {
    if ((next_run_time[i] > 0) && (next_run_time[i] < earliest_run_time))
    {
      earliest_run_time = next_run_time[i]; // This is run at time and not sec for next run. ( since EPOC or boot )
      next_task_to_run = i;
    }
  }
  if (earliest_run_time == LONG_MAX)
  {
    time_t now_l;
    time(&now_l);
    earliest_run_time = now_l + MIN_SLEEPING_TIME_SECS;
    //Serial.printf("earliest_run_time =MIN_SLEEPING_TIME_SECS \r\n");
  }
  //Serial.printf("earliest_run_time %ld, task %d\r\n", earliest_run_time, next_task_to_run);
  return earliest_run_time;
}

/********** SETUP **********/
void setup()
{
  Serial.begin(19200);//115200 Other UARTS 1-2 seem to follow this even they have own sett  ings 
  delay(20);
  Serial.printf("\r\n%s - versio: %s %s\r\n", name, version, __DATE__);
  Serial.print("bootCount:");
  Serial.println(bootCount);
  printLocalTime();
  if (bootCount == 0)
  {
    for (int i = 0; i < MAX_TASK_COUNT; i++)
    {
      next_run_time[i] = UNDEFINED_TIME;
    }
    // disable ble, we do not need it (yet)
    esp_bluedroid_disable();
    esp_bt_controller_disable();
#ifdef RESTART_10_ENABLED
    schedule_next_task_run(restart, RESTART_INTERVAL, true);
#endif
  }

#ifdef SEND_DATA_WIFI_3_ENABLED
mqttsetup();
#endif                                      //SEND_DATA_WIFI_3_ENABLED

#ifdef READ_WEATHER_DAVIS_8_ENABLED
#include "hsdavis.h"
  setupDavis();
  //Serial1.begin(19200, SERIAL_8N1, 14, 13); // Davis
#endif                                      //READ_WEATHER_DAVIS_8_ENABLED
#ifdef READ_VICTRON_ENABLED
#include "hsvictron.h"
  setupVictron();
#endif                                      //READ_WEATHER_DAVIS_8_ENABLED

#ifdef READ_ACUDC
  #include "hsacudc.h"
  setupAcuDC();
#endif
#if defined(OTA_ENABLED) && defined(WIFI_REQUIRED) 
  //connect to WiFi
  bool connected = connectWifi();
  hsota_setup();
#endif     
hslora_setup();
}

/********* LOOP **********/
void loop()
{
#ifdef WIFI_REQUIRED
//connect to WiFi
bool connected = connectWifi();
#endif

#ifdef OTA_ENABLED
  hsota_loop();
#endif //OTA_ENABLED

#ifdef SEND_DATA_LORA_2_ENABLED
  os_runloop_once();
#endif

#ifdef SYNCRONIZE_NTP_TIME_7_ENABLED
  // Tämä viedään myöhemmin omaan luokkaan HSTimeSync
  if (time_to_run_task(task::syncronize_ntp_time))
  {
  if (connected)
    {
      //init and get the time
      //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      configTime(0, 0, ntpServer);
      delay(1000);
      Serial.println(" Got new time");
      printLocalTime();
    }
    else
    {
      Serial.println("Cannot connect wifi, no ntp time update");
    }
    schedule_next_task_run(syncronize_ntp_time, 120, false);
  }

#endif //SYNCRONIZE_NTP_TIME_7_ENABLED

#ifdef READ_WEATHER_DAVIS_8_ENABLED
/*  if (time_to_run_task(read_weather_davis))
  {
    // Read Davis
    readDavis();
    //schedule_next_task_run(send_data_lora, 0, true);
    schedule_next_task_run(read_weather_davis, 30, false); //Shedule next run
  }*/
#endif //READ_WEATHER_DAVIS_8_ENABLED

#ifdef READ_TEMP_HUM_BME280_5_ENABLED

  HSBME280SensorData = HSBME280Sensor.read_sensor_values();
  Serial.printf("BME280 temperature %f, humidity %f, pressure %f\r\n", HSBME280SensorData.temperature, HSBME280SensorData.humidity, HSBME280SensorData.pressure);
  delay(10000); // toistaiseksi näin
#endif          //READ_TEMP_HUM_BME280_5_ENABLED

#ifdef READ_HTU21D_6_ENABLED
  HSHTU21DSensorData = HShtu21dSensor.read_sensor_values();
  Serial.printf("HTU temperature %f, humidity %f, dew point %f\r\n", HSHTU21DSensorData.temperature, HSHTU21DSensorData.humidity, dewPoint(HSHTU21DSensorData.humidity, HSHTU21DSensorData.temperature));
  delay(10000); // toistaiseksi näin
#endif          //READ_HTU21D_6_ENABLED

#ifdef READ_EXTERNAL_VOLTAGE_9_ENABLED
  // EXTERNAL_VOLTAGE_9_GPIO 21
  // EXTERNAL_VOLTAGE_9_FACTOR

  if (time_to_run_task(task::read_external_volt))
  {
    pinMode(EXTERNAL_VOLTAGE_9_GPIO, INPUT); // setup Voltmeter
    int val1;
    delay(1000);
    val1 = analogRead(EXTERNAL_VOLTAGE_9_GPIO);
    // Serial.println("Voltage val1");
    // Serial.println(val1);
    // delay(100);
    //val2 = analogRead(EXTERNAL_VOLTAGE_9_GPIO);
    //delay(100);
    //val3 = analogRead(EXTERNAL_VOLTAGE_9_GPIO);

    DataOut.externalVoltageData.voltage = (float)(((val1)*EXTERNAL_VOLTAGE_9_FACTOR) / 4095);
    Serial.printf("External volttage: %.2f\r\n", DataOut.externalVoltageData.voltage);
    schedule_next_task_run(read_external_volt, TX_INTERVAL, false); // same interval as lora
  }
#endif //READ_EXTERNAL_VOLTAGE_9_ENABLED

#ifdef DEVICE_SCAN_WIFI_0_ENABLED

  // käynnistä pax-wifi-scan jos käynnistysaika koittanut
  if (time_to_run_task(task::scan_wifi))
  {
    //  kutsu skannausfuntiota, joka määritelty omassa
  }
#endif //DEVICE_SCAN_WIFI_0_ENABLED

#ifdef SEND_DATA_LORA_2_ENABLED
  if (time_to_run_task(send_data_lora))
  {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
      Serial.println("OP_TXRXPEND, not sending");
    }
    else
    {
    int sensorReadStatus=0;
    // Prepare upstream data transmission at the next possible time.
    #ifdef READ_WEATHER_DAVIS_8_ENABLED
      readDavis();
      //LMIC_setTxData2(2, (unsigned char *)&LoraOut, sizeof(LoraOut), 0);
    #endif //READ_WEATHER_DAVIS_8_ENABLED
    #ifdef READ_VICTRON_ENABLED
      sensorReadStatus = readVictron();
      //LMIC_svicetTxData2(2, (unsigned char *)&LoraOut, sizeof(LoraOut), 0);
    #endif //READ__ENABLED
    #ifdef READ_EXTERNAL_VOLTAGE_9_ENABLED
      DataOut.externalVoltageData.msg_type = 9;
      DataOut.externalVoltageData.msg_ver = 0;
      Serial.printf("Sending voltage: %f\r\n", DataOut.externalVoltageData.voltage);
      /*
      char buffer[8];
      memcpy( buffer, &voltageOut, 8 );
      for (int i = 0; i < sizeof(voltageOut); i++)
      {
        Serial.printf("%02X", buffer[i]);
      }   
      */
      //LMIC_setTxData2(1, (unsigned char *)&externalVoltageData, sizeof(externalVoltageData), 0);
    #endif
    #ifdef READ_ACUDC
      if( readAcuDC() < 0 )
        {
          Serial.println("AcuDC read fail. Reboot OTA delay pahse.");
          for(int i=0; i<120; i++)
            { // give time for OTA update and reboot
            #ifdef OTA_ENABLED
              hsota_loop();
            #endif //OTA_ENABLED
            delay(200);
            }
          Serial.println("Reboot AcuDC read fail.");
          ESP.restart();
        }
    #endif //READ__ENABLED

    //LMIC_setTxData2(2, STATICMSG, sizeof(STATICMSG), 0);
    if(sensorReadStatus > 2) {
    Serial.println("Sensor read fail. Skip lora send.");
    } 
    else {
    Serial.println("do_send");
    do_send(&sendjob);
    Serial.println("Packet queued!");
    clear_to_sleep = false;                         // do not sleep before the message is send
    }
    //next_run_time[task::send_data_lora] = LONG_MAX; // unschedule for now ( not to repeat send )
    /* EV_TXCOMPLETE in hslora shedules next task, but if TX fails shedule it also in here */
    Serial.printf("Backup shedule next LoRa send in %d seconds\r\n", TX_INTERVAL);
    schedule_next_task_run(send_data_lora, TX_INTERVAL, false);
    }

    // Next TX is scheduled after TX_COMPLETE event.
  }
#endif //SEND_DATA_LORA_2_ENABLED

#ifdef SEND_DATA_WIFI_3_ENABLED
  //mqttloop(); // Listen incoming mqtt
  //next_run_time[send_data_wifi] voidaan asettaa halutuksi (->0) esim. sensorin luvun yhteydessä jos halutaan lähettää heti tuore tieto wifillä
  if (time_to_run_task(send_data_wifi))
  {
  if (connected)
  {
    mqttsend("Davis/data", (const char *)&DataOut);
  }
  else
  {
    Serial.println("Cannot connect wifi!");
  }
#endif //SEND_DATA_WIFI_3_ENABLED

#ifdef WIFI_REQUIRED
  // close wifi here is still open
//  WiFi.disconnect(true);
//  WiFi.mode(WIFI_OFF);
#endif //WIFI_REQUIRED

#ifdef SEND_DATA_LORA_2_ENABLED
  // do it once anyway here
  os_runloop_once();
#endif

#ifdef OTA_ENABLED
  hsota_loop();
#endif //OTA_ENABLED

#ifdef RESTART_10_ENABLED
  //RESTART_INTERVAL 86400
  if (time_to_run_task(restart))
  {
    time_t now;
    time(&now);
    if ((next_run_time[restart] - now) < -3600 * 24 * 30)
    {
      // probably clock was synched from ntp, no reboot now
      schedule_next_task_run(restart, RESTART_INTERVAL, true);
    }
    else
    {
      ESP.restart();
    }
  }
#endif

//These are used with or without sleep
time_t now_local;
time(&now_local);
time_t time_to_next_run = (time_of_earliest_run() - now_local);

#ifdef SLEEP_ENABLED

#ifdef SEND_DATA_LORA_2_ENABLED
  while (!clear_to_sleep)
  { //wait until previous send is complete
    os_runloop_once();
  }
#endif // SEND_DATA_LORA_2_ENABLED

  if (time_to_next_run >= MIN_SLEEPING_TIME_SECS)
  {
    if (time_to_next_run > MAX_SLEEPING_TIME_SECS)
    {
      time_to_next_run = MAX_SLEEPING_TIME_SECS; // do not sleep too long even there is nothing to execute
    }
    bootCount++;
    Serial.printf("Sleeping %d seconds\r\n", (int)time_to_next_run);
    uint64_t sleeptime = (time_to_next_run * 1000000);
    esp_sleep_enable_timer_wakeup(sleeptime);
    esp_deep_sleep_start();
  }
  else
  {
    // just relax, but do not   sleep
    Serial.printf("Waiting %d seconds\r\n", (int)time_to_next_run);
    unsigned long wait_start_ms = millis();
    while ((millis() - wait_start_ms) < (time_to_next_run * 1000))
    {
#ifdef SEND_DATA_LORA_2_ENABLED
      os_runloop_once();
#endif
      delay(100); // Do slicly less actively notihing ;)
      // Serial.printf("%ld\t %d\r\n",(millis() - wait_start_ms),(time_to_next_run * 1000));
    }
  }
/* Do work if not sleeping
#else // no SLEEP_ENABLED
  Serial.printf("No Sleep enabled - waiting %ld sec\r\n", (long)time_to_next_run);
  unsigned long wait_start_ms = millis();
  while ((millis() - wait_start_ms) <= (time_to_next_run * 1000))
  {
#ifdef SEND_DATA_LORA_2_ENABLED
    os_runloop_once();
#endif
    delay(90);
  }*/
#endif // SLEEP_ENABLED 
}
