#include <arduino.h>
#include "tasksdefine.h"
#include "hslora.h"
#include "main.h"

#ifdef SEND_DATA_LORA_2_ENABLED
//const unsigned TX_INTERVAL = 180;
//extern const unsigned TX_INTERVAL;


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t* buf) {
  //memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey(u1_t *buf) {}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
/*
    int size = sizeof(DataOut);
    // declare send buffer (char byte array)
    unsigned char *sendData = new unsigned char[size];
    // copy current configuration (struct) to send buffer
    memcpy(sendData, &DataOut, size);
    LMIC_setTxData2(1, sendData, size-1, 0); // send data unconfirmed on RCMD Port
    delete sendData; // free memory
*/

    // Print data to serial 
    //Serial.write((byte*)&DataOut, sizeof(DataOut));
    xref2u1_t dp = (xref2u1_t) &DataOut;
    for (int i = 0; i < sizeof(DataOut); i++) Serial.printf("%02x", *dp++);
    Serial.println();
    Serial.println("do_send viesti jonossa");

    //unsigned char text[]={"test"};
    //LMIC_setTxData2(1, (xref2u1_t)text, 5, 0);
    LMIC_setTxData2(1, (unsigned char *)&DataOut, sizeof(DataOut), 0);
    //LMIC_setTxData2(1, dp, sizeof(DataOut), 0);

  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        char lora_response[30];
        uint8_t i;
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        for ( i=0 ; i < LMIC.dataLen ; i++ )
          lora_response[i] = LMIC.frame[LMIC.dataBeg + i];
        lora_response[i] = 0;

      // send the responce to serial
      Serial.write((uint8_t*)lora_response, strlen(lora_response));

      Serial.print(F("Data: "));
      Serial.print(lora_response);
      Serial.print(" Received at ");
      Serial.println(millis()/1000.00);
      LMIC.dataLen = 0;

      #ifdef READ_VICTRON_ENABLED
      /*
      if( lora_response[0]=='o' && lora_response[1]=='n' ) {
        Serial.println("Set relay ON");
        digitalWrite(21, HIGH); //Vihrä
        delay(50);            // wait 50ms
        digitalWrite(21, LOW);
      }
      else if( lora_response[0]=='o' && lora_response[1]=='f'  ) {
        Serial.println("Set relay OFF");
        digitalWrite(22, HIGH); // Violetti
        delay(50);            // wait 50ms
        digitalWrite(22, LOW);
      }
      */
      #endif
      if( lora_response[0]=='r' ) {
        ESP.restart();
      }
      
      }
      // Schedule next transmission
      Serial.printf("Next LoRa send in %d seconds\n", TX_INTERVAL);
      //schedule_next_task_run(send_data_lora, TX_INTERVAL, true);
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      clear_to_sleep = true; // buffer clear, sleep ok
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}
#endif //SEND_DATA_LORA_2_ENABLED

void hslora_setup()
{
    #ifdef SEND_DATA_LORA_2_ENABLED
    if (bootCount > -1)
    {
        // LMIC init
        os_init();
        // Reset the MAC state. Session and pending data transfers will be discarded.
        LMIC_reset();

        LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

        // Set up the channels used by the Things Network, which corresponds
        // to the defaults of most gateways. Without this, only three base
        // channels from the LoRaWAN specification are used, which certainly
        // works, so it is good for debugging, but can overload those
        // frequencies, so be sure to configure the full frequency range of
        // your network here (unless your network autoconfigures them).
        // Setting up channels should happen after LMIC_setSession, as that
        // configures the minimal channel set.
        LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
        LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
        LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

        // Disable link check validation
        LMIC_setLinkCheckMode(0);

        // Set data rate and transmit power (note: txpow seems to be ignored by the library)
        //LMIC_setDrTxpow(DR_SF12, 14);
        LMIC_setDrTxpow(DR_SF10, 14);

        // Start job
        //do_send(&sendjob);
        //schedule_next_task_run(send_data_lora, TX_INTERVAL, false);
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
    #endif //SEND_DATA_LORA_2_ENABLED
}
