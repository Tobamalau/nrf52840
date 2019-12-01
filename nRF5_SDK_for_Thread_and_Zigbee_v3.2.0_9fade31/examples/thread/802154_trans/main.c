#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "nrf_802154_config.h"
#include "nrf_802154.h"
#include "nrf_delay.h"
#include "sampleDate.c"
#include "nrf_802154_hp_timer.h"

#define MACHEAD         9
#define PACKHEAD        1
#define PAYLOAD         107 
#define DESTINATIONPAN  0x1234
#define DESTINATION     0x5678
#define SOURCE          0x0001
#define MAX_MESSAGE_SIZE (MACHEAD + PACKHEAD + PAYLOAD)



#define CHANNEL         11
#define PACKETFRAG      3

#define TEMSTAMP 1



static volatile bool m_tx_in_progress;
static volatile bool m_tx_done;

//uint8_t *messageFragment(uint8_t *opusPackNb, uint8_t )

void setMacHead(uint8_t *message)
{
   message[0] = 0x41;                // Set MAC header: short addresses, no ACK
   message[1] = 0x98;                // Set MAC header
   message[3] = DESTINATIONPAN&0xff;
   message[4] = (DESTINATIONPAN>>8)&0xff;
   message[5] = DESTINATION&0xff;
   message[6] = (DESTINATION>>8)&0xff;
   message[7] = SOURCE&0xff;
   message[8] = (SOURCE>>8)&0xff;
}

int main(int argc, char *argv[])
{
   (void) argc;
   (void) argv;

   uint8_t message[MAX_MESSAGE_SIZE];
   memset( message, 0, sizeof(message));


   setMacHead(message);

   
   m_tx_in_progress = false;
   m_tx_done        = false;

   nrf_802154_init();
   nrf_802154_auto_pending_bit_set(false);
   nrf_802154_auto_ack_set(false);
   nrf_802154_channel_set(CHANNEL);
   nrf_802154_receive();

   uint8_t opusPackNb = 0;
   uint8_t opusPackFragNb = 1;
   
   while (1)
   {
      
      if(opusPackFragNb > PACKETFRAG)
      {
         opusPackNb++;
         opusPackFragNb = 1;
      }
      if (m_tx_done)
      {
         m_tx_in_progress = false;
         m_tx_done        = false;
      }

      if (!m_tx_in_progress)
      {
         /*bei uarte recive Pointer von ersten message Datenfeld übergeben*/
         for(int i=0;i<PAYLOAD;i++)
         {
              message[i+MACHEAD+PACKHEAD] = sampleData[i*opusPackFragNb];
         }
         message[2] = opusPackNb;
         message[MACHEAD] = opusPackFragNb;

         m_tx_in_progress = true;

#if NRF_802154_USE_RAW_API
         nrf_802154_transmit_csma_ca_raw(message);
#else
         nrf_802154_transmit_csma_ca(message, sizeof(message));//nrf_802154_transmit(message, sizeof(message), tendif
#endif
         opusPackFragNb++;
      }
   }
return 0;
}



void nrf_802154_transmit_failed(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
   m_tx_done = true;
}

#if NRF_802154_USE_RAW_API
void nrf_802154_transmitted_raw(const uint8_t * p_frame, uint8_t * p_ack, int8_t power, uint8_t lqi)
{
    (void) p_frame;
    (void) power;
    (void) lqi;

    m_tx_done = true;

    if (p_ack != NULL)
    {
        nrf_802154_buffer_free_raw(p_ack);
    }
}

#else

#if TEMSTAMP
void nrf_802154_transmitted_timestamp(const uint8_t * p_frame, uint8_t * p_ack, uint8_t length, int8_t power, uint8_t lqi, uint32_t time)
{
   (void) p_frame;
   (void) length;
   (void) power;
   (void) lqi;

   m_tx_done = true;
   uint32_t test = nrf_802154_hp_timer_timestamp_get();
   test = nrf_802154_hp_timer_current_time_get();
   printf("%d", test);
   if (p_ack != NULL)
   {
     nrf_802154_buffer_free(p_ack);
   }
}
#else
void nrf_802154_transmitted(const uint8_t * p_frame, uint8_t * p_ack, uint8_t length, int8_t power, uint8_t lqi)
{
   (void) p_frame;
   (void) length;
   (void) power;
   (void) lqi;

   m_tx_done = true;

   if (p_ack != NULL)
   {
     nrf_802154_buffer_free(p_ack);
   }
}
#endif
#endif

