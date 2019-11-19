#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "nrf_802154_config.h"
#include "nrf_802154.h"
#include "nrf_delay.h"

#define MAX_MESSAGE_SIZE 125
#define CHANNEL          11

static volatile bool m_tx_in_progress;
static volatile bool m_tx_done;

int main(int argc, char *argv[])
{
   (void) argc;
   (void) argv;

   uint8_t message[MAX_MESSAGE_SIZE];

   for (uint32_t i = 0; i < sizeof(message) / sizeof(message[0]); i++)
   {
     message[i] = i;
   }

   message[0] = 0x41;                // Set MAC header: short addresses, no ACK
   message[1] = 0x98;                // Set MAC header
   for(int i=3;i<MAX_MESSAGE_SIZE;i++)
   {
      char buffer [2];
      itoa(i, buffer, 10);
      if(i<10)
        message[i] = buffer[0];
      else
      {
        message[i] = buffer[1];
      }
   }

   m_tx_in_progress = false;
   m_tx_done        = false;

   nrf_802154_init();
   nrf_802154_auto_pending_bit_set(false);
   nrf_802154_auto_ack_set(false);
   nrf_802154_channel_set(CHANNEL);
   nrf_802154_receive();

   uint8_t loopCnt = 0;
   while (1)
   {
     if (m_tx_done)
     {
         m_tx_in_progress = false;
         m_tx_done        = false;
     }

     if (!m_tx_in_progress)
     {
         //nrf_delay_ms(20);
         message[2] = loopCnt;
         m_tx_in_progress = true;
         nrf_802154_transmit_csma_ca (message, sizeof(message));//nrf_802154_transmit(message, sizeof(message), true);
         loopCnt++;
     }
   }
return 0;
}

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

void nrf_802154_transmit_failed(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
   m_tx_done = true;
}
/*

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
*/