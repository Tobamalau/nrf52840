#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <string.h>

#include "sdk_config.h"
#include "opus.h"
#include "opusTobi.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "youtube48_8_vbr.c"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "nrfx_uarte.h"

#include "nrf_802154_config.h"
#include "nrf_802154.h"

#define VERBOSE 0
#define EXTMUSIKSOURCE 1
#define UARTE_RX_BUFF_SIZE (FRAME_SIZE/3 + 4)

const nrf_drv_timer_t TIMER = NRF_DRV_TIMER_INSTANCE(0);
volatile uint16_t timerCnt = 0;

volatile int test = 0;
volatile bool Tx_empty = false;


unsigned char *txUarteBuffer;
unsigned char rxUarteBuffer[2][UARTE_RX_BUFF_SIZE];
volatile bool InRecive = false;
volatile uint8_t UarteBufferPos = 0;
volatile uint8_t UartBufferLoad = 0;
volatile uint8_t DecodeBufferPos = 0;
//uint8_t txBuffer[] = {0x72};
volatile uint8_t newFrame = 0;

nrfx_uarte_t m_uart = NRFX_UARTE_INSTANCE(0);

void initI2S()
{
   // Enable transmission
   NRF_I2S->CONFIG.TXEN = (I2S_CONFIG_TXEN_TXEN_ENABLE << I2S_CONFIG_TXEN_TXEN_Pos);
   // Enable MCK generator
   NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
   // MCKFREQ
   NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
   // Ratio = 96
   NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X << I2S_CONFIG_RATIO_RATIO_Pos;    //64
   // Master mode, 16Bit, left aligned
   NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;
   NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16BIT << I2S_CONFIG_SWIDTH_SWIDTH_Pos;
   NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_LEFT << I2S_CONFIG_ALIGN_ALIGN_Pos;
   // Format = I2S
   NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos;
   // Use stereo
   //NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_STEREO << I2S_CONFIG_CHANNELS_CHANNELS_Pos;
   NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Left << I2S_CONFIG_CHANNELS_CHANNELS_Pos;
   // Configure pins
   NRF_I2S->PSEL.MCK = (I2S_CONFIG_MCK_PIN << I2S_PSEL_MCK_PIN_Pos);
   NRF_I2S->PSEL.SCK = (I2S_CONFIG_SCK_PIN << I2S_PSEL_SCK_PIN_Pos);
   NRF_I2S->PSEL.LRCK = (I2S_CONFIG_LRCK_PIN << I2S_PSEL_LRCK_PIN_Pos);
   NRF_I2S->PSEL.SDOUT = (I2S_SDOUT_PIN << I2S_PSEL_SDOUT_PIN_Pos);

   NRF_I2S->ENABLE = 1;
}
void stopI2S()
{
   NRF_I2S->TASKS_START = 0;
   NRF_I2S->TASKS_STOP = 1;
   *((volatile uint32_t *)0x40025038) = 1;/*Workaround for sdk issu*/
   *((volatile uint32_t *)0x4002503C) = 1;
   //InRecive = true;
   //nrfx_uarte_rx(&m_uart, rxUarteBuffer[UarteBufferPos], sizeof(rxUarteBuffer[UarteBufferPos]));
}
/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            timerCnt++;
            if(timerCnt == 1000)
            {
               timerCnt = 0;
               //printf("\r1 Sekunde");
            }
            break;

        default:
            //Do nothing.
            break;
    }
}
void m_uart_callback(nrfx_uarte_event_t const * p_event,
                      void *                     p_context)
{

   switch(p_event->type)
   {
      case NRFX_UARTE_EVT_TX_DONE:  
         
         nrfx_uarte_rx(&m_uart, rxUarteBuffer[UarteBufferPos], sizeof(rxUarteBuffer[UarteBufferPos]));        
         break;

      case NRFX_UARTE_EVT_RX_DONE: 
         InRecive = false;
         UartBufferLoad |= (1 << UarteBufferPos); //new decode Buffer available
         UarteBufferPos ^= (1 << 0);            //UartBufferLoad &= ~(1 << UarteBufferPos) löschen          
         if(UartBufferLoad != 3)
         {
            InRecive = true;
            char txBuffer[] = {'r', 'E', DecodeBufferPos+0x30, UartBufferLoad+0x30, UarteBufferPos+0x30, test+0x30};
            nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer, sizeof(txBuffer));
         }
         if(NRF_I2S->TASKS_START == 0)
            newFrame = 1;
         break;

      case NRFX_UARTE_EVT_ERROR:
         break;      
   }


}
void m_uart_context_callback()
{

}
uint8_t uarte_init()
{
    
    nrfx_uarte_config_t m_uart_config = {//= NRFX_UARTE_DEFAULT_CONFIG;
                      TX_PIN_NUMBER,               ///< TXD pin number.
                      RX_PIN_NUMBER,               ///< RXD pin number.
                      CTS_PIN_NUMBER,              ///< CTS pin number.
                      RTS_PIN_NUMBER,              ///< RTS pin number.
                      //NULL,
                      m_uart_context_callback,                        ///< Context passed to interrupt handler.
                      UARTE_CONFIG_HWFC_Disabled,              ///< Flow control configuration.
                      UARTE_CONFIG_PARITY_Excluded,            ///< Parity configuration.
                      UARTE_BAUDRATE_BAUDRATE_Baud460800,      ///< Baudrate.
                      NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,  ///< Interrupt priority.
    };
    return nrfx_uarte_init(&m_uart, &m_uart_config, m_uart_callback);
}
int main(void)
{
   uint32_t err;
   err = uarte_init();
   APP_ERROR_CHECK(err);
   initI2S();
   /*Timer Init*/
   uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
   uint32_t time_ticks;

   //Configure TIMER for generating 1ms takt
   nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
   err = nrf_drv_timer_init(&TIMER, &timer_cfg, timer_event_handler);
   APP_ERROR_CHECK(err);

   time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER, time_ms);

   nrf_drv_timer_extended_compare(
      &TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

   nrf_drv_timer_enable(&TIMER);
         
    
   volatile uint8_t bufferNr = 0;
   struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
   struct frame FrameInstanz = {&OpusInstanz, 0, 0};
   FrameInstanz.nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
   initOpusFrame(&FrameInstanz);
#if VERBOSE == 1
   printf("Opus/UART Init\n");
#endif

   InRecive = true;
   nrfx_uarte_rx(&m_uart, rxUarteBuffer[UarteBufferPos], sizeof(rxUarteBuffer[UarteBufferPos]));
   while (1)
   {
      while (FrameInstanz.nbbytescnt>FrameInstanz.loopcnt)
      {
         if(newFrame)
         {
            /*no new Buffer available*/
            if(!UartBufferLoad != 0)   
            {
               stopI2S();
               newFrame = 0;
            }          

            FrameInstanz.opus_t->input = rxUarteBuffer[DecodeBufferPos] + 4;//msgBuffer + 4;
            FrameInstanz.opus_t->nbBytes = UARTE_RX_BUFF_SIZE - 4;   
            
            UartBufferLoad &= ~(1 << DecodeBufferPos);    //Delet Bufferloadmemory
            DecodeBufferPos ^= (1 << 0);                  //switch to new Uart Buffer

            /*new buffer request from Uarte*/
            if(!InRecive)
            {
               InRecive = true;
               char txBuffer[] = {'r', 'W', DecodeBufferPos+0x30, UartBufferLoad+0x30, UarteBufferPos+0x30, test+0x30};            
               nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer, sizeof(txBuffer));
            }

            if(!getPcm(&FrameInstanz, bufferNr))
               APP_ERROR_CHECK(NRF_ERROR_BUSY);

            if(NRF_I2S->TASKS_START == 0)
            {
               NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
               NRF_I2S->RXTXD.MAXCNT = FRAME_SIZE/2;
               NRF_I2S->TASKS_START = 1;
               bufferNr ^= (1 << 0);
            }
            newFrame = 0;
         }
         /* New I2S buffer*/
         if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
         {
#if VERBOSE == 2
            //printf("\nEVENTS_TXPTRUPD bufferNr:%d\n", bufferNr);
            printf("\r%d ms", timerCnt);
#endif
            NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
            timerCnt = 0;
            NRF_I2S->EVENTS_TXPTRUPD = 0;
            newFrame = 1;
            bufferNr ^= (1 << 0);
         }
      }
      FrameInstanz.loopcnt = 0;
      FrameInstanz.nbbytessum = 0;
      }
}
