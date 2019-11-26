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

#include "nrf_gpiote.h"

#include "nrf_802154_config.h"
#include "nrf_802154.h"

/*###CONFIG###*/
#define VERBOSE 0
#define NOTIMER 1

#define UARTE_RX_BUFF_SIZE 104//(FRAME_SIZE/3 + 4) //hier erst normal nach Packeten suchen und Byteanzahl ermitteln
#define GPIOTE_CHANNEL_0 0
/*802.15.4 defines*/
#define MACHEAD         9
#define PACKHEAD        4
#define PAYLOAD         UARTE_RX_BUFF_SIZE - PACKHEAD//107 
#define DESTINATIONPAN  0x1234
#define DESTINATION     0x5678
#define SOURCE          0x0001
#define MAX_MESSAGE_SIZE (MACHEAD + PACKHEAD + PAYLOAD)
#define CHANNEL         11

#if NOTIMER
const nrf_drv_timer_t TIMER = NRF_DRV_TIMER_INSTANCE(0);
volatile uint16_t timerCnt = 0;
volatile uint16_t timeIEEEsent = 0;
volatile uint16_t timeNewFrameSeq = 0;
#endif
volatile int test = 0;
volatile bool Tx_empty = false;

//struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
unsigned char *txUarteBuffer;
unsigned char rxUarteBuffer[2][UARTE_RX_BUFF_SIZE];
volatile bool UarteInRecive = false;
volatile bool I2sInProgress = false;
volatile bool IEEEReciveActiv = false;
volatile bool SendIEEE = false;
volatile uint8_t UarteBufferPos = 0;
volatile uint8_t UartBufferLoad = 0;
volatile uint8_t DecodeBufferPos = 0;
//uint8_t txBuffer[] = {0x72};
volatile uint8_t newFrame = 0;
static volatile bool IEEE802154_tx_in_progress;
static volatile bool IEEE802154_tx_done;
static volatile uint32_t IEEE802154_rx_counter;
nrfx_uarte_t m_uart = NRFX_UARTE_INSTANCE(0);

void timer_event_handler(nrf_timer_event_t event_type, void* p_context);

void initPeripheral()
{
   uint32_t err;
/*#### I2S ####*/
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

/*#### GPIO ####*/
   nrf_gpio_cfg_input(BSP_BUTTON_0,BUTTON_PULL); //Configure button 0 as input
   nrf_gpio_cfg_output(BSP_LED_0); 
   nrf_gpio_cfg_output(BSP_LED_1);
   nrf_gpio_pin_write(BSP_LED_0, 1);
   nrf_gpio_pin_write(BSP_LED_1, 1);
   nrf_gpiote_event_configure(GPIOTE_CHANNEL_0, BSP_BUTTON_0, NRF_GPIOTE_POLARITY_TOGGLE); 
   NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0
   nrf_gpiote_event_enable(0);
   NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts

/*#### TIMER ####*/
   #if NOTIMER
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
#endif
}
void stopI2S()
{
   IEEEReciveActiv = false;
   I2sInProgress = false;
   NRF_I2S->TASKS_START = 0;
   NRF_I2S->TASKS_STOP = 1;
   *((volatile uint32_t *)0x40025038) = 1;/*Workaround for sdk issu*/
   *((volatile uint32_t *)0x4002503C) = 1;
   //UarteInRecive = true;
   //nrfx_uarte_rx(&m_uart, rxUarteBuffer[UarteBufferPos], sizeof(rxUarteBuffer[UarteBufferPos]));
}
void setMacHead(uint8_t *message)
{
   message[0] = 0x41;                // Set MAC header: short addresses, no ACK 0100 0001
   message[1] = 0x98;                // Set MAC header 1001 1000
   message[3] = DESTINATIONPAN&0xff;
   message[4] = (DESTINATIONPAN>>8)&0xff;
   message[5] = DESTINATION&0xff;
   message[6] = (DESTINATION>>8)&0xff;
   message[7] = SOURCE&0xff;
   message[8] = (SOURCE>>8)&0xff;
}
void nrf_802154_transmitted(const uint8_t * p_frame, uint8_t * p_ack, uint8_t length, int8_t power, uint8_t lqi)
{
   (void) p_frame;
   (void) length;
   (void) power;
   (void) lqi;
   timeIEEEsent = timerCnt;
   IEEE802154_tx_done = true;
   nrf_gpio_pin_toggle(BSP_LED_0);

   if (p_ack != NULL)
   {
     nrf_802154_buffer_free(p_ack);
   }
}
void nrf_802154_transmit_failed(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
   IEEE802154_tx_done = true;
}
void nrf_802154_received(uint8_t * p_data, uint8_t length, int8_t power, uint8_t lqi)
{
   (void) power;
   (void) lqi;

   if (length > MAX_PACKET_SIZE)
     goto exit;
   
   memcpy(rxUarteBuffer[UarteBufferPos], p_data + MACHEAD, (PAYLOAD + PACKHEAD));
   nrfx_uarte_tx(&m_uart, (uint8_t *)rxUarteBuffer[UarteBufferPos], sizeof(rxUarteBuffer[UarteBufferPos]));
   UartBufferLoad |= (1 << UarteBufferPos); //new decode Buffer available
   UarteBufferPos ^= (1 << 0);            //UartBufferLoad &= ~(1 << UarteBufferPos) löschen    

   if(!I2sInProgress)
      newFrame = 1;
   
   nrf_gpio_pin_toggle(BSP_LED_1);
   IEEE802154_rx_counter++;
   IEEEReciveActiv = true; //Nur wenn Opus Packet noch abfragen!
   
exit:
   nrf_802154_buffer_free(p_data);

   return;
}
#if NOTIMER
/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            if(timerCnt%20 == 0 && I2sInProgress)
               SendIEEE = true;
            timerCnt++;
            /*if((timerCnt%200 == 0) && I2sInProgress)
               timerCnt = 0;*/
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
#endif
// Interrupt handler
void GPIOTE_IRQHandler()
{
/*	  timerCnt = 0;
	  NRF_GPIOTE->EVENTS_IN[0] = 0;
          */
}
void m_uart_callback(nrfx_uarte_event_t const * p_event, void * p_context)
{
   switch(p_event->type)
   {
      case NRFX_UARTE_EVT_TX_DONE:  
         
         nrfx_uarte_rx(&m_uart, rxUarteBuffer[UarteBufferPos], sizeof(rxUarteBuffer[UarteBufferPos]));        
         break;

      case NRFX_UARTE_EVT_RX_DONE: 
         UarteInRecive = false;
         UartBufferLoad |= (1 << UarteBufferPos); //new decode Buffer available
         UarteBufferPos ^= (1 << 0);            //UartBufferLoad &= ~(1 << UarteBufferPos) löschen          
         if(UartBufferLoad != 3)
         {
            UarteInRecive = true;
            char txBuffer[] = {'r', 'E', DecodeBufferPos+0x30, UartBufferLoad+0x30, UarteBufferPos+0x30, test+0x30};
            nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer, sizeof(txBuffer));
         }
         if(!I2sInProgress)
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
void IEEE802154_init(uint8_t *message)
{
   uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
   uint8_t short_address[]    = {0x78, 0x56};
   uint8_t pan_id[]           = {0x34, 0x12};

   memset( message, 0, sizeof(message));
   setMacHead(message);
   IEEE802154_tx_in_progress = false;
   IEEE802154_tx_done        = false;

   nrf_802154_init();
   nrf_802154_auto_pending_bit_set(false);
   nrf_802154_auto_ack_set(false);
   nrf_802154_short_address_set(short_address);
   nrf_802154_extended_address_set(extended_address);
   nrf_802154_pan_id_set(pan_id);
   nrf_802154_channel_set(CHANNEL);
   nrf_802154_receive();
}
char* itoaT(int value, char* result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
            tmp_value = value;
            value /= base;
            *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while(ptr1 < ptr) {
            tmp_char = *ptr;
            *ptr--= *ptr1;
            *ptr1++ = tmp_char;
    }
    return result;
}
/*
char* itoaT(int val, int base){
	
	static char buf[32] = {0};
	
	int i = 30;
	
	for(; val && i ; --i, val /= base)
	
		buf[i] = "0123456789abcdef"[val % base];
	
	return &buf[i+1];
	
}*/
int main(void)
{
   uint32_t err;
   err = uarte_init();
   APP_ERROR_CHECK(err);
   initPeripheral();

   uint8_t IEEE802154_message[MAX_MESSAGE_SIZE];
   uint8_t opusPackNb = 0;
   IEEE802154_init(IEEE802154_message);
 
   volatile uint8_t bufferNr = 0;
   struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
   struct frame FrameInstanz = {&OpusInstanz, 0, 0};
   FrameInstanz.nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
   initOpusFrame(&FrameInstanz);
#if VERBOSE == 1
   //printf("Opus/UART Init\n");
   char txBuffer[] = {'S', 't'};
   nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer, sizeof(txBuffer));
#endif

   UarteInRecive = true;
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
               break;
            }          

            FrameInstanz.opus_t->input = rxUarteBuffer[DecodeBufferPos] + 4;//msgBuffer + 4;
            FrameInstanz.opus_t->nbBytes = UARTE_RX_BUFF_SIZE - 4;  
            /*IEEE802.15.4 transmitt*/
            if (!IEEE802154_tx_in_progress && !IEEEReciveActiv)
            {
               memcpy(IEEE802154_message+MACHEAD ,rxUarteBuffer[DecodeBufferPos], (PAYLOAD + PACKHEAD));
               IEEE802154_message[2] = opusPackNb;
               IEEE802154_tx_in_progress = true;
               nrf_802154_transmit_csma_ca(IEEE802154_message, (uint8_t)MAX_MESSAGE_SIZE);
               opusPackNb++;
               SendIEEE = false;
            }           
            UartBufferLoad &= ~(1 << DecodeBufferPos);    //Delet Bufferloadmemory
            DecodeBufferPos ^= (1 << 0);                  //switch to new Uart Buffer

            /*new buffer request from Uarte*/
            if(!UarteInRecive && !IEEEReciveActiv)
            {  
               char tmp[10] = {0};
               char tmp2[10] = {0};
               itoaT(timeIEEEsent, tmp, 10); // Zeit von EVENTS_TXPTRUPD bis IEEE Packet gesendet
               itoaT(timeNewFrameSeq, tmp2, 10); // Zeit von EVENTS_TXPTRUPD bis newFrame Routine abgearbeitet
               UarteInRecive = true;
               char txBuffer[] = {'r', 'W', DecodeBufferPos+0x30, UartBufferLoad+0x30, UarteBufferPos+0x30, '\t' , *tmp, '\t' , *tmp2};    //test+0x30        
               nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer, sizeof(txBuffer));
            }

            if(!getPcm(&FrameInstanz, bufferNr))
               APP_ERROR_CHECK(NRF_ERROR_BUSY);

            if(!I2sInProgress)
            {
               NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
               NRF_I2S->RXTXD.MAXCNT = FRAME_SIZE/2;
               NRF_I2S->TASKS_START = 1;
               I2sInProgress = true;
               SendIEEE = false;
               bufferNr ^= (1 << 0);
            }
            newFrame = 0;
            timeNewFrameSeq = timerCnt;
         }
         /* New I2S buffer*/
         if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
         {
#if VERBOSE == 2
            //printf("\nEVENTS_TXPTRUPD bufferNr:%d\n", bufferNr);
            printf("\r%d ms", timerCnt);
#endif
            timerCnt = 0;
            NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
            NRF_I2S->EVENTS_TXPTRUPD = 0;
            newFrame = 1;
            bufferNr ^= (1 << 0);
         }
         /*IEEE802.15.4 transmitted*/
         if (IEEE802154_tx_done)
         {
            IEEE802154_tx_in_progress = false;
            IEEE802154_tx_done   = false;
         }
      }
      FrameInstanz.loopcnt = 0;
      FrameInstanz.nbbytessum = 0;
      }
}
