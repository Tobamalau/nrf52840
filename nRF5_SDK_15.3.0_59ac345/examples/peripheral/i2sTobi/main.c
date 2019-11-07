#include <stdio.h>
#include <nrf.h>

#include "sdk_config.h"
//#include "sound.h"
#include "opus.h"
//#include "opusFile.c"
#include "opusTobi.h"

#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "youtube48_8_vbr.c"

#include "nrf_drv_timer.h" //Timer

/*Anfang Uart Init*/
#include "boards.h"
#include "app_uart.h"
#define UART_TX_BUF_SIZE                1024                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
/*Ende Uart Init*/

const nrf_drv_timer_t TIMER = NRF_DRV_TIMER_INSTANCE(0);
volatile uint16_t timerCnt = 0;
//int16_t sine_table[] = { 0, 0, 23170, 23170, 32767, 32767, 23170, 23170, 0, 0, -23170, -23170, -32768, -32768, -23170, -23170};

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
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
	// Look at the UART example.
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static uint32_t uart_init(void)
{
   uint32_t                     err_code;
   const app_uart_comm_params_t comm_params =
   {
     RX_PIN_NUMBER,
     TX_PIN_NUMBER,
     RTS_PIN_NUMBER,
     CTS_PIN_NUMBER,
     APP_UART_FLOW_CONTROL_DISABLED,
     false,
     UART_BAUDRATE_BAUDRATE_Baud115200
   };
   APP_UART_FIFO_INIT( &comm_params,
                    UART_RX_BUF_SIZE,
                    UART_TX_BUF_SIZE,
                    uart_event_handle,
                    APP_IRQ_PRIORITY_MID,
                    err_code);
   return err_code;
   //APP_ERROR_CHECK(err_code);
}

int main(void)
{
   //volatile uint16_t curTime;
   uint32_t err_t;
   err_t = uart_init();
   printf("\n\nUart Init:%ld\n", err_t);
   initI2S();
   /*Timer Init*/
   uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
   uint32_t time_ticks;

   //Configure TIMER for generating 1ms takt
   nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
   err_t = nrf_drv_timer_init(&TIMER, &timer_cfg, timer_event_handler);
   APP_ERROR_CHECK(err_t);

   time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER, time_ms);

   nrf_drv_timer_extended_compare(
      &TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

   nrf_drv_timer_enable(&TIMER);
         
  
   //uint_fast16_t len = 57;
   volatile uint8_t newFrame = 1;
   volatile uint8_t bufferNr = 0;
   struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
   struct frame FrameInstanz = {&OpusInstanz, 0, 0};
   FrameInstanz.nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
   initOpusFrame(&FrameInstanz);
 
   printf("Opus                Init\n");

   FrameInstanz.opus_t->input = opusData + FrameInstanz.nbbytessum;
   FrameInstanz.opus_t->nbBytes = NBbytes[FrameInstanz.loopcnt];
   getPcm(&FrameInstanz, bufferNr);
   //int test =  sizeof(sine_table) / sizeof(uint32_t);

   NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
   //NRF_I2S->RXTXD.MAXCNT = 960;//NBbytes[FrameInstanz.loopcnt-1];
   NRF_I2S->RXTXD.MAXCNT = 960/2;//sizeof(OpusInstanz.pcm_bytes[bufferNr]) / sizeof(uint32_t);
   NRF_I2S->TASKS_START = 1;
   bufferNr ^= (1 << 0);
 

   // Since we are not updating the TXD pointer, the sine wave will play over and over again.
   // The TXD pointer can be updated after the EVENTS_TXPTRUPD arrives.
   
   while (1)
   {

    //__WFE();
    while (FrameInstanz.nbbytescnt>FrameInstanz.loopcnt)
    //while (4>FrameInstanz.loopcnt)
    {
       if(newFrame)
       {
          //printf("\ngetPcm bufferNr:%d", bufferNr);
          FrameInstanz.opus_t->input = opusData + FrameInstanz.nbbytessum;
          FrameInstanz.opus_t->nbBytes = NBbytes[FrameInstanz.loopcnt];
          getPcm(&FrameInstanz, bufferNr);
          newFrame = 0;
       }
       /* New I2S buffer*/
       if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
       {
           //printf("\nEVENT bufferNr:%d\n", bufferNr);
           printf("\r%d ms", timerCnt);
           NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];//(uint32_t)&sine_table[0];//
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
