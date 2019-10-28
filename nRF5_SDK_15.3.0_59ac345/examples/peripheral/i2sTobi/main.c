#include <stdio.h>
#include <nrf.h>
#include "sdk_config.h"
//#include "sound.h"
#include "opus.h"
//#include "opusFile.c"
#include "opusTobi.h"

/*Anfang Uart Init*/
#include "boards.h"
#include "app_uart.h"
#define UART_TX_BUF_SIZE                1024                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
/*Ende Uart Init*/

int16_t sine_table[] = { 0, 0, 23170, 23170, 32767, 32767, 23170, 23170, 0, 0, -23170, -23170, -32768, -32768, -23170, -23170};

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
   uint32_t err_t;
   err_t = uart_init();
   

   printf("\n\nUart Init:%ld\n", err_t);
   initI2S();


   //uint_fast16_t len = 57;
   volatile uint8_t newFrame = 1;
   volatile uint8_t bufferNr = 0;
   struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
   struct frame FrameInstanz = {&OpusInstanz, 0, 0};
   initOpusFrame(&FrameInstanz);
   //initOpus(&OpusInstanz);
   printf("Opus                Init\n");

   getPcm(&FrameInstanz, bufferNr);
   //int test =  sizeof(sine_table) / sizeof(uint32_t);

  NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
  //NRF_I2S->RXTXD.MAXCNT = 960;//NBbytes[FrameInstanz.loopcnt-1];
  NRF_I2S->RXTXD.MAXCNT = sizeof(OpusInstanz.pcm_bytes[bufferNr]) / sizeof(uint32_t);
  NRF_I2S->TASKS_START = 1;
  bufferNr ^= (1 << 0);
   /*
     NRF_I2S->TXD.PTR = (uint32_t)&sine_table[0];
  NRF_I2S->RXTXD.MAXCNT = sizeof(sine_table) / sizeof(uint32_t);
   NRF_I2S->TASKS_START = 1;
*/
  

   // Since we are not updating the TXD pointer, the sine wave will play over and over again.
   // The TXD pointer can be updated after the EVENTS_TXPTRUPD arrives.
   
   while (1)
   {
    __WFE();
    //while (FrameInstanz.nbbytescnt>FrameInstanz.loopcnt)
    while (4>FrameInstanz.loopcnt)
    {

       if(newFrame)
       {
          //printf("\ngetPcm bufferNr:%d", bufferNr);
          getPcm(&FrameInstanz, bufferNr);
          newFrame = 0;
       }
       /* New I2S buffer*/
       if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
       {
           //printf("\nEVENT bufferNr:%d\n", bufferNr);
           NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];//(uint32_t)&sine_table[0];//
           NRF_I2S->RXTXD.MAXCNT = sizeof(OpusInstanz.pcm_bytes[bufferNr]) / sizeof(uint32_t);
           NRF_I2S->EVENTS_TXPTRUPD = 0;
           newFrame = 1;
           bufferNr ^= (1 << 0);
       }
    }
    FrameInstanz.loopcnt = 0;
    FrameInstanz.nbbytessum = 0;
   }
}
