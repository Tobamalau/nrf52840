#include <stdio.h>
#include <nrf.h>
#include "sdk_config.h"
//#include "sound.h"
#include "opus.h"
//#include "opusFile.c"
#include "opusTobi.h"

/*
struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
struct frame FrameInstanz = {&OpusInstanz, 0, 0};

volatile int NBbytes[] = {249,134,135,258,189,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,
                          104,114,207,175,181,162,184,161,161,161,161,145,177,161,161,157,165,152,137,193,
                          144,179,154,168,161,161,161,161,161,161,161,137,128,161,148,142,166};
*/



void initI2S()
{
   // Enable transmission
   NRF_I2S->CONFIG.TXEN = (I2S_CONFIG_TXEN_TXEN_ENABLE << I2S_CONFIG_TXEN_TXEN_Pos);
   // Enable MCK generator
   NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
   // MCKFREQ
   NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
   // Ratio = 96
   NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos;
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
   // Configure data pointer
   //NRF_I2S->TXD.PTR = (uint32_t)&m_musik_table[0];
   //NRF_I2S->RXTXD.MAXCNT = leftChannelSize;//sizeof(sine_table) / sizeof(uint32_t);//
   //printf("NRF_I2S->RXTXD.MAXCNT:%d,%d,%d", NRF_I2S->RXTXD.MAXCNT,  sizeof(sine_table), sizeof(uint32_t));
}



int main(void)
{
   printf("Start");
   initI2S();

/*
   uint_fast16_t loopcnt = 0;
   uint_fast16_t nbbytessum = 0;
   uint_fast16_t len = sizeof(NBbytes) / sizeof(NBbytes[0]);
   initOpus(&OpusInstanz);
   FrameInstanz.opus_t = &OpusInstanz;
  */
   uint_fast16_t len = 57;
   volatile uint_fast8_t newFrame = 0;
   struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
   struct frame FrameInstanz = {&OpusInstanz, 0, 0};

   getPcm(&FrameInstanz);
   NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes;
   NRF_I2S->RXTXD.MAXCNT = NBbytes[FrameInstanz.loopcnt-1];
   // Start transmitting I2S data
   NRF_I2S->TASKS_START = 1;


   // Since we are not updating the TXD pointer, the sine wave will play over and over again.
   // The TXD pointer can be updated after the EVENTS_TXPTRUPD arrives.
   while (1)
   {

    __WFE();
    while (len>FrameInstanz.loopcnt)
    {
       /*
       OpusInstanz.input = marioTestenc_opus + nbbytessum;
       OpusInstanz.nbBytes = NBbytes[loopcnt];
       decodeOpusFrame(&OpusInstanz);
       nbbytessum += OpusInstanz.nbBytes;
       loopcnt++;*/
       if(newFrame)
       {
          getPcm(&FrameInstanz);
          printf("pcm:%o", OpusInstanz.pcm_bytes[0]);
       }
       /* New I2S buffer*/
       if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
       {
           NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes;
           NRF_I2S->RXTXD.MAXCNT = NBbytes[FrameInstanz.loopcnt-1];
           NRF_I2S->EVENTS_TXPTRUPD = 0;
           newFrame = 1;
       }
    }
    FrameInstanz.loopcnt = 0;
    FrameInstanz.nbbytessum = 0;
   }
}
