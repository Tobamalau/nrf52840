#include <stdio.h>
#include <nrf.h>
#include "sdk_config.h"
#include "sound.h"
#include "marioTestenc.c"
#include "opus.h"

/*The frame size is hardcoded for this sample code but it doesn't have to be*/
#define FRAME_SIZE 960
#define SAMPLE_RATE 48000
#define OPUSCHANNELS 1
//#define APPLICATION OPUS_APPLICATION_AUDIO
#define BITRATE 64000
#define MAX_FRAME_SIZE (6*960)
#define MAX_PACKET_SIZE (3*1276)

int err;
int frame_size;
opus_int16 out[MAX_FRAME_SIZE*OPUSCHANNELS];
unsigned char cbits[MAX_PACKET_SIZE];
int nbBytes = MAX_PACKET_SIZE;
unsigned char pcm_bytes[MAX_FRAME_SIZE*OPUSCHANNELS*2];

struct opusfile {
   OpusDecoder *decoder;
   int nbBytes;
   int max_frame_size;
   int framesize;
   unsigned char *input;
   unsigned int inputLenpth;
   opus_int16 out[MAX_FRAME_SIZE*CHANNELS];
   //unsigned char cbits[MAX_PACKET_SIZE];
} OpusInstanz = {NULL, NBBYTES, MAX_FRAME_SIZE, 0, NULL, MAX_PACKET_SIZE, NULL};

void opus()
{
   /* Create a new decoder state. */
   decoder = opus_decoder_create(SAMPLE_RATE, OPUSCHANNELS, &err);
   if (err<0)
   {
      printf("failed to create decoder: %s\n", opus_strerror(err));
      return;
   }

    /* Decode the data. In this example, frame_size will be constant because
       the encoder is using a constant frame size. However, that may not
       be the case for all encoders, so the decoder must always check
       the frame size returned. */
    frame_size = opus_decode(decoder, cbits, nbBytes, out, MAX_FRAME_SIZE, 0);
    if (frame_size<0)
    {
       fprintf(stderr, "decoder failed: %s\n", opus_strerror(frame_size));
       return;
    }

    /* Convert to little-endian ordering. */
    for(int i=0;i<OPUSCHANNELS*frame_size;i++)
    {
        pcm_bytes[2*i]=out[i]&0xFF;
        pcm_bytes[2*i+1]=(out[i]>>8)&0xFF;
        printf("%o %o\n",  pcm_bytes[2*i], pcm_bytes[2*i+1]);

    }
    /* Write the decoded audio to file. */
}

int_fast8_t decodeOpus(struct opusfile *lib)
{
   lib->framesize = opus_decode(lib->decoder, lib->input, lib->nbBytes, lib->out, lib->max_frame_size, 0);
   if (lib->framesize<0)
   {
    fprintf(stderr, "decoder failed: %s\n", opus_strerror(lib->framesize));
    return EXIT_FAILURE;
   }
   return EXIT_SUCCESS;
}

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
   NRF_I2S->TXD.PTR = (uint32_t)&m_musik_table[0];
   NRF_I2S->RXTXD.MAXCNT = leftChannelSize;//sizeof(sine_table) / sizeof(uint32_t);//
   //printf("NRF_I2S->RXTXD.MAXCNT:%d,%d,%d", NRF_I2S->RXTXD.MAXCNT,  sizeof(sine_table), sizeof(uint32_t));
}



int main(void)
{
   printf("Start");
   initI2S();
   // Start transmitting I2S data
   NRF_I2S->TASKS_START = 1;

   /* Create a new decoder state. */
   OpusInstanz.decoder = opus_decoder_create(SAMPLE_RATE, OPUSCHANNELS, &err);
   if (err<0)
   {
      printf("failed to create decoder: %s\n", opus_strerror(err));
      return 0;
   }
   OpusInstanz.input = &marioTestenc_opus;
   decodeOpus(&OpusInstanz);
   unsigned char pcm_bytes[MAX_FRAME_SIZE*CHANNELS*2];
   /* Convert to little-endian ordering. */
   for(int i=0;i<CHANNELS*OpusInstanz.framesize;i++)
   {
      pcm_bytes[2*i]=OpusInstanz.out[i]&0xFF;
      pcm_bytes[2*i+1]=(OpusInstanz.out[i]>>8)&0xFF;
   }

  // Since we are not updating the TXD pointer, the sine wave will play over and over again.
  // The TXD pointer can be updated after the EVENTS_TXPTRUPD arrives.
  while (1)
  {

    __WFE();
    
  }

  opus_decoder_destroy(OpusInstanz.decoder);
}
