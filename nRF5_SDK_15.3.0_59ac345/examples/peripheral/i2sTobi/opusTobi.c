/*lib for Opos functions created by Tobias*/

#include "opusTobi.h"
#include "youtube48_8_vbr.c"
//#include "opusFile.c"
#include <stdio.h>
#include <string.h>
/*
int NBbytes[] = {249,134,135,258,189,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,
                          104,114,207,175,181,162,184,161,161,161,161,145,177,161,161,157,165,152,137,193,
                          144,179,154,168,161,161,161,161,161,161,161,137,128,161,148,142,166};
*/
int_fast8_t decodeOpusFrame(struct opus *opus_t, uint8_t bufferNr)
{
   //memset(&opus_t->pcm_bytes[bufferNr], '\0', sizeof(opus_t->pcm_bytes[bufferNr]));
   //getopus_decoder_ctl(opus_t->decoder, OPUS_SET_BITRATE(10630));
   int frame_size = opus_decode(opus_t->decoder, opus_t->input, opus_t->nbBytes, opus_t->out, MAX_FRAME_SIZE, 0);
   if (frame_size<0)
   {
    printf("decoder failed: %s\n", opus_strerror(frame_size));
    return 0;
   }
   opus_int32 rate;
   opus_decoder_ctl(opus_t->decoder, OPUS_GET_BITRATE(&rate));
   //printf("\nOPUS_GET_BITRATE:%ld ", rate);
   /* Convert to little-endian ordering.*/
#if VERBOSE
   printf("\nframe_size:%d, bufferNr:%d\n", frame_size, bufferNr);
   int printcnr = 0;
#endif
   for(int i=0;i<OPUSCHANNELS*frame_size;i++)
   {
#if 0
      int16_t a = ((opus_t->out[i]&0xFF)<<8) | ((opus_t->out[i]>>8)&0xFF);
      if (bufferNr) {
        a= 0x00aa;
      } else {
        a=0xff00;
      }
#endif
      //opus_t->pcm_bytes[bufferNr][i] = a;
      opus_t->pcm_bytes[bufferNr][i]=opus_t->out[i];
      //opus_t->pcm_bytes[bufferNr][2*i+1]=(opus_t->out[i]>>8)&0xFF;
#if VERBOSE
      if(i<128 && i > 16*7+12)
      {
        printf("%x,%x\t",opus_t->pcm_bytes[bufferNr][2*i], opus_t->pcm_bytes[bufferNr][2*i+1]);
        if(printcnr%7 == 0 && printcnr!= 0)
        {
          printf("\n");
          printcnr = 0;
          continue;
        }
        printcnr++;
      }
#endif
   }
   return 1;
}

int initOpus(struct opus *opus_t)
{
   int err;
   opus_t->decoder = opus_decoder_create(SAMPLE_RATE, OPUSCHANNELS, &err);
   if (err<0)
   {
      printf("failed to create decoder: %s\n", opus_strerror(err));
      return 0;
   }
   return 1;
}

int initOpusFrame(struct frame *frame_t)
{
   int err;
   frame_t->opus_t->decoder = opus_decoder_create(SAMPLE_RATE, OPUSCHANNELS, &err);
   if (err<0)
   {
      printf("failed to create decoder: %s\n", opus_strerror(err));
      return 0;
   }
   frame_t->nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
   return 1;
}

void getPcm(struct frame *frame_t, uint8_t bufferNr)
{
   frame_t->opus_t->input = opusData + frame_t->nbbytessum;
   frame_t->opus_t->nbBytes = NBbytes[frame_t->loopcnt];
   decodeOpusFrame(frame_t->opus_t, bufferNr);
   frame_t->nbbytessum += frame_t->opus_t->nbBytes;
   frame_t->loopcnt++;
}

