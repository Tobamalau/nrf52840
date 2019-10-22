/*lib for Opos functions created by Tobias*/

#include "opusTobi.h"
#include "opusFile.c"
#include <stdio.h>

int NBbytes[] = {249,134,135,258,189,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,
                          104,114,207,175,181,162,184,161,161,161,161,145,177,161,161,157,165,152,137,193,
                          144,179,154,168,161,161,161,161,161,161,161,137,128,161,148,142,166};

int_fast8_t decodeOpusFrame(struct opus *opus_t)
{
   //memset(opus_t->pcm_bytes, '\0', sizeof(opus_t->pcm_bytes));
   int frame_size = opus_decode(opus_t->decoder, opus_t->input, opus_t->nbBytes, opus_t->out, MAX_FRAME_SIZE, 0);
   if (frame_size<0)
   {
    printf("decoder failed: %s\n", opus_strerror(frame_size));
    return 0;
   }
   /* Convert to little-endian ordering.*/
   for(int i=0;i<OPUSCHANNELS*frame_size;i++)
   {
      opus_t->pcm_bytes[2*i]=opus_t->out[i]&0xFF;
      opus_t->pcm_bytes[2*i+1]=(opus_t->out[i]>>8)&0xFF;
#if VERBOSE
      printf("%o\t%o\t",opus_t->pcm_bytes[2*i], opus_t->pcm_bytes[2*i+1]);
      if(i%10 == 0)
         printf("\n");
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

void getPcm(struct frame *frame_t)
{
   frame_t->opus_t->input = marioTestenc_opus + frame_t->nbbytessum;
   frame_t->opus_t->nbBytes = NBbytes[frame_t->loopcnt];
   decodeOpusFrame(frame_t->opus_t);
   frame_t->nbbytessum += frame_t->opus_t->nbBytes;
   frame_t->loopcnt++;
}

