/*lib for Opos functions created by Tobias*/

#include "opusTobi.h"
#include "stdbool.h"

//#include "youtube48_8_vbr.c"
//#include "opusFile.c"
#include <stdio.h>
#include <string.h>

int8_t decodeOpusFrame(struct opus *opus_t, uint8_t bufferNr)
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
      opus_t->pcm_bytes[bufferNr][i]=opus_t->out[i];    //Array pcm Bytes kann noch wegrationalisiert werden
#if VERBOSE
      if(i<128 && i > 16*7+12)
      {
        printf("%x,%x\t",opus_t->pcm_bytes[bufferNr][i], opus_t->pcm_bytes[bufferNr][i+1]);
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

int8_t initOpus(struct opus *opus_t)
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

int8_t initOpusFrame(struct frame *frame_t)
{
   int err;
   frame_t->opus_t->decoder = opus_decoder_create(SAMPLE_RATE, OPUSCHANNELS, &err);
   if (err<0)
   {
      printf("failed to create decoder: %s\n", opus_strerror(err));
      return 0;
   }
   //frame_t->nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
   return 1;
}

void getPcm(struct frame *frame_t, uint8_t bufferNr)
{
   //frame_t->opus_t->input = opusData + frame_t->nbbytessum;
   //frame_t->opus_t->nbBytes = NBbytes[frame_t->loopcnt];
   decodeOpusFrame(frame_t->opus_t, bufferNr);
   frame_t->nbbytessum += frame_t->opus_t->nbBytes;
   frame_t->loopcnt++;
}

unsigned char *getOpusPacketHeader(uint8_t framecnt, int *framesize, uint16_t *payloadlength)
{
   //uint8_t headerSize = 1 + 1 + OPUSPACKETPERREQUEST * 2;   //1Byte Typ, 1 Byte Size, 2 Byte/Framesize
   unsigned char *p;
   int memorySize = HEADERMEMSYZE(framecnt);//2 + framecnt * 2;

   p=(unsigned char *) nrf_malloc(memorySize);
   if(p == NULL)
   {
      return NULL;
   }
   //NRF_LOG_INFO("malloc memory: %d Bytes", memorySize);
   uint8_t headerPos = 2;
   p[0] = OPUSPACKETIDENTIFIER;        //Typ
   p[1] = framecnt;   //Size
   for(int i = 0; i < framecnt; i++)
   {
      p[headerPos] = (framesize[i] & 0xff);//uint16 var2 = (uint16) ~((unsigned int)var1);
      p[headerPos+1] = ((framesize[i]>>8) & 0xff);
      *payloadlength += (p[headerPos] + 1)<<8 | (p[headerPos]);
      headerPos += 2;
   }
   return p;
}

bool isOpusPacket(const unsigned char *msgBuffer, uint16_t msgLength)
{
   (void)(msgLength);
   if(!(msgBuffer[0] == OPUSPACKETIDENTIFIER))
      return false;
   uint8_t framecnt = *msgBuffer + 1;
   if((framecnt = 0) || framecnt > OPUSPACKETMAXCNT)
      return false;
/*   if(msgLength != 2 + ...)    //correct meassage length?
      return false;*/
   return true;
}

const unsigned char *saveOpusPacket(const unsigned char *msgBuffer, uint16_t msgLength)
{
   if(!isOpusPacket(msgBuffer, msgLength))
      return NULL;
   unsigned char *p;
   p = (unsigned char *) nrf_malloc(msgLength);
   if(p == NULL)
   {
      return NULL;
   }
   return p;
}

const unsigned char *getOpusFrameFromPacket(const unsigned char *msgBuffer, uint8_t pos)
{
   const unsigned char *p = msgBuffer;
   uint8_t framecnt = *msgBuffer + 1;
   if((framecnt = 0) || pos > framecnt)
      return NULL;
   uint16_t nbbytessum = 0;
   for(int i=1; i<pos; i++)
   {
      nbbytessum += (uint16_t)*msgBuffer + i + 1;//atoi((const char *)&msgBuffer[i + 1]); //hier aus 2Byte eine Zahl machen
   }
   return p + (2 + framecnt * 2 + nbbytessum);
}