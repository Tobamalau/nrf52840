/*header for Opus functions created by Tobias*/

#ifndef OPUS_TOBI_H
#define OPUS_TOBI_H

#include "opus.h"
#include "mem_manager.h"

//#include <stdlib.h>

#define FRAME_SIZE 960
#define APPLICATION OPUS_APPLICATION_AUDIO
//#define BITRATE 64000
#define NBBYTES 249
#define OPUSCHANNELS 1
#define MAX_FRAME_SIZE (6*960)
#define MAX_PACKET_SIZE (3*1276)
#define SAMPLE_RATE 48000   //input Sample Rate of opus file
#define OPUSPACKETIDENTIFIER 0xff
#define OPUSPACKETPERREQUEST 1
#define OPUSPACKETMAXCNT 10
#define HEADERMEMSYZE(X) (X * 2 + 2)



extern int NBbytes[];

struct opus {
   OpusDecoder *decoder;
   int nbBytes;
   const unsigned char *input;
   opus_int16 out[MAX_FRAME_SIZE*OPUSCHANNELS];
   int16_t pcm_bytes[2][MAX_FRAME_SIZE*OPUSCHANNELS];
};

struct frame {
   struct opus *opus_t;
   uint_fast16_t loopcnt;
   uint_fast16_t nbbytessum;
   uint16_t nbbytescnt;
};

int8_t decodeOpusFrame(struct opus *opus_t, uint8_t bufferNr);
int8_t initOpus(struct opus *opus_t);
int8_t initOpusFrame(struct frame *frame_t);
void getPcm(struct frame *frame_t, uint8_t bufferNr);
unsigned char *getOpusPacketHeader(uint8_t framecnt, int *framesize, uint16_t *payloadlength);
bool isOpusPacket(unsigned char *msgBuffer, uint16_t msgLength);
unsigned char *saveOpusPacket(unsigned char *msgBuffer, uint16_t msgLength);
const unsigned char *getOpusFrameFromPacket(const unsigned char *msgBuffer, uint8_t pos);

#endif
