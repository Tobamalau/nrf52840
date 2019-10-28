/*header for Opus functions created by Tobias*/

#ifndef OPUS_TOBI_H
#define OPUS_TOBI_H

#include "opus.h"


#define FRAME_SIZE 960
#define APPLICATION OPUS_APPLICATION_AUDIO
//#define BITRATE 64000
#define NBBYTES 249
#define OPUSCHANNELS 1
#define MAX_FRAME_SIZE (6*960)
#define MAX_PACKET_SIZE (3*1276)
#define SAMPLE_RATE 48000   //input Sample Rate of opus file

#define VERBOSE 1

extern int NBbytes[];

struct opus {
   OpusDecoder *decoder;
   int nbBytes;
   const unsigned char *input;
   opus_int16 out[MAX_FRAME_SIZE*OPUSCHANNELS];
   unsigned char pcm_bytes[2][MAX_FRAME_SIZE*OPUSCHANNELS*2];
};

struct frame {
   struct opus *opus_t;
   uint_fast16_t loopcnt;
   uint_fast16_t nbbytessum;
   uint16_t nbbytescnt;
};

int_fast8_t decodeOpusFrame(struct opus *opus_t, uint8_t bufferNr);
int initOpus(struct opus *opus_t);
int initOpusFrame(struct frame *frame_t);
void getPcm(struct frame *frame_t, uint8_t bufferNr);

#endif
