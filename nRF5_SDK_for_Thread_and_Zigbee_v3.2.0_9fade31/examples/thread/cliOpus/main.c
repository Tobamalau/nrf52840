/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup cli_example_main main.c
 * @{
 * @ingroup cli_example
 * @brief An example presenting OpenThread CLI.
 *
 */

#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"
#include "mem_manager.h"

#include "thread_utils.h"

#include "opus.h"
#include "opusTobi.h"
#include "youtube48_8_vbr.c"

#include <openthread/thread.h>

#define SCHED_QUEUE_SIZE      32                              /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum app_scheduler event size. */
#define SENDTRIAL 10                                          /* Maximale Sendeversuche */


static const unsigned char UDP_PAYLOAD[]   = "Hello New World!";
static const unsigned char UDP_REQUEST[]   = "r";

int16_t sine_table[] = { 0, 0, 23170, 23170, 32767, 32767, 23170, 23170, 0, 0, -23170, -23170, -32768, -32768, -23170, -23170};

int16_t Nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
int16_t sendLoopCnt = 0;
uint32_t Nbbytessum = 0;
uint8_t bufferNr = 0;
bool OpusPackRequ = false;
bool UpdI2SBuffer = false;
volatile bool OpusPackRequSend = false;

struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
const unsigned char *input, *MsgBuffer;

void setI2SBuffer();
void handleUdpReceive(void *aContext, otMessage *aMessage, 
                      const otMessageInfo *aMessageInfo);
/***************************************************************************************************
 * @section Callbacks
 **************************************************************************************************/

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Stop Streaming");          
            OpusPackRequ = false;
            UpdI2SBuffer = false;
            NRF_I2S->TASKS_START = 0;
            NRF_I2S->TASKS_STOP = 1;
            *((volatile uint32_t *)0x40025038) = 1;/*Workaround for sdk issu*/
            *((volatile uint32_t *)0x4002503C) = 1;
            sendUdp(thread_ot_instance_get(), UDP_PAYLOAD, sizeof(UDP_PAYLOAD));
            break;

        case BSP_EVENT_KEY_1:
            NRF_LOG_INFO("Button 2 pressed");
            sendUdp(thread_ot_instance_get(), opusData, NBbytes[0]);
            break;

        case BSP_EVENT_KEY_2:
            NRF_LOG_INFO("Start Streaming");
            UpdI2SBuffer = true;        
            break;

        case BSP_EVENT_KEY_3:
            NRF_LOG_INFO("Button 4 (send all) pressed");
            int16_t nbbytescnt = sizeof(NBbytes) / sizeof(NBbytes[0]);
            uint32_t nbbytessum = 0;
            input = opusData;
            
            for(int i = 0; i<nbbytescnt; i++)
            {
              sendUdp(thread_ot_instance_get(), input, NBbytes[i]);
              /*while(!otLinkIsInTransmitState(thread_ot_instance_get())){
                  NRF_LOG_INFO("otLinkIsInTransmitState");
                  __WFE();
              }*/
              nbbytessum += NBbytes[i];
              input = opusData + nbbytessum;
            }
            //NRF_LOG_INFO("%d mal UDP send", i);
            break;

          default:
            break;
    }
}

void handleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
   int err;
   uint8_t errorloop = 0;
   uint16_t msgLength = otMessageGetLength(aMessage);
   uint16_t msgOffset = otMessageGetOffset(aMessage);
   unsigned char msgBuffer[msgLength];

   OT_UNUSED_VARIABLE(aContext);
   OT_UNUSED_VARIABLE(aMessageInfo);
   otMessageRead(aMessage, msgOffset, msgBuffer, msgLength);
   NRF_LOG_INFO("UDP Message recived Offset:%d Length:%d\n%s", msgOffset, msgLength, msgBuffer);
   bsp_board_led_invert(1);
   /*nur im ersten Durchlauf*/
   if(OpusPackRequSend)    //Problem, dass nur anfragender Slave neue Daten bekommt!!!
   {
      MsgBuffer = saveOpusPacket(msgBuffer, msgLength);
      OpusPackRequSend = false;
   }
   /*Stream Request empfangen sende n-te Daten*/
   else if(msgBuffer[0] == 0x72)// && otMessageGetLength(aMessage) == 1)      //Problem das alle Teinehmer zurücksenden würden!!
   {
      uint16_t headerlength = 0;
      if(sendLoopCnt>=Nbbytescnt)
      {
         sendLoopCnt = 0;
         Nbbytessum = 0;
      }

      input = opusData + Nbbytessum;
      NRF_LOG_INFO("input:%x,%x, Length: %d", *input, input, NBbytes[sendLoopCnt]);
      do{
         const unsigned char *header = getOpusPacketHeader(OPUSPACKETPERREQUEST, &NBbytes[sendLoopCnt], &headerlength);
         err = sendUdpOpusPacket(thread_ot_instance_get(), input, headerlength, header, HEADERMEMSYZE(OPUSPACKETPERREQUEST));
         if(errorloop == SENDTRIAL)
         {
            NRF_LOG_INFO("send failed after %d trial", SENDTRIAL);
            break;
         }
         errorloop++;
      }while(err != OT_ERROR_NONE);
      Nbbytessum += headerlength;
      sendLoopCnt+= OPUSPACKETPERREQUEST;  
         
   }   
}

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n",
                 flags, otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/
/**@brief Function for initializing I2S Module.
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
   NRF_I2S->PSEL.SDOUT = (I2S_CONFIG_SDOUT_PIN << I2S_PSEL_SDOUT_PIN_Pos);

   NRF_I2S->ENABLE = 1;
   }

/**@brief Function for initializing the Application Timer Module.
 */
static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the LEDs.
 */
static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for deinitializing the Thread Stack.
 *
 */
static void thread_instance_finalize(void)
{
    bsp_thread_deinit(thread_ot_instance_get());
    thread_soft_deinit();
}


/**@brief Function for initializing scheduler module.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
/***************************************************************************************************
 * @section Functions
 **************************************************************************************************/
void setI2SBuffer()
{
   NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];//(uint32_t)&sine_table[0];//
   NRF_I2S->RXTXD.MAXCNT = 960/2;
   NRF_I2S->EVENTS_TXPTRUPD = 0;
   NRF_I2S->TASKS_START = 1;
   UpdI2SBuffer = 1;
   bufferNr ^= (1 << 0);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/
int main(int argc, char *argv[])
{
   int err;
   uint32_t err_code = nrf_mem_init();    //Init Memory Manager vor allocating memory
   APP_ERROR_CHECK(err_code);
   log_init();
   scheduler_init();    //Scheduler speichert die Events?
   timer_init();
   leds_init();

   err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
   APP_ERROR_CHECK(err_code);

   /*Init I2S/Opus decoder*/
   initI2S();
   //NRF_I2S->TXD.PTR = (uint32_t)&sine_table[0];
   //NRF_I2S->RXTXD.MAXCNT = sizeof(sine_table) / sizeof(sine_table);
   
   err = initOpus(&OpusInstanz);
   if(!err)
      NRF_LOG_INFO("initOpus error");

   uint16_t headerlength = 0;
   MsgBuffer = getOpusPacketHeader(OPUSPACKETPERREQUEST, &NBbytes[0], &headerlength);
   if(MsgBuffer == NULL)
   {
      NRF_LOG_INFO("malloc failed");
      NRF_LOG_PROCESS(); //display all Logs
      APP_ERROR_CHECK(NRF_ERROR_NULL);
   }
   uint16_t test = *(MsgBuffer+3)<<8 | *(MsgBuffer+2);
   NRF_LOG_INFO("Opusheader-Pointer: &%x, %d, erstre Wert %d",MsgBuffer, *MsgBuffer, test);
   nrf_free((unsigned char*)MsgBuffer);

   /*while loop*/
   while (true)
   {
      thread_init_Tobi(thread_state_changed_callback);
      initUdp(thread_ot_instance_get(), handleUdpReceive);


      while (!thread_soft_reset_was_requested())
      {

         thread_process();
         app_sched_execute();    //Call app_sched_execute() from the main loop each time the application wakes up
         NRF_LOG_PROCESS(); //display all Logs

         /*Opus Packet Request*/
         if(OpusPackRequ)
         {
            err = sendUdp(thread_ot_instance_get(), UDP_REQUEST, sizeof(UDP_REQUEST));
            if(err == OT_ERROR_NONE)
            {
               OpusPackRequ = true;
            }
            OpusPackRequ = false;
            OpusPackRequSend = true;
         }
         /*Update I2S Buffer*/  
         if(UpdI2SBuffer)
         {
            if(MsgBuffer != NULL)
            {
               OpusInstanz.input = getOpusFrameFromPacket(MsgBuffer, 1);
               OpusInstanz.nbBytes = atoi((const char *)&MsgBuffer[1 + 1]);
               decodeOpusFrame(&OpusInstanz, bufferNr);
               UpdI2SBuffer = false;
            }
         }        
         /* New I2S buffer*/
         if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
         {
            if(OpusPackRequ)  //Request were not served
            {
               NRF_LOG_INFO("I2S Buffer empty");
               NRF_I2S->EVENTS_TXPTRUPD = 0;
               //NRF_I2S->TASKS_STOP = 1;
               //*((volatile uint32_t *)0x40025038) = 1;/*Workaround for sdk failure*/
               //*((volatile uint32_t *)0x4002503C) = 1;
            }
            else
            {
               setI2SBuffer();
            }
         }

      }

      thread_instance_finalize();
   }
}
