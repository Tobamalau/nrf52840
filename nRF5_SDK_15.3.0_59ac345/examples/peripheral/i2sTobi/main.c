#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>
#include <string.h>

#include "sdk_config.h"
#include "opus.h"
#include "opusTobi.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "nrfx_uarte.h"
#include "nrf_gpiote.h"
#include "nrf_802154_config.h"
#include "nrf_802154.h"
#include "nrf_drv_twi.h"

//#include "nrf_drv_pwm.h"
//#include "app_timer.h"
//#include "nrf_drv_clock.h"

/*###CONFIG###*/
#define VERBOSE 0
#define TIMER_ENABLE 1
#define I2SHAL 1
#define I2C_EN 1
#define CSMACA 0

#define UARTE_RX_BUFF_SIZE 104//(FRAME_SIZE/3 + 4) //hier erst normal nach Packeten suchen und Byteanzahl ermitteln
#define UARTE_TX_BUFF_SIZE 20
#define GPIOTE_CHANNEL_0 0
/*802.15.4 defines*/
#define MACHEAD         2//9
#define PAYLOAD         UARTE_RX_BUFF_SIZE - OPUSPACKHEAD
#define DESTINATIONPAN  0x1234
#define DESTINATION     0x5678
#define SOURCE          0x0001
#define MAX_MESSAGE_SIZE (MACHEAD + OPUSPACKHEAD + PAYLOAD)
#define IEEECHANNEL        11
#define MAXLOSTPACKETS     3
#define IEEETXPOWER        8

/*16 Bit Counter*/
#define _IEEETRANSFAILED   0
#define _IEEECCAFAILED     1
#define _IEEENOTREADY      2
#define _IEEELOSTPACKET    3
#define _IEEECONINTERRUPT  4
/*32 Bit Counter*/
#define _IEEETRANSMITTED   0
#define _IEEERECIVED       1

#if TIMER_ENABLE
const nrf_drv_timer_t TIMER = NRF_DRV_TIMER_INSTANCE(0);
#endif

volatile bool StopAudio = false;
volatile bool PacketIsLost = false;
uint16_t Counter16[5];
uint32_t Counter32[2];
uint32_t RSSISum = 0;
uint32_t RSSIAverage = 0;

volatile uint8_t PacketNum = 0;  //zu Array zusammenfassen
volatile uint16_t timerCnt = 0;
volatile uint16_t timerTotalCnt = 0;
volatile uint16_t timeIEEEsent = 0;
volatile uint16_t timeNewFrameSeq = 0;
volatile uint16_t timelastI2SLoop = 0;

volatile uint8_t bufferNr = 0;
struct opus OpusInstanz = {NULL, NBBYTES, NULL, {}, {}};
unsigned char rxUarteBuffer[3][UARTE_RX_BUFF_SIZE];
char txBuffer[UARTE_TX_BUFF_SIZE];
char txBuffer2[120];
enum _State{Idle, CheckBuffer, SetBuffer, Decode, SeqEnd, AudioStart};
enum _State State = Idle;
volatile bool UarteInRecive = false;
volatile bool I2sInProgress = false;
volatile bool IEEEReciveActiv = false;
volatile bool I2CAbort = false;
volatile uint8_t ReciveBufferPos = 0;
volatile uint8_t ReciveBufferLoad = 0;
volatile uint8_t DecodeBufferPos = 0;
static volatile bool IEEE802154_tx_in_progress;
static volatile bool IEEE802154_tx_done;
volatile bool IEEEnewFrame = false;
uint8_t opusPackNb = 1;
nrfx_uarte_t m_uart = NRFX_UARTE_INSTANCE(0);



#if I2C_EN
#define TWI_INSTANCE_ID     0
#define TDA7901_ADDR        0b1101010    //0xd4d5,0xd6d7,0xd8d9,0xdadb//0b1101000

//#define TDA7901_SUBADDR    
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static volatile bool m_xfer_done = false;
static uint8_t m_sample;
#endif

void timer_event_handler(nrf_timer_event_t event_type, void* p_context);
void m_uart_callback(nrfx_uarte_event_t const * p_event, void * p_context);
void m_uart_context_callback();
void setMacHead(uint8_t *message);
void stopI2S();
uint8_t uarte_init();
void toggleBuffer(volatile uint8_t *buffer);

#if !I2SHAL
static void i2sdata_handler(nrf_drv_i2s_buffers_t const * p_released, uint32_t status);
#endif

#if I2C_EN
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                printf("Temperature: %d Celsius degrees.", m_sample);
            }
            m_xfer_done = true;
            break;
         case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        default:
            break;
    }
}
static void read_sensor_data(uint8_t addr)
{
    m_xfer_done = false;
    ret_code_t err_code;   //0xd4d5,0xd6d7,0xd8d9,0xdadb
   //(void) addr;
     
    err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, &addr, sizeof(addr), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    while (nrf_drv_twi_is_busy(&m_twi));

 /*   err_code = nrf_drv_twi_rx(&m_twi, TDA7901_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);*/

    err_code = nrf_drv_twi_rx(&m_twi, TDA7901_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    while (nrf_drv_twi_is_busy(&m_twi));
}
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}
void TDA7901_set_mode(void)
{
   ret_code_t err_code;
   uint8_t reg[2];
   
   reg[0] = 0x02;
   reg[1] = 0b01100010;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false)
      if(I2CAbort)
         return;
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(reg[0]);

   /* Sample frequency, 16 Bit*/
   reg[0] = 0x03;
   reg[1] = 0b01010100;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(reg[0]);

   /* Volume Control*/
   reg[0] = 0x05;
   reg[1] = 0x7f;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(reg[0]);

   /* Channel 1 LOAD 5 R*/
   reg[0] = 0x11;
   reg[1] = 0b00010001;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(reg[0]);

   /* Channel 1 on*/
   reg[0] = 0x04;
   reg[1] = 0b00010101;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(reg[0]);

   for(int i=0x0c; i<0x26; i++)
   {
      read_sensor_data(i);
   }

   /* Clear fault*/
   reg[0] = 0x21;
   reg[1] = 0b10100000;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(reg[0]);

   for(int i=0x0c; i<0x26; i++)
   {
      read_sensor_data(i);
   }
#if 0
   ret_code_t err_code;
   uint8_t reg[2];
   /* Sample frequency*/
   reg[0] = 0x03;
   reg[1] = 0b01000000;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(0x20);
   /* Amplifier on/off*/
   reg[0] = 0x07;
   reg[1] = 0b00000001;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   if(err_code==NRF_ERROR_DRV_TWI_ERR_ANACK)
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   /* Digital Mute off*/
   reg[0] = 0x02;
   reg[1] = 0b00011000;
   err_code = nrf_drv_twi_tx(&m_twi, TDA7901_ADDR, reg, sizeof(reg), false);
   APP_ERROR_CHECK(err_code);
   while (m_xfer_done == false);
   while (nrf_drv_twi_is_busy(&m_twi));

   read_sensor_data(0x02);

   for(int i=0x20; i<0x42; i++)
   {
      read_sensor_data(i);
   }
#endif
}
#endif
void initPeripheral()
{
/*### Reset Counter Arrays ###*/
   uint32_t err;
   memset( Counter16, 0, sizeof(Counter16)); 
   memset( Counter32, 0, sizeof(Counter32));

#if I2SHAL
/*#### I2S ####*/
   // Enable transmission
   NRF_I2S->CONFIG.TXEN = (I2S_CONFIG_TXEN_TXEN_ENABLE << I2S_CONFIG_TXEN_TXEN_Pos);
   // Enable MCK generator
   NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
   // MCKFREQ
   NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;  //best clock config for 48kHz
   //NRF_I2S->CONFIG.MCKFREQ = 0x30000000  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;                         //clock config for TI AMP because need mclk

   NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X << I2S_CONFIG_RATIO_RATIO_Pos;                  //best clock config for 48kHz
   //NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_128X << I2S_CONFIG_RATIO_RATIO_Pos;                 //clock config for TI AMP because need mclk
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

#else 
   nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;
   config.mck_setup = NRF_I2S_MCK_32MDIV21;
   config.ratio     = NRF_I2S_RATIO_32X;
   err = nrf_drv_i2s_init(&config, i2sdata_handler);
   APP_ERROR_CHECK(err);
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
#endif
/*#### UARTE ####*/
   err = uarte_init();
   APP_ERROR_CHECK(err);
/*#### GPIO ####*/
   nrf_gpio_cfg_input(BSP_BUTTON_0,BUTTON_PULL); //Configure button 0 as input
   nrf_gpio_cfg_input(BSP_BUTTON_1,BUTTON_PULL); //Configure button 0 as input
   nrf_gpio_cfg_output(BSP_LED_0); 
   nrf_gpio_cfg_output(BSP_LED_1);
   nrf_gpio_cfg_output(BSP_LED_2);
   nrf_gpio_cfg_output(BSP_LED_3);
   nrf_gpio_pin_write(BSP_LED_0, 1);
   nrf_gpio_pin_write(BSP_LED_1, 1);
   nrf_gpio_pin_write(BSP_LED_2, 1);
   nrf_gpio_pin_write(BSP_LED_3, 0);
   nrf_gpiote_event_configure(GPIOTE_CHANNEL_0, BSP_BUTTON_0, NRF_GPIOTE_POLARITY_LOTOHI); 
   NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0
   nrf_gpiote_event_enable(0);
   NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts
   
/*#### TIMER ####*/
#if TIMER_ENABLE
   /*Timer Init*/
   uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
   uint32_t time_ticks;
   //Configure TIMER for generating 1ms takt
   nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
   err = nrf_drv_timer_init(&TIMER, &timer_cfg, timer_event_handler);
   APP_ERROR_CHECK(err);
   time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER, time_ms);

   nrf_drv_timer_extended_compare(
      &TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

   nrf_drv_timer_enable(&TIMER);
#endif
#if I2C_EN
   twi_init();
   TDA7901_set_mode();
#endif
}   


/*#### UART ####*/
uint8_t uarte_init()
{
    
    nrfx_uarte_config_t m_uart_config = {//= NRFX_UARTE_DEFAULT_CONFIG;
                      TX_PIN_NUMBER,               ///< TXD pin number.
                      RX_PIN_NUMBER,               ///< RXD pin number.
                      CTS_PIN_NUMBER,              ///< CTS pin number.
                      RTS_PIN_NUMBER,              ///< RTS pin number.
                      //NULL,
                      m_uart_context_callback,                        ///< Context passed to interrupt handler.
                      UARTE_CONFIG_HWFC_Disabled,              ///< Flow control configuration.
                      UARTE_CONFIG_PARITY_Excluded,            ///< Parity configuration.
                      UARTE_BAUDRATE_BAUDRATE_Baud460800,      ///< Baudrate.
                      NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,  ///< Interrupt priority.
    };
    return nrfx_uarte_init(&m_uart, &m_uart_config, m_uart_callback);
}
void IEEE802154_init(uint8_t *message)
{
   uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
   uint8_t short_address[]    = {0x78, 0x56};
   uint8_t pan_id[]           = {0x34, 0x12};

   memset( message, 0, sizeof(message));
   setMacHead(message);
   IEEE802154_tx_in_progress = false;
   IEEE802154_tx_done        = false;

   nrf_802154_init();
   nrf_802154_auto_pending_bit_set(false);
   nrf_802154_auto_ack_set(false);
   nrf_802154_short_address_set(short_address);
   nrf_802154_extended_address_set(extended_address);
   nrf_802154_pan_id_set(pan_id);
   nrf_802154_channel_set(IEEECHANNEL);
   nrf_802154_promiscuous_set(true);
   nrf_802154_receive();
   nrf_802154_tx_power_set(IEEETXPOWER);
}
bool isAudioOff(volatile uint8_t pos)
{
   char str1[10];
   char str2[10];
   strcpy(str1, "EndOfFile");
   memcpy(str2, rxUarteBuffer[pos]+4, 9);
   str2[9] = '\0';
   return strcmp(str1, str2);
}
void stopI2S()
{
   IEEEReciveActiv = false;
   I2sInProgress = false;
   StopAudio = false; 
#if 0
   memset( rxUarteBuffer[0], 0, UARTE_RX_BUFF_SIZE);
   memset( rxUarteBuffer[1], 0, UARTE_RX_BUFF_SIZE);
   memset( rxUarteBuffer[2], 0, UARTE_RX_BUFF_SIZE);
#endif

#if !I2SHAL
   nrf_drv_i2s_stop();
#else
   NRF_I2S->EVENTS_TXPTRUPD = 0;
   NRF_I2S->TASKS_START = 0;
   NRF_I2S->TASKS_STOP = 1;
   *((volatile uint32_t *)0x40025038) = 1;/*Workaround for sdk issu*/
   *((volatile uint32_t *)0x4002503C) = 1;
#endif
   if(!UarteInRecive)
   {
      UarteInRecive = true;
      nrfx_uarte_rx(&m_uart, rxUarteBuffer[ReciveBufferPos], sizeof(rxUarteBuffer[ReciveBufferPos]));
   }               
}
void setMacHead(uint8_t *message)
{
   message[0] = 0b00000001;                // Set MAC header: short addresses, no ACK 0100 0001 (0x41)
   message[1] = 0b00100001;                // Set MAC header 1001 1000  (0x98)
   message[3] = DESTINATIONPAN&0xff;
   message[4] = (DESTINATIONPAN>>8)&0xff;
   message[5] = DESTINATION&0xff;
   message[6] = (DESTINATION>>8)&0xff;
   message[7] = SOURCE&0xff;
   message[8] = (SOURCE>>8)&0xff;
}
void sendStateToUart(char id)
{
   memset( txBuffer, 0, sizeof(txBuffer));
   int length = sprintf(txBuffer, "r%c%d%d%d\t%d\t%d\t%d", id, DecodeBufferPos, ReciveBufferLoad, ReciveBufferPos, timeIEEEsent, timeNewFrameSeq, timelastI2SLoop);
   if(id == 'I')
      txBuffer[length] = '\n';
   UarteInRecive = true;
   nrfx_uarte_rx(&m_uart, rxUarteBuffer[ReciveBufferPos], sizeof(rxUarteBuffer[ReciveBufferPos]));   
   nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer, UARTE_TX_BUFF_SIZE);
}
void nrf_802154_transmitted(const uint8_t * p_frame, uint8_t * p_ack, uint8_t length, int8_t power, uint8_t lqi)
{
   (void) p_frame;
   (void) length;
   (void) power;
   (void) lqi;
   timeIEEEsent = timerCnt;
   IEEE802154_tx_done = true;
   nrf_gpio_pin_toggle(BSP_LED_0);
   opusPackNb++;
   Counter32[_IEEETRANSMITTED]++;
   if (p_ack != NULL)
   {
     nrf_802154_buffer_free(p_ack);
   }
}
void nrf_802154_transmit_failed(const uint8_t * p_frame, nrf_802154_tx_error_t error)
{
   IEEE802154_tx_done = true;
   Counter16[_IEEETRANSFAILED]++;
}
void nrf_802154_cca_failed(nrf_802154_cca_error_t error)
{
   IEEE802154_tx_done = true;
   Counter16[_IEEECCAFAILED]++;
}
void nrf_802154_received(uint8_t * p_data, uint8_t length, int8_t power, uint8_t lqi)
{
   (void) lqi;

   if (length > MAX_MESSAGE_SIZE + 2 || IEEE802154_tx_done || !isOpusPacket(p_data+MACHEAD, (length - MACHEAD - 2)))
     goto exit;
   memcpy(rxUarteBuffer[ReciveBufferPos], p_data + MACHEAD, (PAYLOAD + OPUSPACKHEAD));
   PacketNum = rxUarteBuffer[ReciveBufferPos][1];   
   toggleBuffer(&ReciveBufferPos);
   timeIEEEsent = timerCnt;
   nrf_gpio_pin_toggle(BSP_LED_1);
   ReciveBufferLoad++;
   if(State == Idle && ReciveBufferLoad > 1)
      State = CheckBuffer; 
   Counter32[_IEEERECIVED]++;
   RSSISum+=power;
   RSSIAverage = (power/Counter32[_IEEERECIVED]);
   IEEEReciveActiv = true;
   PacketIsLost = false;
   
   memset( txBuffer2, 0, sizeof(txBuffer2));
   int len = sprintf(txBuffer2, "RecTime:%d\tPackNum:%d\tBufLoad:%d\n", timerTotalCnt, PacketNum, ReciveBufferLoad);
   nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer2, len);
   
exit:
   nrf_802154_buffer_free(p_data);

   return;
}
#if TIMER_ENABLE
/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
   switch (event_type)
   {
     case NRF_TIMER_EVENT_COMPARE0:
         timerCnt++;
         timerTotalCnt++;
         if(timerCnt == 1000)
         {
            timerCnt = 0;
            if(!m_xfer_done)
               I2CAbort = true;
         }
         if(timerTotalCnt == 50000)
            timerTotalCnt = 0;
         break;

     default:
         //Do nothing.
         break;
   }
}
#endif
// Interrupt handler
void GPIOTE_IRQHandler()
{
   memset( txBuffer2, 0, sizeof(txBuffer2));
   int length = sprintf(txBuffer2, "\ntFail:%d\tccaFail:%d\tnotReady:%d\tlostP:%d\tConInt:%d\nTotTrans:%ld\tTotRec:%ld",Counter16[_IEEETRANSFAILED],Counter16[_IEEECCAFAILED],Counter16[_IEEENOTREADY],Counter16[_IEEELOSTPACKET],Counter16[_IEEECONINTERRUPT],Counter32[_IEEETRANSMITTED],Counter32[_IEEERECIVED]);
   txBuffer2[length] = '\n';
   nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer2, length);
   NRF_GPIOTE->EVENTS_IN[0] = 0;
   nrf_gpio_pin_toggle(BSP_LED_3);
}
#if !I2SHAL
static void i2sdata_handler(nrf_drv_i2s_buffers_t const * p_released, uint32_t status)
{
   ASSERT(p_released);
   if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
     return;

   timelastI2SLoop = timerCnt; 
   if(timelastI2SLoop == 0)
     timerCnt = 0; 
   timerCnt = 0;
   timeNewFrameSeq = 0;
   timeIEEEsent = 0;
   nrf_drv_i2s_buffers_t const next_buffers = {
      //.p_rx_buffer = NULL,
      .p_tx_buffer = (uint32_t *)OpusInstanz.pcm_bytes[bufferNr],
   };
   APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));

   State = CheckBuffer;
   toggleBuffer(&bufferNr);
}
#endif
void m_uart_callback(nrfx_uarte_event_t const * p_event, void * p_context)
{
   switch(p_event->type)
   {
      case NRFX_UARTE_EVT_TX_DONE:  
         break;

      case NRFX_UARTE_EVT_RX_DONE: 
         if(!UarteInRecive)
            break;
         if(!isOpusPacket(rxUarteBuffer[ReciveBufferPos], UARTE_RX_BUFF_SIZE) || IEEEReciveActiv)
         {
            sendStateToUart('X');
            break;
         }
         PacketNum = rxUarteBuffer[ReciveBufferPos][1];
         UarteInRecive = false;
         ReciveBufferLoad++;       
         IEEEnewFrame = true;   
         if(State == Idle && ReciveBufferLoad > 2)
            State = CheckBuffer;
         break;

      case NRFX_UARTE_EVT_ERROR:
         break;      
   }
}
void m_uart_context_callback()
{

}
void toggleBuffer(volatile uint8_t *buffer)
{
   (*buffer)++;
   if(*buffer == 3)
      *buffer = 0;
}
int main(void)
{
   bool noBuffer = false;
   initPeripheral();

   uint8_t IEEE802154_message[MAX_MESSAGE_SIZE];
   IEEE802154_init(IEEE802154_message);
 
   struct frame FrameInstanz = {&OpusInstanz, 0, 0};
   initOpusFrame(&FrameInstanz);

   uint8_t lastPacketRec = 0; 
   uint16_t lastLostPacketCnt = 0; 


   UarteInRecive = true;
   nrfx_uarte_rx(&m_uart, rxUarteBuffer[ReciveBufferPos], sizeof(rxUarteBuffer[ReciveBufferPos]));
   while (1)
   {
      /*### Statemachine ###*/
      switch(State)//, , , , , 
      {
         case Idle:  //Idle I2S Off
            break;
         case SetBuffer:  //Wait
            /* New I2S buffer*/
            if(NRF_I2S->EVENTS_TXPTRUPD  != 0)
            {
               timelastI2SLoop = timerCnt; 
               if(timelastI2SLoop == 0)
                 timerCnt = 0; 
               timerCnt = 0;
               NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
               toggleBuffer(&bufferNr);
               NRF_I2S->EVENTS_TXPTRUPD = 0;
               State = CheckBuffer;
            } 
            break;
         case CheckBuffer:  //Buffer check
            if(!ReciveBufferLoad)   //No Buffer
            {
               if(IEEEReciveActiv)
               {
                  if((Counter16[_IEEELOSTPACKET]-lastLostPacketCnt < MAXLOSTPACKETS) && !StopAudio)
                  {
                     //toggleBuffer(&bufferNr);                 //switch to latest Buffer
                     if(Counter16[_IEEELOSTPACKET] == lastLostPacketCnt)
                        lastPacketRec = PacketNum;

                     Counter16[_IEEELOSTPACKET]++;
                     memset( txBuffer2, 0, sizeof(txBuffer2));
                     int length = sprintf(txBuffer2, "Timest: %d\tLostPack:%d\n", timerTotalCnt, Counter16[_IEEELOSTPACKET]);
                     nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer2, length);
                     noBuffer = true;
                     State = Decode;
                  }
                  else
                  {
                     if(!StopAudio)
                        Counter16[_IEEECONINTERRUPT]++;
                     stopI2S();
                     State = Idle;
                  }
               }
               else
               {
                  stopI2S(); 
                  State = Idle;
               }
               nrf_gpio_pin_toggle(BSP_LED_2);
               
            }   
            else  //Buffer available
            {
               if(lastLostPacketCnt != Counter16[_IEEELOSTPACKET])
               {
                  memset( txBuffer2, 0, sizeof(txBuffer2));
                  int length = sprintf(txBuffer2, "LastPack: %d\tNewPack:%d\tLost:%d\tNotready:%d\tTotRec:%ld\n", lastPacketRec, PacketNum, (PacketNum-lastPacketRec-1), Counter16[_IEEENOTREADY], Counter32[_IEEERECIVED]);
                  nrfx_uarte_tx(&m_uart, (uint8_t *)txBuffer2, length);
                  lastLostPacketCnt = Counter16[_IEEELOSTPACKET];
               }
               lastPacketRec = 0;
               if(!isOpusPacket(rxUarteBuffer[DecodeBufferPos], UARTE_RX_BUFF_SIZE))
                  State = SeqEnd;
               else
                  State = Decode;
            }
            break;

         case Decode:  /*### Decode Frame ###*/
            if (!isAudioOff(DecodeBufferPos))  //EndOfFile
               StopAudio = true;
            else
            {
               /*Error Consealment*/
               if(noBuffer)   
               {
                  FrameInstanz.opus_t->input = NULL;
                  FrameInstanz.opus_t->nbBytes = 0;
                  
               }
               else
               {
                  FrameInstanz.opus_t->input = rxUarteBuffer[DecodeBufferPos] + 4;
                  FrameInstanz.opus_t->nbBytes = UARTE_RX_BUFF_SIZE - 4;
               }
                  
               /*new buffer request from Uarte*/
               if(!UarteInRecive && !IEEEReciveActiv)
               {
                  toggleBuffer(&ReciveBufferPos);
                  sendStateToUart('W');
               }

               if(!getPcm(&FrameInstanz, bufferNr))
                  APP_ERROR_CHECK(NRF_ERROR_BUSY);
            }
            if(!noBuffer)   
            {
               ReciveBufferLoad--;
               toggleBuffer(&DecodeBufferPos);
            }
            noBuffer = false;
            if(!I2sInProgress)
               State = AudioStart;
            else
               State = SeqEnd;
            break;         

         case AudioStart:  /*### Start I2S ###*/
#if !I2SHAL
            nrf_drv_i2s_buffers_t const initial_buffers = {
               .p_tx_buffer = (uint32_t *)OpusInstanz.pcm_bytes[bufferNr],
               //.p_rx_buffer = NULL,
            };
            APP_ERROR_CHECK(nrf_drv_i2s_start(&initial_buffers, FRAME_SIZE/2, 0));
#else
            NRF_I2S->TXD.PTR = (uint32_t)OpusInstanz.pcm_bytes[bufferNr];
            NRF_I2S->RXTXD.MAXCNT = FRAME_SIZE/2;
            NRF_I2S->TASKS_START = 1;
#endif
            I2sInProgress = true;
            toggleBuffer(&bufferNr);
            State = SeqEnd;
            break;

         case SeqEnd:  /*### Sequence End ###*/
            timeNewFrameSeq = timerCnt;
            State = SetBuffer;
            break;                      
      }
      /*IEEE802.15.4 transmitted*/
      if (IEEE802154_tx_done)
      {
         IEEE802154_tx_in_progress = false;
         IEEE802154_tx_done   = false;
         if(!UarteInRecive && ReciveBufferLoad < 3)
         {
            toggleBuffer(&ReciveBufferPos);
            sendStateToUart('E');
         }
      }
      /*### IEEE802.15.4 transmitt asynchron ###*/  
      if(!IEEEReciveActiv && IEEEnewFrame)
      {
         if (!IEEE802154_tx_in_progress)
         {
            memcpy(IEEE802154_message+MACHEAD ,rxUarteBuffer[ReciveBufferPos], (PAYLOAD + OPUSPACKHEAD));
            IEEE802154_tx_in_progress = true;
#if CSMACA
            nrf_802154_transmit_csma_ca(IEEE802154_message, (uint8_t)MAX_MESSAGE_SIZE);
            IEEEnewFrame = false;
#else
            if(nrf_802154_transmit(IEEE802154_message, (uint8_t)MAX_MESSAGE_SIZE, false))
               IEEEnewFrame = false;
            else
               Counter16[_IEEENOTREADY]++;
#endif
            
         }
         else if(lastPacketRec != opusPackNb)   //just for debugging
         {
#if CSMACA
            Counter16[_IEEENOTREADY]++; 
#endif
            lastPacketRec = opusPackNb;
         }
      }
      if(!nrf_gpio_pin_read(BSP_BUTTON_1))
      {
         memset( Counter16, 0, sizeof(Counter16)); 
         memset( Counter32, 0, sizeof(Counter32));
      }
   }
}
