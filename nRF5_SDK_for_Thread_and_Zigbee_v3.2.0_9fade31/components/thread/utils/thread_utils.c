/**
 * Copyright (c) 2018 - 2019, Nordic Semiconductor ASA
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

#include "thread_utils.h"

#include "app_util_platform.h"
#include "nrf_assert.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_soc.h"
#include "sdk_config.h"
#include <assert.h>

#if defined(MULTIPROTOCOL_802154_CONFIG_PRESENT)
#include "multiprotocol_802154_config.h"
#endif

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/link.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/platform/openthread-system.h>


#include <openthread/dataset_ftd.h>
#include <openthread/code_utils.h>

#define UDP_PORT 1212
#define THREAD_CONFIG 1

static const char UDP_DEST_ADDR[] = "ff02::1";
//static const char UDP_DEST_ADDR[] = "fdde:ad00:beef:0:bb1:ebd6:ad10:f33";



/**@brief Pointer to the OpenThread instance. */
static otInstance * mp_ot_instance;

static otUdpSocket sUdpSocket;
//void handleUdpReceive(void *aContext, otMessage *aMessage, 
//                      const otMessageInfo *aMessageInfo);


void thread_init_Tobi(otStateChangedCallback handler)
{
    otError error;
    otSysInit(0, NULL);

    mp_ot_instance = otInstanceInitSingle();
    ASSERT(mp_ot_instance != NULL);

    NRF_LOG_INFO("Thread version: %s", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s",
                 (uint32_t)otThreadGetNetworkName(mp_ot_instance));
    otCliUartInit(mp_ot_instance);
    thread_state_changed_callback_set(handler);
    setNetworkConfiguration(mp_ot_instance);
    //otThreadSetRouterRoleEnabled(mp_ot_instance, false);

    error = otIp6SetEnabled(mp_ot_instance, true); /* Start the Thread network interface (CLI cmd > ifconfig up) */
    ASSERT(error == OT_ERROR_NONE);
            error = otThreadSetEnabled(mp_ot_instance, true); /* Start the Thread stack (CLI cmd > thread start) */
    ASSERT(error == OT_ERROR_NONE);

    NRF_LOG_INFO("Thread interface has been enabled.");
    NRF_LOG_INFO("802.15.4 Channel : %d", otLinkGetChannel(mp_ot_instance));
    NRF_LOG_INFO("802.15.4 PAN ID  : 0x%04x", otLinkGetPanId(mp_ot_instance));
    NRF_LOG_INFO("Radio mode:      : %s", otThreadGetLinkMode(mp_ot_instance).mRxOnWhenIdle ?
                                    "rx-on-when-idle" : "rx-off-when-idle");
    NRF_LOG_DEBUG("Debug Log Info");
    
    
}

void thread_init(const thread_configuration_t * p_config)
{
    otError error;

    otSysInit(0, NULL);

#if defined(MULTIPROTOCOL_802154_CONFIG_PRESENT) && defined(MULTIPROTOCOL_802154_MODE)
    uint32_t retval = multiprotocol_802154_mode_set((multiprotocol_802154_mode_t)MULTIPROTOCOL_802154_MODE);
    ASSERT(retval == NRF_SUCCESS);
#endif

    mp_ot_instance = otInstanceInitSingle();
    ASSERT(mp_ot_instance != NULL);

    NRF_LOG_INFO("Thread version: %s", (uint32_t)otGetVersionString());
    NRF_LOG_INFO("Network name:   %s",
                 (uint32_t)otThreadGetNetworkName(mp_ot_instance));

    if (!otDatasetIsCommissioned(mp_ot_instance) && p_config->autocommissioning)
    {
        error = otLinkSetChannel(mp_ot_instance, THREAD_CHANNEL);
        ASSERT(error == OT_ERROR_NONE);

        error = otLinkSetPanId(mp_ot_instance, THREAD_PANID);
        ASSERT(error == OT_ERROR_NONE);
    }

    if (!p_config->autostart_disable)
    {
        otLinkModeConfig mode;
        memset(&mode, 0, sizeof(mode));

        if (p_config->radio_mode == THREAD_RADIO_MODE_RX_OFF_WHEN_IDLE)
        {
            mode.mRxOnWhenIdle       = false; // Join network as SED.
            mode.mSecureDataRequests = true;

            error = otLinkSetPollPeriod(mp_ot_instance, p_config->poll_period);
            ASSERT(error == OT_ERROR_NONE);
        }
        else
        {
            mode.mRxOnWhenIdle       = true;
            mode.mSecureDataRequests = true;
#ifdef OPENTHREAD_FTD
            mode.mDeviceType         = true;
            mode.mNetworkData        = true;
#endif
        }

        error = otThreadSetLinkMode(mp_ot_instance, mode);
        ASSERT(error == OT_ERROR_NONE);

        if (p_config->default_child_timeout != 0)
        {
            otThreadSetChildTimeout(mp_ot_instance, p_config->default_child_timeout);
        }

        error = otIp6SetEnabled(mp_ot_instance, true); /* Start the Thread network interface (CLI cmd > ifconfig up) */
        ASSERT(error == OT_ERROR_NONE);

        if (otDatasetIsCommissioned(mp_ot_instance) || p_config->autocommissioning)
        {
            error = otThreadSetEnabled(mp_ot_instance, true); /* Start the Thread stack (CLI cmd > thread start) */
            ASSERT(error == OT_ERROR_NONE);

            NRF_LOG_INFO("Thread interface has been enabled.");
            NRF_LOG_INFO("802.15.4 Channel : %d", otLinkGetChannel(mp_ot_instance));
            NRF_LOG_INFO("802.15.4 PAN ID  : 0x%04x", otLinkGetPanId(mp_ot_instance));
            NRF_LOG_INFO("Radio mode:      : %s", otThreadGetLinkMode(mp_ot_instance).mRxOnWhenIdle ?
                                            "rx-on-when-idle" : "rx-off-when-idle");
        }
    }
}

void thread_cli_init(void)
{
    ASSERT(mp_ot_instance != NULL);

    otCliUartInit(mp_ot_instance);
    otDiagInit(mp_ot_instance);
}

void thread_deinit(void)
{
    ASSERT(mp_ot_instance != NULL);

    otInstanceFinalize(mp_ot_instance);
    otSysDeinit();
    mp_ot_instance = NULL;
}

void thread_soft_deinit(void)
{
    ASSERT(mp_ot_instance != NULL);

    otInstanceFinalize(mp_ot_instance);
    mp_ot_instance = NULL;
}

void thread_process(void)
{
    ASSERT(mp_ot_instance != NULL);

    otTaskletsProcess(mp_ot_instance);
    otSysProcessDrivers(mp_ot_instance);
}

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
static void fpu_sleep_prepare(void)
{
    uint32_t original_fpscr;

    CRITICAL_REGION_ENTER();
    original_fpscr = __get_FPSCR();
    /*
     * Clear FPU exceptions.
     * Without this step, the FPU interrupt is marked as pending,
     * preventing system from sleeping. Exceptions cleared:
     * - IOC - Invalid Operation cumulative exception bit.
     * - DZC - Division by Zero cumulative exception bit.
     * - OFC - Overflow cumulative exception bit.
     * - UFC - Underflow cumulative exception bit.
     * - IXC - Inexact cumulative exception bit.
     * - IDC - Input Denormal cumulative exception bit.
     */
    __set_FPSCR(original_fpscr & ~0x9Fu);
    __DMB();
    NVIC_ClearPendingIRQ(FPU_IRQn);
    CRITICAL_REGION_EXIT();

    /*
     * The last chance to indicate an error in FPU to the user
     * as the FPSCR is now cleared
     *
     * This assert is related to previous FPU operations
     * and not power management.
     *
     * Critical FPU exceptions signaled:
     * - IOC - Invalid Operation cumulative exception bit.
     * - DZC - Division by Zero cumulative exception bit.
     * - OFC - Overflow cumulative exception bit.
     */

    ASSERT((original_fpscr & 0x7) == 0);
}
#endif // (__FPU_PRESENT == 1) && (__FPU_USED == 1)

void thread_sleep(void)
{
    ASSERT(mp_ot_instance != NULL);

    // Enter sleep state if no more tasks are pending.
    if (!otTaskletsArePending(mp_ot_instance))
    {
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
        fpu_sleep_prepare();
#endif

#ifdef SOFTDEVICE_PRESENT
        ret_code_t err_code = sd_app_evt_wait();
        ASSERT(err_code == NRF_SUCCESS);
#else
        __WFE();
#endif
    }
}

otInstance * thread_ot_instance_get(void)
{
    ASSERT(mp_ot_instance != NULL);

    return mp_ot_instance;
}

void thread_state_changed_callback_set(otStateChangedCallback handler)
{
    ASSERT(mp_ot_instance != NULL);

    otError error = otSetStateChangedCallback(mp_ot_instance, handler, mp_ot_instance);
    ASSERT(error == OT_ERROR_NONE);
}

bool thread_soft_reset_was_requested(void)
{
    return otSysPseudoResetWasRequested();
}


/**
 *  UDP Functions
 **/
/**
 * Initialize UDP socket
 */
void initUdp(otInstance *aInstance, otUdpReceive reciveHandler)
{
    otError       error = OT_ERROR_NONE;
    otSockAddr  listenSockAddr;

    memset(&sUdpSocket, 0, sizeof(sUdpSocket));
    memset(&listenSockAddr, 0, sizeof(listenSockAddr));

    listenSockAddr.mPort    = UDP_PORT;
   // listenSockAddr.mScopeId = OT_NETIF_INTERFACE_ID_THREAD;

    //error = otUdpOpen(aInstance, &sUdpSocket, handleUdpReceive, aInstance);
    error = otUdpOpen(aInstance, &sUdpSocket, reciveHandler, aInstance);
    
    NRF_LOG_INFO("otUdpOpen: %d", error);
    error = otUdpBind(&sUdpSocket, &listenSockAddr);
    NRF_LOG_INFO("otUdpBind: %d", error);

}

/**
 * Send a UDP datagram
 */
int sendUdp(otInstance *aInstance, const unsigned char *payload, uint16_t payloadLength)
{
    otError       error = OT_ERROR_NONE;
    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  destinationAddr;
    if(otLinkIsInTransmitState(aInstance))
    {
        NRF_LOG_INFO("otLinkIsInTransmitState");
        return -1;
    }
    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(UDP_DEST_ADDR, &destinationAddr);
    messageInfo.mPeerAddr    = destinationAddr;
    messageInfo.mPeerPort    = UDP_PORT;
    messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;  //Neccesary for this version of thread in newer version it's not needed

    message = otUdpNewMessage(aInstance, NULL);
    otEXPECT_ACTION(message != NULL, error = OT_ERROR_NO_BUFS);

    error = otMessageAppend(message, payload, payloadLength);
    otEXPECT(error == OT_ERROR_NONE);

    error = otUdpSend(&sUdpSocket, message, &messageInfo);
    //error = otUdpSendDatagram(aInstance, message, &messageInfo);

 exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        NRF_LOG_INFO("Error: otUdpSend %d", error);
        otMessageFree(message);  //Muss das nicht auch noch gemacht werden????
    }
    return OT_ERROR_NONE;
}

/**
 * Send a UDP datagram Opus Packet
 */
int sendUdpOpusPacket(otInstance *aInstance, const unsigned char *payload, uint16_t payloadLength, const unsigned char *header, uint16_t headerLength)
{
    otError       error = OT_ERROR_NONE;
    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  destinationAddr;
    if(otLinkIsInTransmitState(aInstance))
    {
        NRF_LOG_INFO("otLinkIsInTransmitState");
        return -1;
    }
    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(UDP_DEST_ADDR, &destinationAddr);
    messageInfo.mPeerAddr    = destinationAddr;
    messageInfo.mPeerPort    = UDP_PORT;
    messageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;  //Neccesary for this version of thread in newer version it's not needed

    message = otUdpNewMessage(aInstance, NULL);
    //otEXPECT_ACTION(message != NULL, error = OT_ERROR_NO_BUFS);
        if (message == NULL || error == OT_ERROR_NO_BUFS)
    {
        NRF_LOG_INFO("Error: otUdpSend %d", error);
        otMessageFree(message);
    }
    printotBufferInfo();
    error = otMessageAppend(message, header, headerLength);
    //otEXPECT(error == OT_ERROR_NONE);
        if (error != OT_ERROR_NONE && message != NULL)
    {
        NRF_LOG_INFO("Error: otUdpSend %d", error);
        otMessageFree(message);
    }
    printotBufferInfo();
    error = otMessageAppend(message, payload, payloadLength);
    //otEXPECT(error == OT_ERROR_NONE);
        if (error != OT_ERROR_NONE && message != NULL)
    {
        NRF_LOG_INFO("Error: otUdpSend %d", error);
        otMessageFree(message);
    }

    error = otUdpSend(&sUdpSocket, message, &messageInfo);
    //error = otUdpSendDatagram(aInstance, message, &messageInfo);

 //exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        NRF_LOG_INFO("Error: otUdpSend %d", error);
        otMessageFree(message);
    }
    
    return OT_ERROR_NONE;
}

/**
 * Override default network settings, such as panid, so the devices can join a network
 */
void setNetworkConfiguration(otInstance *aInstance)
{
    static char          aNetworkName[] = "OTCodelab";
    otOperationalDataset aDataset;

    memset(&aDataset, 0, sizeof(otOperationalDataset));
   
    /*
     * Fields that can be configured in otOperationDataset to override defaults:
     *     Network Name, Mesh Local Prefix, Extended PAN ID, PAN ID, Delay Timer,
     *     Channel, Channel Mask Page 0, Network Master Key, PSKc, Security Policy
     */
    aDataset.mActiveTimestamp                      = 1;
    aDataset.mComponents.mIsActiveTimestampPresent = true;
    //aDataset.mSecurityPolicy 
    /* Set Channel to 15 */
    aDataset.mChannel                      = 11;
    aDataset.mComponents.mIsChannelPresent = true;
    
    /* Set Pan ID to 2222 */
    aDataset.mPanId                      = (otPanId)0x2222;
    aDataset.mComponents.mIsPanIdPresent = true;

    /* Set Extended Pan ID to C0DE1AB5C0DE1AB5 */
    uint8_t extPanId[OT_EXT_PAN_ID_SIZE] = {0xC0, 0xDE, 0x1A, 0xB5, 0xC0, 0xDE, 0x1A, 0xB5};
    memcpy(aDataset.mExtendedPanId.m8, extPanId, sizeof(aDataset.mExtendedPanId));
    aDataset.mComponents.mIsExtendedPanIdPresent = true;
    
    /* Set master key to 1234C0DE1AB51234C0DE1AB51234C0DE */
    uint8_t key[OT_MASTER_KEY_SIZE] = {0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5, 0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5};
    memcpy(aDataset.mMasterKey.m8, key, sizeof(aDataset.mMasterKey));
    aDataset.mComponents.mIsMasterKeyPresent = true;

    aDataset.mComponents.mIsSecurityPolicyPresent = false;

    /* Set Network Name to OTCodelab */
    size_t length = strlen(aNetworkName);
    assert(length <= OT_NETWORK_NAME_MAX_SIZE);
    memcpy(aDataset.mNetworkName.m8, aNetworkName, length);
    aDataset.mComponents.mIsNetworkNamePresent = true;

#if OPENTHREAD_FTD
    otDatasetSetActive(aInstance, &aDataset);
    
    /* Set the router selection jitter to override the 2 minute default.
       CLI cmd > routerselectionjitter 20
       Warning: For demo purposes only - not to be used in a real product */
    uint8_t jitterValue = 20;
    otThreadSetRouterSelectionJitter(aInstance, jitterValue);
#else
    OT_UNUSED_VARIABLE(aInstance);
#endif
}
void printotBufferInfo()
{  
   otBufferInfo info;
   otMessageGetBufferInfo(thread_ot_instance_get(), &info);
   NRF_LOG_WARNING("BufferInfo:\nmTotalBuffers:%d\tmFreeBuffers:%d\r\n\
                              m6loSendMessages:%d\tm6loSendBuffers:%d\r\n\
                              m6loReassemblyMessages:%d\tm6loReassemblyBuffers:%d\r\n",

   info.mTotalBuffers,
   info.mFreeBuffers,
   info.m6loSendMessages,
   info.m6loSendBuffers,
   info.m6loReassemblyMessages,
   info.m6loReassemblyBuffers);
   NRF_LOG_WARNING("\nmIp6Messages:%d\tmIp6Buffers:%d\r\n\
                              mMplMessages:%d\tmMplBuffers:%d\r\n\
                              mMleMessages:%d\tmMleBuffers:%d\r\n",

   info.mIp6Messages,
   info.mIp6Buffers,
   info.mMplMessages,
   info.mMplBuffers,
   info.mMleMessages,
   info.mMleBuffers);
   NRF_LOG_WARNING("\nmArpMessages:%d\tmArpBuffers:%d\r\n\
                              mCoapMessages:%d\tmCoapBuffers:%d\r\n\
                              mCoapSecureMessages:%d\tmCoapSecureBuffers:%d\r\n",

   info.mArpMessages,
   info.mArpBuffers,
   info.mCoapMessages,
   info.mCoapBuffers,
   info.mCoapSecureMessages,
   info.mCoapSecureBuffers);
   NRF_LOG_WARNING("\nmApplicationCoapMessages:%d\tmApplicationCoapBuffers:%d\r\n",

   info.mApplicationCoapMessages,
   info.mApplicationCoapBuffers); 
}


