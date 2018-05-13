/*
 * usbmodule.c
 *
 *  Created on: 28 июл. 2017 г.
 *      Author: walery
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

#include "driverlib/sysctl.h"

#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "usb_bulk_structs.h"
//--------------

#include "inc/typedefs.h"
#include "usbmodule.h"

#include "exchange/status.h"
#include "msmotor/sSegment.h"
#include "msmotor/msport.h"
#include "msmotor/mempool.h"
#include "orderlyTask.h"
//------------- defs

//-------------- vars
//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
volatile uint32_t packet_counter = 0;

volatile uint32_t size_list[3];

static uint8_t *pvMsgData_tmp;

static uint8_t message;
//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

//-------------- function
//#define  sendStatus_p_NO


#ifdef sendStatus_p

uint32_t
sendStatus()
{
    uint32_t ui32Loop, ui32Space, ui32Count;

    uint32_t ui32WriteIndex;
    uint32_t ui32ReadIndex;

    tUSBRingBufObject sTxRing;

    packet_counter = 0;
    g_ui32RxCount = 0;

    //
    // Get the current buffer information to allow us to write directly to
    // the transmit buffer (we already have enough information from the
    // parameters to access the receive buffer directly).
    //
    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);

    //
    // How much space is there in the transmit buffer?
    //
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    ui32WriteIndex = sTxRing.ui32WriteIndex;
    uint32_t tmpr,i;
    union{
        uint8_t* bstatus;
        uint32_t* mstatus;  //[8]
        struct Status_t* cstatus;
    }stunion;

    stunion.cstatus = getStatus();

    i = 0;

    ui32Loop = sizeof(struct Status_t);
    ui32Count = ui32Loop;

    //
    // Set up to process the characters by directly accessing the USB buffers.
    //
    ui32ReadIndex = (uint32_t)(pvMsgData_tmp - g_pui8USBRxBuffer);

    while(ui32Loop)
    {

        g_pui8USBTxBuffer[ui32WriteIndex] = stunion.bstatus[i];
        i++;

        ui32WriteIndex++;
        ui32WriteIndex = (ui32WriteIndex == BULK_BUFFER_SIZE) ?
                0 : ui32WriteIndex;

        ui32Loop--;
    }

    //
    // We've processed the data in place so now send the processed data
    // back to the host.
    //
    USBBufferDataWritten(&g_sTxBuffer, ui32Count);

    //
    // We processed as much data as we can directly from the receive buffer so
    // we need to return the number of bytes to allow the lower layer to
    // update its read pointer appropriately.
    //
    return(ui32Count);
}
#endif


#define CHAR_SEND
//*****************************************************************************
//
// Receive new data and echo it back to the host.
//
// \param psDevice points to the instance data for the device whose data is to
// be processed.
// \param pui8Data points to the newly received data in the USB receive buffer.
// \param ui32NumBytes is the number of bytes of data available to be processed.
//
// This function is called whenever we receive a notification that data is
// available from the host. We read the data, byte-by-byte and swap the case
// of any alphabetical characters found then write it back out to be
// transmitted back to the host.
//
// \return Returns the number of bytes of data processed.
//
//*****************************************************************************
static uint32_t
EchoNewDataToHost(tUSBDBulkDevice *psDevice, uint8_t *pui8Data,
                  uint32_t ui32NumBytes)
{
    uint32_t ui32Loop, ui32Space, ui32Count = 0;
    uint32_t ui32ReadIndex;
    uint32_t ui32WriteIndex;
    tUSBRingBufObject sTxRing;

//    struct Status_t* tx_status;
    uint8_t* tx_status;

//    size_list[packet_counter] = ui32NumBytes;

    // Копирование полученных данных в накопительный буфер.uint8_t cmdBuffer_usb
    size_list[packet_counter] = USBBufferRead(&g_sRxBuffer, &cmdBuffer_usb[g_ui32RxCount], ui32NumBytes);

    g_ui32RxCount += ui32NumBytes;
    packet_counter++;
//
//    if(packet_counter%3 != 0){
//        return g_ui32RxCount;
//    }
    if(g_ui32RxCount < sizeof(struct ComDataReq_t)){
        return g_ui32RxCount;
    }
#ifndef sendStatus_p
//    xTaskNotifyFromISR(orderlyHandling,SignalUSBbufferReady,eSetBits,NULL);
    message = SignalUSBbufferReady;
    xQueueSendFromISR(orderlyQueue,&message,NULL);
#endif
    pvMsgData_tmp = pui8Data;
#ifndef sendStatus_p
    packet_counter = 0;
    g_ui32RxCount = 0;
    //
    // Get the current buffer information to allow us to write directly to
    // the transmit buffer (we already have enough information from the
    // parameters to access the receive buffer directly).
    //
    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);

    //
    // How much space is there in the transmit buffer?
    //
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    ui32WriteIndex = sTxRing.ui32WriteIndex;
#ifdef STATUS_T
    ui32Count = sizeof(struct Status_t) ;
//    ui32Count = 44;

    if(ui32Space>= ui32Count)
    {
        tx_status = (uint8_t*)getStatus();

        memcpy(&g_pui8USBTxBuffer[ui32WriteIndex], tx_status, ui32Count);
    }
    else
#endif

#ifdef CHAR_SEND__

    //
    // How many characters can we process this time round?
    //
    ui32Loop = (ui32Space < ui32NumBytes) ? ui32Space : ui32NumBytes;
    ui32Count = ui32Loop;

    //
    // Update our receive counter.
    //
    g_ui32RxCount += ui32NumBytes;

    //
    // Dump a debug message.
    //
    //    DEBUG_PRINT("Received %d bytes\n", ui32NumBytes);

    //
    // Set up to process the characters by directly accessing the USB buffers.
    //
    ui32ReadIndex = (uint32_t)(pui8Data - g_pui8USBRxBuffer);

    while(ui32Loop)
    {
        //
        // Copy from the receive buffer to the transmit buffer converting
        // character case on the way.
        //

        //
        // Is this a lower case character?
        //
        if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'a') &&
                (g_pui8USBRxBuffer[ui32ReadIndex] <= 'z'))
        {
            //
            // Convert to upper case and write to the transmit buffer.
            //
            g_pui8USBTxBuffer[ui32WriteIndex] =
                    (g_pui8USBRxBuffer[ui32ReadIndex] - 'a') + 'A';
        }
        else
        {
            //
            // Is this an upper case character?
            //
            if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'A') &&
                    (g_pui8USBRxBuffer[ui32ReadIndex] <= 'Z'))
            {
                //
                // Convert to lower case and write to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                        (g_pui8USBRxBuffer[ui32ReadIndex] - 'Z') + 'z';
            }
            else
            {
                //
                // Copy the received character to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                        g_pui8USBRxBuffer[ui32ReadIndex];
            }
        }

        //
        // Move to the next character taking care to adjust the pointer for
        // the buffer wrap if necessary.
        //
        ui32WriteIndex++;
        ui32WriteIndex = (ui32WriteIndex == BULK_BUFFER_SIZE) ?
                0 : ui32WriteIndex;

        ui32ReadIndex++;
        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ?
                0 : ui32ReadIndex;

        ui32Loop--;
    }
#else
    uint8_t tmpb[5] = {0, 0, 0, 232};
    uint32_t tmpr,i;
    union{
        uint8_t* bstatus;
        uint32_t* mstatus;  //[8]
        struct Status_t* cstatus;
    }stunion;

    stunion.cstatus = getStatus();

    uint16_t x = 1; /* 0x0001 */
#define big_endian  2
#define  little_endian  4
#define Revers  __asm (" REV R0,R0\n")

    uint8_t endian = *((unsigned char *) &x) == 0 ? big_endian : little_endian;
/*
    if(endian == little_endian){
        for(i=0;i<8;i++){
            tmpr = stunion.mstatus[0];
//            Revers;
__asm(
       " REV  R0, R0\n str r0, [sp, #0x30] \n"
       );
            stunion.mstatus[0] = tmpr;
        }
//        tmpr = stunion.cstatus->modelState;
//         Revers;

//        tmpb[0] = 232;
//        tmpb[3] = 0;
    }
*/

//    uint8_t i = 0;
    i = 0;

    //
    // How many characters can we process this time round?
    //
//    ui32Loop = (ui32Space < ui32NumBytes) ? ui32Space : ui32NumBytes;
//    ui32Count = ui32Loop;
    ui32Loop = sizeof(struct Status_t);
    ui32Count = ui32Loop;

    //
    // Update our receive counter.
    //
//    g_ui32RxCount += ui32NumBytes;

    //
    // Dump a debug message.
    //
    //    DEBUG_PRINT("Received %d bytes\n", ui32NumBytes);

    //
    // Set up to process the characters by directly accessing the USB buffers.
    //
    ui32ReadIndex = (uint32_t)(pui8Data - g_pui8USBRxBuffer);

    while(ui32Loop)
    {

        g_pui8USBTxBuffer[ui32WriteIndex] = stunion.bstatus[i];
        i++;

        ui32WriteIndex++;
        ui32WriteIndex = (ui32WriteIndex == BULK_BUFFER_SIZE) ?
                0 : ui32WriteIndex;

//        ui32ReadIndex++;
//        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ?
//                0 : ui32ReadIndex;

        ui32Loop--;
    }

#endif

    //
    // We've processed the data in place so now send the processed data
    // back to the host.
    //
    USBBufferDataWritten(&g_sTxBuffer, ui32Count);

//    DEBUG_PRINT("Wrote %d bytes\n", ui32Count);
#endif
    //
    // We processed as much data as we can directly from the receive buffer so
    // we need to return the number of bytes to allow the lower layer to
    // update its read pointer appropriately.
    //
    return(ui32Count);
}

void parcerSegment(uint8_t* pmsg ,uint32_t size)
{
    struct sSegment* segment = (struct sSegment*)pmsg;
    //Size of sSegment :152 (head:8 and axis:36)
    uint32_t rowcount = sizeof(segment)/sizeof(uint32_t);
    uint32_t row,i, ui32Loop;
    uint32_t ui32ReadIndex;
    uint32_t* prow = (uint32_t*)pmsg;

    ui32Loop = size;

    ui32ReadIndex = (uint32_t)(pmsg - g_pui8USBRxBuffer);

    while(ui32Loop){

/*
//    for(i=0;i<rowcount;i++){
        row = prow[ui32ReadIndex];

        __asm(
               " REV  R0, R0\n str r0, [sp, #0x10] \n"
               );
        prow[ui32ReadIndex] = row;
//    }

*/


        //
        // Move to the next character taking care to adjust the pointer for
        // the buffer wrap if necessary.
        //
//        ui32WriteIndex++;
//        ui32WriteIndex = (ui32WriteIndex == BULK_BUFFER_SIZE) ?
//                0 : ui32WriteIndex;

        ui32ReadIndex++;
        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ?
                0 : ui32ReadIndex;

        ui32Loop--;

    }

    NoOperation;
}


//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
//            UARTprintf("Host connected.\n");

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
//            UARTprintf("Host disconnected.\n");
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            psDevice = (tUSBDBulkDevice *)pvCBData;
            parcerSegment(pvMsgData, ui32MsgValue);
            //
            // Read the new packet and echo it back to the host.
            //
            return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        //
        // Ignore all other events and return 0.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
//    DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return(0);
}




uint8_t usb_init(void){


    //
    // Initialize the transmit and receive buffers.
    //
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDBulkInit(0, &g_sBulkDevice);


    //
    // Clear our local byte counters.
    //
//    ui32RxCount = 0;
//    ui32TxCount = 0;
    return 0;

}



