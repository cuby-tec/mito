/*
 * SPI_Microstepper.c
 *
 *  Created on: 8 дек. 2017 г.
 *      Author: walery
 */

//--------------

#include "SPI_Microstepper.h"
#include "msmotor/msport.h"
#include "inc/typedefs.h"
#include <stdbool.h>
//#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"


//------------- defs
#define ui32BitRate     1000000

// Value ui32DataWidth as from 4 to 16. The SSIDR register is 16-bits wide.
#define ui32DataWidth   (8)
//-------------- vars


//-------------- function
void SPI_init(void){
    uint32_t pui32Data[4];
    SSIConfigSetExpClk(EXT_PORT, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, ui32BitRate, ui32DataWidth);
    //
    // Enable the SSI0 module.
    //
    SSIEnable(EXT_PORT);

    while(SSIDataGetNonBlocking(EXT_PORT, &pui32Data[0])){
        NoOperation;
    }
}

void SPI_Send(uint8_t* pui32DataTx,uint8_t count){
    uint8_t ui32Index;
    for(ui32Index = 0; ui32Index < count; ui32Index++){
        //
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        //
        SSIDataPut(EXT_PORT, pui32DataTx[ui32Index]);
    }

    //
    // Wait until SSI2 is done transferring all the data in the transmit FIFO.
    //
    while(SSIBusy(EXT_PORT))
    {
    }
    GPIOPinWrite(EXT_BASE, EXT_RCLK, EXT_RCLK);
    NoOperation;
    GPIOPinWrite(EXT_BASE, EXT_RCLK, 0);

}

void SPI_Read(uint32_t* data,uint8_t count){
//    uint8_t ui32Index;
    uint32_t pui32DataRx;
    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx))
    {
    }

    GPIOPinWrite(EXT_BASE, EXT_SHLD, 0);
    NoOperation;
    GPIOPinWrite(EXT_BASE, EXT_SHLD, EXT_SHLD);
    //
    // Receive the data using the "blocking" Get function. This function
    // will wait until there is data in the receive FIFO before returning.
    //
//    SSIDataGet(SSI2_BASE, &pui32DataRx);
    SSIDataPut(SSI2_BASE, 0);
    SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx);
    *data = pui32DataRx;
}
