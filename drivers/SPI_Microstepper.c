/*
 * SPI_Microstepper.c
 *
 *  Created on: 8 дек. 2017 г.
 *      Author: walery
 */

//--------------


#include <stdbool.h>
//#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"

#include "SPI_Microstepper.h"

//------------- defs
#define ui32BitRate     1000000

// Value ui32DataWidth as from 4 to 16. The SSIDR register is 16-bits wide.
#define ui32DataWidth   (8)
//-------------- vars


//-------------- function
void SPI_init(void){
    uint32_t pui32Data;
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, ui32BitRate, ui32DataWidth);
    SSIDataGetNonBlocking(SSI2_BASE, &pui32Data);
    if(pui32Data){
        while(pui32Data){
            SSIDataGetNonBlocking(SSI2_BASE, &pui32Data);
        }
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
        SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]);
    }

    //
    // Wait until SSI2 is done transferring all the data in the transmit FIFO.
    //
    while(SSIBusy(SSI2_BASE))
    {
    }

}

void SPI_Read(uint8_t* data,uint8_t count){
    uint8_t ui32Index;
    uint32_t pui32DataRx[4];
    for(ui32Index = 0; ui32Index < count; ui32Index++){
        //
        // Receive the data using the "blocking" Get function. This function
        // will wait until there is data in the receive FIFO before returning.
        //
        SSIDataGet(SSI2_BASE, &pui32DataRx[ui32Index]);
    }
}
