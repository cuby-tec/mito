/*
 * vTaskGetRunTimeStats.c
 *
 *  Created on: 14 сент. 2017 г.
 *      Author: walery
 */

//--------------

#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "vTaskGetRunTimeStats.h"

//------------- defs

//-------------- vars



char statsBuffer[TASK_NUMBER_STATE *TASK_BUFFER_SIZE];

//-------------- function

/*

void vTaskGetRunTimeStats( char *pcWriteBuffer )
{
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    unsigned long ulTotalRunTime, ulStatsAsPercentage;

//        Make sure the write buffer does not contain a string.
       *pcWriteBuffer = 0x00;

//        Take a snapshot of the number of tasks in case it changes while this
//       function is executing.
       uxArraySize = uxCurrentNumberOfTasks();

//        Allocate a TaskStatus_t structure for each task.  An array could be
//       allocated statically at compile time.
       pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

       if( pxTaskStatusArray != NULL )
       {
//           Generate raw status information about each task.
          uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                     uxArraySize,
                                     &ulTotalRunTime );

//           For percentage calculations.
          ulTotalRunTime /= 100UL;

//           Avoid divide by zero errors.
          if( ulTotalRunTime > 0 )
          {
//              For each populated position in the pxTaskStatusArray array,
//             format the raw data as human readable ASCII data.
             for( x = 0; x < uxArraySize; x++ )
             {
//                 What percentage of the total run time has the task used?
//                This will always be rounded down to the nearest integer.
//                ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage =
                      pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

                if( ulStatsAsPercentage > 0UL )
                {
                   sprintf( pcWriteBuffer, "%s\t\t%lu\t\t%lu%%\r\n",
                                     pxTaskStatusArray[ x ].pcTaskName,
                                     pxTaskStatusArray[ x ].ulRunTimeCounter,
                                     ulStatsAsPercentage );
                }
                else
                {
//                    If the percentage is zero here then the task has
//                   consumed less than 1% of the total run time.
                   sprintf( pcWriteBuffer, "%s\t\t%lu\t\t<1%%\r\n",
                                     pxTaskStatusArray[ x ].pcTaskName,
                                     pxTaskStatusArray[ x ].ulRunTimeCounter );
                }

                pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
             }
          }

//           The array is no longer needed, free the memory it consumes.
          vPortFree( pxTaskStatusArray );
       }
}

*/


