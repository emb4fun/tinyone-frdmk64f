/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2018-2023 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
***************************************************************************
*  History:
*
*  16.03.2018  mifi  First Version, tested with the FRDM-K64F board.
**************************************************************************/
#define __TALCPU_COM_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include "tal.h"

#include "fsl_uart.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static TAL_COM_DCB *DCBArray[TAL_COM_PORT_MAX];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  IRQHandler                                                           */
/*                                                                       */
/*  This is the generic IRQ handler.                                     */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void IRQHandler (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error;
   TAL_COM_HW  *pHW    = &pDCB->HW;
   UART_Type   *pUARTx = (UART_Type*)pHW->dBaseAddress;
   uint8_t      bData;

   /*
    * RX Interrupt
    */
   if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(pUARTx))
   {
      bData = pUARTx->D;

      /* If we have no overflow... */
      if (TAL_FALSE == pDCB->bRxOverflow)
      {
         /* ... put it into the ring buffer */
         Error = tal_MISCRingAdd(&pDCB->RxRing, &bData);
         if (TAL_OK == Error)
         {
            /* Signal counting semaphore */
            OS_SemaSignalFromInt(&pDCB->RxRdySema);
         }
         else
         {
            /* Ups, overflow */
            pDCB->bRxOverflow = TAL_OK;
         }
      }
   } /* end RX interrupt */


   /*
    * Check for TX interrupt, but only if enabled
    */
   if ((UART_GetEnabledInterrupts(pUARTx) & kUART_TxDataRegEmptyInterruptEnable) && /* <= enabled ? */
       (kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(pUARTx)))                    /* <= TX interrupt ? */
   {
      /* Read Data from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, &bData);
      if (Error != TAL_OK)
      {
         /* Ups, no data available, disable interrupt */
         UART_DisableInterrupts(pUARTx, kUART_TxDataRegEmptyInterruptEnable);
      }
      else
      {
         /* Send data */
         pUARTx->D = bData;
      }
   } /* end "TX interrupt */

} /* IRQHandler */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  UARTx_IRQHandler                                                     */
/*                                                                       */
/*  This is the Cortex USARTx IRQ handler.                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void UART0_RX_TX_IRQHandler (void)
{
   TAL_CPU_IRQ_ENTER();
   IRQHandler(DCBArray[TAL_COM_PORT_1]);

   /*
    * Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store
    * immediate overlapping exception return operation might vector to
    * incorrect interrupt
    */
#if defined(__CORTEX_M) && (__CORTEX_M == 4U)
    __DSB();
#endif

   TAL_CPU_IRQ_EXIT();
} /* UART0_RX_TX_IRQHandler */

/*************************************************************************/
/*  cpu_COMInit                                                          */
/*                                                                       */
/*  Prepare the hardware for use by the Open function later. Set the HW  */
/*  information depending of ePort and "enable" the COM port.            */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMInit (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_ERR_COM_PORT_RANGE;
   TAL_COM_HW  *pHW    = &pDCB->HW;

   switch (pDCB->ePort)
   {
      case TAL_COM_PORT_1:
      {
         Error = tal_BoardEnableCOM1();
         if (TAL_OK == Error)
         {
            DCBArray[TAL_COM_PORT_1] = pDCB;

            pHW->dBaseAddress = UART0_BASE;
            pHW->nIrqNumber   = UART0_RX_TX_IRQn;
            pHW->nIrqPriority = UART0_PRIO;

            /* Disable UART */
            UART0->C2 = 0;

            /* Set irq and priority */
            tal_CPUIrqSetPriority(pHW->nIrqNumber, pHW->nIrqPriority);
         }
         break;
      } /* TAL_COM_PORT_1 */

      default:
      {
         /* Do nothing */
         break;
      }
   } /* end switch (pDCB->ePort) */

   return(Error);
} /* cpu_COMInit */

/*************************************************************************/
/*  cpu_COMIoctl                                                         */
/*                                                                       */
/*  Call a IOCTL function.                                               */
/*                                                                       */
/*  In    : pDCB, wNum, pParam                                           */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMIoctl (TAL_COM_DCB *pDCB, TAL_COM_IOCTL eFunc, uint32_t *pParam)
{
   (void)pDCB;
   (void)eFunc;
   (void)pParam;

   return(TAL_ERROR);
} /* cpu_COMIoctl */

/*************************************************************************/
/*  cpu_COMOpen                                                          */
/*                                                                       */
/*  Open the COM port.                                                   */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMOpen (TAL_COM_DCB *pDCB)
{
   TAL_RESULT     Error = TAL_ERROR;
   TAL_COM_HW   *pHW    = &pDCB->HW;
   UART_Type    *pUARTx = (UART_Type*)pHW->dBaseAddress;
   uint32_t      dUartClock;
   uart_config_t  config;
   status_t       status;

   UART_GetDefaultConfig(&config);

   /*
    * Check parameter first
    */

   /* Check word length */
   switch (pDCB->Settings.eLength)
   {
      case TAL_COM_LENGTH_8:
      {
         /* Do nothing */
         break;
      }

      default:
      {
         Error = TAL_ERR_COM_LENGTH;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eLength) */

   /* Check parity settings */
   switch (pDCB->Settings.eParity)
   {
      case TAL_COM_PARITY_NONE:
      {
         config.parityMode = kUART_ParityDisabled;
         break;
      }

      case TAL_COM_PARITY_EVEN:
      {
         config.parityMode = kUART_ParityEven;
         break;
      }

      case TAL_COM_PARITY_ODD:
      {
         config.parityMode = kUART_ParityOdd;
         break;
      }

      default:
      {
         Error = TAL_ERR_COM_PARITY;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eParity) */

   /* Check stop bit settings */
   switch (pDCB->Settings.eStop)
   {
      case TAL_COM_STOP_1_0:
      {
         config.stopBitCount = kUART_OneStopBit;
         break;
      }

      default:
      {
         Error = TAL_ERR_COM_STOP;
         goto COMOpenEnd;  /*lint !e801*/
         break;   /*lint !e527*/
      }
   } /* switch (pDCB->Settings.eStop) */

   /* Check baud rate */
   if (pDCB->Settings.dBaudrate != 0)
   {
      config.baudRate_Bps = pDCB->Settings.dBaudrate;
   }
   else
   {
      Error = TAL_ERR_COM_BAUDRATE;
      goto COMOpenEnd;  /*lint !e801*/
   }

   /*
    * Initializes an UART instance with the user configuration structure and the peripheral clock.
    */
   config.txFifoWatermark = 0;

   dUartClock = CLOCK_GetFreq(UART0_CLK_SRC);

   status = UART_Init(pUARTx, &config, dUartClock);
   if (kStatus_Success == status)
   {
      /* Enable RX interrupt. */
      UART_EnableInterrupts(pUARTx, (kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable));

      /* Enable the interrupt */
      tal_CPUIrqEnable(pHW->nIrqNumber);

      /* Enable RX/TX */
      pUARTx->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);

      //LPUART_EnableTx(pUARTx, (bool)1);
      //LPUART_EnableRx(pUARTx, (bool)1);

      Error = TAL_OK;
   }

COMOpenEnd:
   return(Error);
} /* cpu_COMOpen */

/*************************************************************************/
/*  cpu_COMClose                                                         */
/*                                                                       */
/*  Close the COM port.                                                  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMClose (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_OK;
   TAL_COM_HW  *pHW    = &pDCB->HW;
   UART_Type   *pUARTx = (UART_Type*)pHW->dBaseAddress;

   /* Disable UART */
   pUARTx->C2 = 0;

   /* Disable the interrupt in the GIC. */
   tal_CPUIrqDisable(pHW->nIrqNumber);

   return(Error);
} /* cpu_COMClose */

/*************************************************************************/
/*  cpu_COMStartTx                                                       */
/*                                                                       */
/*  Send the data from the ring buffer if the TX interrupt is disabled.  */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / error cause                                         */
/*************************************************************************/
TAL_RESULT cpu_COMStartTx (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_OK;
   TAL_COM_HW  *pHW    = &pDCB->HW;
   UART_Type   *pUARTx = (UART_Type*)pHW->dBaseAddress;
   uint8_t      bData;

   TAL_CPU_DISABLE_ALL_INTS();
   if (UART_GetEnabledInterrupts(pUARTx) & kUART_TxDataRegEmptyInterruptEnable)
   {
      /* TX interrupt is enabled, do nothing */
   }
   else
   {
      /* Get data from the ring buffer */
      Error = tal_MISCRingGet(&pDCB->TxRing, &bData);
      if (TAL_OK == Error)
      {
         /* Send data */
         pUARTx->D = bData;

         /* Enable TX interrupt */
         UART_EnableInterrupts(pUARTx, kUART_TxDataRegEmptyInterruptEnable);
      }
   }
   TAL_CPU_ENABLE_ALL_INTS();

   return(Error);
} /* cpu_COMStartTx */

/*************************************************************************/
/*  cpu_COMTxIsRunning                                                   */
/*                                                                       */
/*  Check if TX is still running.                                        */
/*                                                                       */
/*  In    : pDCB                                                         */
/*  Out   : none                                                         */
/*  Return: TAL_OK / TAL_ERROR                                           */
/*************************************************************************/
TAL_RESULT cpu_COMTxIsRunning (TAL_COM_DCB *pDCB)
{
   TAL_RESULT    Error = TAL_OK;
   TAL_COM_HW  *pHW    = &pDCB->HW;
   UART_Type   *pUARTx = (UART_Type*)pHW->dBaseAddress;

   TAL_CPU_DISABLE_ALL_INTS();
   if (UART_GetEnabledInterrupts(pUARTx) & kUART_TxDataRegEmptyInterruptEnable)
   {
      /* TX is still running */
      Error = TAL_OK;
   }
   else
   {
      /* TX is not running */
      Error = TAL_ERROR;
   }
   TAL_CPU_ENABLE_ALL_INTS();

   return(Error);
} /* cpu_COMTxIsRunning */

/*** EOF ***/
