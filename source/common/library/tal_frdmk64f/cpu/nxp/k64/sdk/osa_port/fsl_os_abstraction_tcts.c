/**************************************************************************
*  Copyright (c) 2021 by Michael Fischer (www.emb4fun.de).
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
*  06.02.2021  mifi  First Version.
**************************************************************************/
#define __FSL_OS_ABSTRACTION_TCTS_C__

/*=======================================================================*/
/*  Include                                                              */
/*=======================================================================*/
#include <stdint.h>
#include <string.h>
#include "tcts.h"
#include "tal.h"

#include "fsl_os_abstraction.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*
 * Disable and enable interrupt macros.
 */
#if 0 
__attribute__( ( always_inline ) ) static inline uint32_t _DisableAllInts (void)
{
   uint32_t dMask = 0;

   __asm__ volatile ("MRS %0, primask" : "=r" (dMask) );
   __asm__ volatile ("cpsid i" : : : "memory");
  
   return(dMask);
} /* _DisableAllInts */
  
__attribute__( ( always_inline ) ) static inline void _EnableAllInts (uint32_t dMask)
{
   __set_PRIMASK(dMask);
} /* _EnableAllInts */
#endif


/*
 * Message queue
 */
typedef struct _msgq_
{
   OS_SEMA      UsedCntSema;
   OS_SEMA      FreeCntSema;
   uint16_t    wCountMax;
   uint16_t    wCount;
   uint16_t    wSize;
   uint16_t    wInIndex;
   uint16_t    wOutIndex;
   uint8_t    *pBuffer;
} msgq_t;

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static OS_SEMA  OSASema;
static OS_MUTEX OSAMutex;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */

/*=======================================================================*/

/*************************************************************************/
/*  OSA_EnterCritical                                                    */
/*                                                                       */
/*  Enter critical mode.                                                 */
/*                                                                       */
/*  In    : sr                                                           */
/*  Out   : sr                                                           */
/*  Return: none                                                         */
/*************************************************************************/
void OSA_EnterCritical (uint32_t *sr)
{
   *sr = _DisableAllInts();
} /* OSA_EnterCritical */

/*************************************************************************/
/*  OSA_ExitCritical                                                     */
/*                                                                       */
/*  Exit critical mode and retore the previous mode.                     */
/*                                                                       */
/*  In    : sr                                                           */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OSA_ExitCritical (uint32_t sr)
{
   _EnableAllInts(sr);
} /* OSA_ExitCritical */

/*************************************************************************/
/*  OSA_TimeDelay                                                        */
/*                                                                       */
/*  This function is used to suspend the active thread for the given     */
/*  number of milliseconds.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void OSA_TimeDelay (uint32_t millisec)
{
   OS_TimeDly(millisec);

} /* OSA_TimeDelay */

/*************************************************************************/
/*  OSA_SemaphoreCreate                                                  */
/*                                                                       */
/*  Ccreates a semaphore and sets the value to the parameter initValue.  */ 
/*                                                                       */
/*  In    : semaphoreHandle, initValue                                   */
/*  Out   : semaphoreHandle                                              */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_SemaphoreCreate (osa_semaphore_handle_t semaphoreHandle, uint32_t initValue)
{
   static uint8_t InitDone = 0;
   osa_status_t   Status = KOSA_StatusError;

   if ((0 == InitDone) && (semaphoreHandle != NULL))
   {
      InitDone = 1;
      Status   = KOSA_StatusSuccess;

      OS_SemaCreate(&OSASema, (int32_t)initValue, 0xFF);

      *(uint32_t *)semaphoreHandle = (uint32_t)&OSASema;
   }

   return(Status);
} /* OSA_SemaphoreCreate */

/*************************************************************************/
/*  OSA_SemaphoreWait                                                    */
/*                                                                       */
/*  Checks the semaphore's counting value. If it is positive, decreases  */
/*  it and returns KOSA_StatusSuccess. Otherwise, a timeout is used to   */
/*  wait.                                                                */
/*                                                                       */
/*  In    : semaphoreHandle, millisec                                    */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError / KOSA_StatusTimeout   */
/*************************************************************************/
osa_status_t OSA_SemaphoreWait (osa_semaphore_handle_t semaphoreHandle, uint32_t millisec)
{
   osa_status_t Status = KOSA_StatusError;
   int          rc;
   uint32_t     TimeoutMs;
   OS_SEMA    *pSema;

   if ((semaphoreHandle != NULL) && (*(uint32_t*)semaphoreHandle != 0))
   {
      TimeoutMs = (osaWaitForever_c == millisec) ? OS_WAIT_INFINITE : millisec;

      pSema = (OS_SEMA*)(*(uint32_t*)semaphoreHandle);
      rc = OS_SemaWait(pSema, TimeoutMs);
      if (OS_RC_OK == rc)
      {
         Status = KOSA_StatusSuccess;  /* Semaphore taken */
      }
      else
      {
         Status = KOSA_StatusTimeout;  /* Timeout */
      }
   }

   return(Status);
} /* OSA_SemaphoreCreate */

/*************************************************************************/
/*  OSA_SemaphorePost                                                    */
/*                                                                       */
/*  Signals for someone waiting on the semaphore to wake up.             */
/*                                                                       */
/*  In    : semaphoreHandle                                              */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_SemaphorePost (osa_semaphore_handle_t semaphoreHandle)
{
   osa_status_t Status = KOSA_StatusError;
   OS_SEMA    *pSema;

   if ((semaphoreHandle != NULL) && (*(uint32_t*)semaphoreHandle != 0))
   {
      pSema  = (OS_SEMA*)(*(uint32_t*)semaphoreHandle);
      Status = KOSA_StatusSuccess;

      if (0U != __get_IPSR())
      {
         OS_SemaSignalFromInt(pSema);
      }
      else
      {
         OS_SemaSignal(pSema);
      }
   }

   return(Status);
} /* OSA_SemaphorePost */

/*************************************************************************/
/*  OSA_MutexCreate                                                      */
/*                                                                       */
/*  This function is used to create a mutex.                             */
/*                                                                       */
/*  In    : mutexHandle                                                  */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MutexCreate (osa_mutex_handle_t mutexHandle)
{
   static uint8_t InitDone = 0;
   osa_status_t   Status = KOSA_StatusError;
   
   if (mutexHandle != NULL)
   {
      Status = KOSA_StatusSuccess;

      if (0 == InitDone)
      {
         InitDone = 1;
         Status   = KOSA_StatusSuccess;

         OS_MutexCreate(&OSAMutex);
      }
      
      *(uint32_t *)mutexHandle = (uint32_t)&OSAMutex;
   }      

   return(Status);
} /* OSA_MutexCreate */

/*************************************************************************/
/*  OSA_MutexLock                                                        */
/*                                                                       */
/*  This function checks the mutex's status, if it is unlocked, lock it  */
/*  and returns KOSA_StatusSuccess, otherwise, wait for the mutex.       */
/*  This function returns KOSA_StatusSuccess if the mutex is obtained,   */
/*  returns KOSA_StatusError if any errors occur during waiting. If the  */
/*  mutex has been locked, pass 0 as timeout will return                 */
/*  KOSA_StatusTimeout immediately.                                      */
/*                                                                       */
/*  In    : mutexHandle, millisec                                        */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError / KOSA_StatusTimeout   */
/*************************************************************************/
osa_status_t OSA_MutexLock (osa_mutex_handle_t mutexHandle, uint32_t millisec)
{
   osa_status_t Status = KOSA_StatusError;
   int          rc;
   uint32_t     TimeoutMs;
   OS_MUTEX   *pMutex;

   if ((mutexHandle != NULL) && (*(uint32_t*)mutexHandle != 0))
   {
      TimeoutMs = (osaWaitForever_c == millisec) ? OS_WAIT_INFINITE : millisec;

      pMutex = (OS_MUTEX*)(*(uint32_t*)mutexHandle);
      rc = OS_MutexWait(pMutex, TimeoutMs);
      if (OS_RC_OK == rc)
      {
         Status = KOSA_StatusSuccess;  /* Semaphore taken */
      }
      else
      {
         Status = KOSA_StatusTimeout;  /* Timeout */
      }
   
   }
   
   return(Status);
} /* OSA_MutexLock */

/*************************************************************************/
/*  OSA_MutexUnlock                                                      */
/*                                                                       */
/*  This function is used to unlock a mutex.                             */
/*                                                                       */
/*  In    : mutexHandle                                                  */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MutexUnlock (osa_mutex_handle_t mutexHandle)
{
   osa_status_t Status = KOSA_StatusError;
   OS_MUTEX   *pMutex;

   if ((mutexHandle != NULL) && (*(uint32_t*)mutexHandle != 0))
   {
      pMutex  = (OS_MUTEX*)(*(uint32_t*)mutexHandle);
      Status = KOSA_StatusSuccess;

      if (0U != __get_IPSR())
      {
         while(1)
         {
          /* Not supported */
          __asm__ volatile ("nop");
         }
      }
      else
      {
         OS_MutexSignal(pMutex);
      }
   }

   return(Status);

} /* OSA_MutexUnlock */

/*************************************************************************/
/*  OSA_MutexDestroy                                                     */
/*                                                                       */
/*  This function is used to destroy a mutex.                            */
/*                                                                       */
/*  In    : mutexHandle                                                  */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MutexDestroy (osa_mutex_handle_t mutexHandle)
{
   (void)mutexHandle;
   
   return(KOSA_StatusSuccess);
} /* OSA_MutexDestroy */

/*************************************************************************/
/*  OSA_MsgQCreate                                                       */
/*                                                                       */
/*  This function  allocates memory for and initializes a message queue. */
/*                                                                       */
/*  In    : msgqHandle, msgNo, msgSize                                   */
/*  Out   : msgqHandle                                                   */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MsgQCreate (osa_msgq_handle_t msgqHandle, uint32_t msgNo, uint32_t msgSize)
{
   osa_status_t   Status = KOSA_StatusError;
   msgq_t       *pMSGQ;
   uint8_t      *pBuffer;
   
   uint32_t       Value;
   
   if (msgqHandle != NULL)
   {
      pMSGQ   = xmalloc(XM_ID_HEAP, sizeof(msgq_t));
      pBuffer = xmalloc(XM_ID_HEAP, (msgNo * msgSize));
      
      if ((pMSGQ != NULL) && (pBuffer != NULL))
      {
         Status = KOSA_StatusSuccess;

         memset(pMSGQ, 0x00, sizeof(msgq_t));

         OS_SemaCreate(&pMSGQ->UsedCntSema, 0,     msgNo);
         OS_SemaCreate(&pMSGQ->FreeCntSema, msgNo, msgNo);

         pMSGQ->wCountMax = msgNo;
         pMSGQ->wCount    = 0;
         pMSGQ->wSize     = msgSize;
         pMSGQ->wInIndex  = 0;
         pMSGQ->wOutIndex = 0;
         pMSGQ->pBuffer   = pBuffer;

         *(uint32_t *)msgqHandle = (uint32_t)pMSGQ;
      }
      else
      {
         xfree(pMSGQ);
         xfree(pBuffer);
      }
   }   
   
   return(Status);
} /* OSA_MsgQCreate */

/*************************************************************************/
/*  OSA_MsgQPut                                                          */
/*                                                                       */
/*  Puts a message at the end of the queue.                              */
/*                                                                       */
/*  In    : msgqHandle, pMessage                                         */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MsgQPut (osa_msgq_handle_t msgqHandle, osa_msg_handle_t pMessage)
{
   osa_status_t   Status = KOSA_StatusError;
   msgq_t       *pMSGQ;
   uint16_t      wFreeCnt;
   uint16_t      wOffset;

   if ((msgqHandle != NULL) && (*(uint32_t *)msgqHandle != 0))
   {
      pMSGQ = (msgq_t *)(*(uint32_t *)msgqHandle);

      TAL_CPU_DISABLE_ALL_INTS();
      
      wFreeCnt = pMSGQ->wCountMax - pMSGQ->wCount;
      if (wFreeCnt != 0)
      {
         /*
          * OSSemaWait cannot be used inside an interrupt.
          * Therefore decrement the semaphore counter direct.
          */
         pMSGQ->FreeCntSema.nCounter--;

         /* Increment counter, one more message available */
         pMSGQ->wCount++;

         /* Store message */
         wOffset = pMSGQ->wInIndex * pMSGQ->wSize;
         memcpy(&pMSGQ->pBuffer[wOffset], (uint8_t*)pMessage, pMSGQ->wSize); 
         
         /* Switch to next place */
         pMSGQ->wInIndex++;

         /* Check end of ring buffer */
         if (pMSGQ->wCountMax == pMSGQ->wInIndex)
         {
            pMSGQ->wInIndex = 0;
         }

         /* Signal message available */
         OS_SemaSignalFromInt(&pMSGQ->UsedCntSema);

         Status = KOSA_StatusSuccess;
      }
      
      TAL_CPU_ENABLE_ALL_INTS();
   }   
   
   return(Status);
} /* OSA_MsgQPut */

/*************************************************************************/
/*  OSA_MsgQGet                                                          */
/*                                                                       */
/*  Reads and remove a message at the head of the queue.                 */
/*                                                                       */
/*  In    : msgqHandle, pMessage, millisec                               */
/*  Out   : pMessage                                                     */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MsgQGet (osa_msgq_handle_t msgqHandle, osa_msg_handle_t pMessage, uint32_t millisec)
{
   osa_status_t   Status = KOSA_StatusError;
   msgq_t       *pMSGQ;
   uint16_t      wCount;
   uint16_t      wOffset;
   int            rc;

   if ((msgqHandle != NULL) && (*(uint32_t *)msgqHandle != 0))
   {
      pMSGQ = (msgq_t *)(*(uint32_t *)msgqHandle);

      if (0 == millisec)
      {
         TAL_CPU_DISABLE_ALL_INTS();
         wCount = pMSGQ->wCount;
         TAL_CPU_ENABLE_ALL_INTS();
      
         if (0 == wCount)
         {
            return(KOSA_StatusError);
         }
      }
      else
      {
         if (osaWaitForever_c == millisec)
         {
            millisec = OS_WAIT_INFINITE;
         }

         rc = OS_SemaWait(&pMSGQ->UsedCntSema, millisec);
         if (OS_RC_OK == rc)
         {
            TAL_CPU_DISABLE_ALL_INTS();

            /* Decrement counter, one message will be removed */
            pMSGQ->wCount--;

            /* Get message */
            wOffset = pMSGQ->wOutIndex * pMSGQ->wSize;
            memcpy((uint8_t*)pMessage, &pMSGQ->pBuffer[wOffset], pMSGQ->wSize); 

            /* Switch to next place */
            pMSGQ->wOutIndex++;

            /* Check end of ring buffer */
            if (pMSGQ->wCountMax == pMSGQ->wOutIndex)
            {
               pMSGQ->wOutIndex = 0;
            }

            OS_SemaSignalFromInt(&pMSGQ->FreeCntSema);

            TAL_CPU_ENABLE_ALL_INTS();
            
            Status = KOSA_StatusSuccess;
         }
      }
   }   
   
   return(Status);
} /* OSA_MsgQGet */

/*************************************************************************/
/*  OSA_MsgQAvailableMsgs                                                */
/*                                                                       */
/*  Get the available message.                                           */
/*                                                                       */
/*  In    : msgqHandle                                                   */
/*  Out   : none                                                         */
/*  Return: Available message count                                      */
/*************************************************************************/
int OSA_MsgQAvailableMsgs (osa_msgq_handle_t msgqHandle)
{
   if (msgqHandle != NULL)
   {
   }   

   return(0);
} /* OSA_MsgQAvailableMsgs */

/*************************************************************************/
/*  OSA_MsgQDestroy                                                      */
/*                                                                       */
/*  Destroys a previously created queue.                                 */
/*                                                                       */
/*  In    : msgqHandle                                                   */
/*  Out   : none                                                         */
/*  Return: KOSA_StatusSuccess / KOSA_StatusError                        */
/*************************************************************************/
osa_status_t OSA_MsgQDestroy (osa_msgq_handle_t msgqHandle)
{
   osa_status_t   Status = KOSA_StatusError;
   msgq_t       *pMSGQ;

   if ((msgqHandle != NULL) && (*(uint32_t *)msgqHandle != 0))
   {
      Status = KOSA_StatusSuccess;
      
      pMSGQ = (msgq_t *)(*(uint32_t *)msgqHandle);
      
      OS_SemaDelete(&pMSGQ->UsedCntSema);
      OS_SemaDelete(&pMSGQ->FreeCntSema);
      
      xfree(pMSGQ->pBuffer);
      xfree(pMSGQ);
   }   
   
   return(Status);
} /* OSA_MsgQDestroy */

/*** EOF ***/
