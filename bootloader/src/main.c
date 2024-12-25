/**************************************************************************
*  Copyright (c) 2023-2024 by Michael Fischer (www.emb4fun.de).
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
**************************************************************************/
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdint.h>
#include <stdio.h>
#include "tal.h"
#include "terminal.h"
#include "ff.h"
#include "diskio.h"
#include "adler32.h"
#include "xbin.h"
#include "xmempool.h"
#include "flash.h"

void SystemWaitSW2 (void);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define LOGICAL_DRIVE         "0:/"
#define FW_NAME_1             "/etc/fw1.bin"
#define FW_NAME_2             "/etc/fw2.bin"

#define DATA_BUF_SIZE         (4*1024)

#define FLASH_FW_AREA_START   0x00010000
#define FLASH_FW_AREA_SIZE    0x000F0000

#define FIRMWARE_STAR_ADDR    (FLASH_FW_AREA_START + 0x200)

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*
 * Some TASK variables like stack and task control block.
 */
static OS_STACK (StartStack,  TASK_START_STK_SIZE);
static OS_TCB TCBStartTask;

static FATFS    FSObject;
static FIL     hFile;
static uint8_t DataBuffer[DATA_BUF_SIZE];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  Reboot                                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void Reboot (void)
{
   uint8_t bLoopCnt;
   
   tal_LEDClear(TAL_LED_CHANNEL_1);
   tal_LEDClear(TAL_LED_CHANNEL_2);
   tal_LEDClear(TAL_LED_CHANNEL_3);
   
   for (bLoopCnt = 0; bLoopCnt < 10; bLoopCnt++)
   {
      tal_LEDToggle(TAL_LED_CHANNEL_2);
      OS_TimeDly(500);
   }      
   
   tal_CPUReboot();
   
} /* Reboot */

/*************************************************************************/
/*  Name  : DisableAllInterrupts                                         */
/*                                                                       */
/*  Disable all interrupts used before.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void DisableAllInterrupts (void)
{
   /* Disable SysTick */
   SysTick->CTRL = 0;

   NVIC_DisableIRQ(PORTE_IRQn);   
   NVIC_DisableIRQ(SDHC_IRQn);
   NVIC_DisableIRQ(UART0_RX_TX_IRQn);

} /* DisableAllInterrupts */

/*************************************************************************/
/*  Name  : StartFirmware                                                */
/*                                                                       */
/*  Start the new firmware.                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void StartFirmware (void)
{
   static void      (*StartFirm)(void);
   volatile uint32_t *pStack = (volatile uint32_t*)(FIRMWARE_STAR_ADDR + 0);
   volatile uint32_t *pReset = (volatile uint32_t*)(FIRMWARE_STAR_ADDR + 4);

   TAL_CPU_DISABLE_ALL_INTS(); 
   
   DisableAllInterrupts();  

   StartFirm = (void (*)(void))*pReset;     

   /* Initialize new stack pointer */
   __set_MSP(*pStack);
   
   /* Set CONTROL and PRIMASK to "Reset" state */
   __set_CONTROL(0);
   __set_PRIMASK(0);
   
   /* No cache to disable here */
   
   /* Start new firmware*/   
   SCB->VTOR = FIRMWARE_STAR_ADDR;
   StartFirm();

   TAL_CPU_ENABLE_ALL_INTS();   

} /* StartFirmware */

/*************************************************************************/
/*  HeaderCheck                                                          */
/*                                                                       */
/*  Check if the XBIN header is valid.                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 == OK / error cause                                        */
/*************************************************************************/
static int HeaderCheck (XBIN_HEADER *pHeader)
{
   int       rc = -1;
   uint32_t dCRC32;
   
   /* Check XBIN header */ 
   if ( (XBIN_HEADER_MAGIC_1 == pHeader->dMagic1)      &&
        (XBIN_HEADER_MAGIC_2 == pHeader->dMagic2)      &&
        (XBIN_HEADER_SIZEVER == pHeader->dSizeVersion) )
   {
      /* Check CRC32 of the header */
      dCRC32 = adler32(ADLER_START_VALUE, (uint8_t*)pHeader, sizeof(XBIN_HEADER) - XBIN_SIZE_OF_CRC32);
      if (dCRC32 == pHeader->dHeaderCRC32)
      {
         rc = 0;
      }
   }    
   
   return(rc);
} /* HeaderCheck */

/*************************************************************************/
/*  FlashCheck                                                           */
/*                                                                       */
/*  Check if the flash image is valid.                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 == OK / error cause                                        */
/*************************************************************************/
static int FlashCheck (void)
{
   int           rc = -1;
   XBIN_HEADER *pHeader = (XBIN_HEADER *)FLASH_FW_AREA_START;
   uint32_t     dCRC32;
   
   /* Check XBIN header */ 
   if ( (XBIN_HEADER_MAGIC_1 == pHeader->dMagic1)      &&
        (XBIN_HEADER_MAGIC_2 == pHeader->dMagic2)      &&
        (XBIN_HEADER_SIZEVER == pHeader->dSizeVersion) )
   {
      /* Check CRC32 of the header */
      dCRC32 = adler32(ADLER_START_VALUE, (uint8_t*)pHeader, sizeof(XBIN_HEADER) - XBIN_SIZE_OF_CRC32);
      if (dCRC32 == pHeader->dHeaderCRC32)
      {
         /* Check image */
         dCRC32 = adler32(ADLER_START_VALUE, (uint8_t*)pHeader + sizeof(XBIN_HEADER), pHeader->dDataTotalSize);
         if (dCRC32 == pHeader->dDataCRC32)
         {
            rc = 0;
         }
      }
   }    
   
   return(rc);
} /* FlashCheck */

/*************************************************************************/
/*  FlashUpdate                                                          */
/*                                                                       */
/*  Update the flash by the given file.                                  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 == OK / error cause                                        */
/*************************************************************************/
static int FlashUpdate (char *pFilename)
{
   int           rc = -1;
   FRESULT       res;
   XBIN_HEADER   FileHeader;
   uint32_t     dFlashAddr;
   uint32_t     dTotalSize;
   uint32_t     dDataRead;
   uint32_t     dBytesRead;
   
   tal_LEDClear(TAL_LED_CHANNEL_1);
   tal_LEDClear(TAL_LED_CHANNEL_2);
   tal_LEDClear(TAL_LED_CHANNEL_3);
   
   if (0 == flash_Init())
   {
      res = f_open(&hFile, pFilename, FA_READ);   
      if (FR_OK == res)
      {
         term_printf("Flash programm start\r\n");
         dFlashAddr = FLASH_FW_AREA_START;
         dTotalSize = (uint32_t)f_size(&hFile); 
         while (dTotalSize > 0)
         {
            dDataRead = MIN(DATA_BUF_SIZE, dTotalSize);
            res = f_read(&hFile, DataBuffer, dDataRead, (UINT*)&dBytesRead); 
            if (res != FR_OK)
            {
               f_close(&hFile);
               term_printf("\r\nError SD read\r\n");
               Reboot();   
            }
            
            if (dBytesRead != dDataRead)
            {
               f_close(&hFile);
               term_printf("\r\nError SD read\r\n");
               Reboot();   
            }
            
            rc = flash_EraseSector(dFlashAddr);
            if (-1 == rc)
            {
               f_close(&hFile);
               term_printf("\r\nError flash erase sector\r\n");
               Reboot();   
            }

            rc = flash_Program(dFlashAddr, DataBuffer);            
            if (-1 == rc)
            {
               f_close(&hFile);
               term_printf("\r\nError flash program\r\n");
               Reboot();   
            }
            
            dFlashAddr += DATA_BUF_SIZE;
            dTotalSize -= DATA_BUF_SIZE;
            
            term_printf(".");
            tal_LEDToggle(TAL_LED_CHANNEL_3);
         }  
         
         f_close(&hFile);
         flash_End();
         
         term_printf("\r\n");
         term_printf("Flash program end\r\n");

         term_printf("Flash check ");
         rc = FlashCheck();
         if (0 == rc)
         {
            term_printf("OK\r\n");
         }
         else
         {
            term_printf("Error\r\n");
         }   
   
      }
   }    

   tal_LEDClear(TAL_LED_CHANNEL_1);
   tal_LEDClear(TAL_LED_CHANNEL_2);
   tal_LEDClear(TAL_LED_CHANNEL_3);
   
   return(rc);
} /* FlashUpdate */

/*************************************************************************/
/*  ImageCopy                                                            */
/*                                                                       */
/*  Copies firmware image from non-volatile memory to Flash.             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void ImageCopy (void)
{
   FRESULT       res;
   uint32_t     dByteCnt;
   uint8_t      bIndex = 0;
   char        *pFWName = NULL;
   XBIN_HEADER *pFlashHeader;
   XBIN_HEADER   FileHeader;
   int          nUpdate;
   

   /*
    * Check which file must be used, 1 or 2. Read the index from the
    * "etc/fw.bin" file. If this file is not available, it must be created
    * with index 1.
    */
   res = f_open(&hFile, "/etc/fw.bin", FA_READ);
   if (FR_OK == res)
   {
      /* Read index */
      f_read(&hFile, &bIndex, sizeof(bIndex), (UINT*)&dByteCnt); 
      f_close(&hFile);
   }
   else
   {
      /* Create a "fw.bin" file with index 1 */
      res = f_open(&hFile, "/etc/fw.bin", FA_CREATE_ALWAYS | FA_WRITE);
      if (FR_OK == res)
      {
         bIndex = 1;
         f_write(&hFile, &bIndex, sizeof(bIndex), (UINT*)&dByteCnt);
         f_close(&hFile);
      }
   }      

   /*
    * Try to open the firmware file by the given index.
    */
   if (1 == bIndex) pFWName = FW_NAME_1; 
   if (2 == bIndex) pFWName = FW_NAME_2; 
   
   res = f_open(&hFile, pFWName, FA_READ);   
   if (FR_OK == res)
   {
      /* 
      * Compare file and flash version 
      */

      /* Check file header */      
      res = f_read(&hFile, &FileHeader, sizeof(FileHeader), (UINT*)&dByteCnt); 
      if (res != FR_OK) 
      {
         /* File header could not read */
         term_printf("Error SD, file header read\r\n");
         Reboot();   
      }
      if (-1 == HeaderCheck(&FileHeader)) 
      {
         /* Invalid file header */      
         f_close(&hFile);
         term_printf("Error SD, file header\"%s\"\r\n", pFWName);
         Reboot();   
      }

      f_close(&hFile);
      
      /* Check flash image */
      nUpdate = 0;
      if (-1 == FlashCheck())
      {
         /* Flash error, must update */
         nUpdate = 1;
      }
      else
      {
         /* Check if the file is newer than the flash version */
         pFlashHeader = (XBIN_HEADER *)FLASH_FW_AREA_START;
         if (FileHeader.dDataCreationTime > pFlashHeader->dDataCreationTime)
         {
            /* File version is newer, must update */
            nUpdate = 1;
         }
      }
      
      /* Check if an update is needed */
      if (1 == nUpdate)
      {
         FlashUpdate(pFWName);      
      }
   }
   else
   {
      term_printf("Unable to open \"%s\"\r\n", pFWName);
      Reboot();   
   }

} /* ImageCopy */

/*************************************************************************/
/*  OutputBootMessage                                                    */
/*                                                                       */
/*  Output boot message.                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OutputBootMessage (void)
{
   const char ResetScreen[] = { 0x1B, 'c', 0 };

   term_printf("%s", ResetScreen);
   OS_TimeDly(50);

   term_printf("\r\n");
   term_printf("*********************************\r\n");
   term_printf("  Project: %s\r\n", PROJECT_NAME);
   term_printf("  Board  : %s\r\n", TAL_BOARD);
   term_printf("  Version: v%s\r\n", PROJECT_VER_STRING);
   term_printf("  Build  : "__DATE__ " " __TIME__"\r\n");
   term_printf("*********************************\r\n");
   term_printf("\r\n");

} /* OutputBootMessage */

/*************************************************************************/
/*  StartTask                                                            */
/*                                                                       */
/*  This is the Start task.                                              */
/*                                                                       */
/*  In    : task parameter                                               */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
static void StartTask (void *p)
{
   DSTATUS DiskStatus;
   FRESULT Res;
   
   (void)p;

   /*
    * The StartTask will be used to start all other tasks in the system.
    * At the end the priority will be set to "IDLE" priority.
    */

   OS_SysTickStart();   /* Start the System ticker */
   OS_StatEnable();     /* Enable the statistic function  */

   /*******************************************************************/

   term_Start();        /* Start the Terminal functionality */
   OutputBootMessage(); /* Output startup messages */
   disk_initialize(0);  /* Initialize SD card */

   /* 
    * Check if a memory card is available
    */
   DiskStatus = disk_status(0);
   if (DiskStatus & STA_NODISK)
   {
      term_printf("SD Card not found\r\n");
      Reboot();
   }
   
   /* 
    * Mount SD card 
    */
   Res = f_mount(&FSObject, LOGICAL_DRIVE, 1);
   if (Res != FR_OK)
   {
      term_printf("SD Card mount error\r\n");
      Reboot();
   }
   
   /* 
    * Copies firmware image from non-volatile memory to Flash
    */
   ImageCopy();

   /* 
    * Start firmware
    */
   StartFirmware();
   
   /*
    * We should never come to this place here
    */   
   Reboot();
   
   OS_TaskChangePriority(TASK_START_PRIORITY_IDLE);

   while (1)
   {
      OS_TimeDly(TASK_START_DELAY_MS);
   }

} /* StartTask */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
int main (void)
{
   SystemWaitSW2();

   /*
    * Init the Tiny Abstraction Layer
    */
   tal_Init();

   /*
    * Initialize the memory pool
    */
   xmem_Init();

   /*
    * Create the StartTask.
    * The StartTask is the one and only task which
    * will start all other tasks of the system.
    */
   OS_TaskCreate(&TCBStartTask, StartTask, NULL, TASK_START_PRIORITY,
                 StartStack, sizeof(StartStack),
                 "StartTask");

   /*
    * OSStart must be the last function here.
    *
    * Fasten your seatbelt, engine will be started...
    */
   OS_Start();

   /*
    * This return here make no sense.
    * But to prevent the compiler warning:
    *    "return type of 'main' is not 'int'
    * We use an int as return :-)
    */
   return(0); /*lint !e527*/
} /* main */

/*************************************************************************/
/*  term_RxCallback                                                      */
/*                                                                       */
/*  Will be called from TermTask in case a char is received.             */
/*                                                                       */
/*  In    : bData                                                        */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void term_RxCallback (uint8_t bData)
{
   (void)bData;
   
} /* term_RxCallback */

/*************************************************************************/
/*  HardFault_Handler                                                    */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void HardFault_Handler (void)
{

#if defined(__DEBUG__)
   while (1)
   {
      __asm__ ("nop");
   }
#endif

   tal_CPUReboot();

} /* HardFault_Handler */

/*** EOF ***/
