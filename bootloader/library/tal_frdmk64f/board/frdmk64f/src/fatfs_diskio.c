/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2023 by Michael Fischer (www.emb4fun.de).
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

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include <string.h>
#include "tal.h"
#include "ff.h"
#include "diskio.h"

#include "fsl_sd.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_common.h"

/*=======================================================================*/
/*  Global                                                               */
/*=======================================================================*/

/*=======================================================================*/
/*  Extern                                                               */
/*=======================================================================*/

void SDHC_DriverIRQHandler(void);
void BOARD_SD_Config (sd_card_t *card, sd_cd_t cd, uint32_t hostIRQPriority, void *userData);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define SWISSBIT_DELAY_MS  500   /* It works with 200ms, but for robustness we use 500ms */

#define BLOCK_SIZE   512

#define SD_LED_ON()
#define SD_LED_OFF()

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*
 * sd card descriptor
 */
static sd_card_t SDCard;

static DSTATUS DiskStatus = STA_NOINIT;

static OS_SEMA Mutex[FF_VOLUMES + 1];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  GPIO_CONFIG                                                          */
/*                                                                       */
/*  Configures the GPIO ports.                                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void GPIO_CONFIG (void)
{
   /* Port E Clock Gate Control: Clock enabled */
   CLOCK_EnableClock(kCLOCK_PortE);

   const port_pin_config_t porte0_config = {/* Internal pull-up resistor is enabled */
                                                    kPORT_PullUp,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as SDHC0_D1 */
                                                    kPORT_MuxAlt4,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
   /* PORTE0 is configured as SDHC0_D1 */
   PORT_SetPinConfig(PORTE, 0U, &porte0_config);

   const port_pin_config_t porte1_config = {/* Internal pull-up resistor is enabled */
                                                    kPORT_PullUp,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as SDHC0_D0 */
                                                    kPORT_MuxAlt4,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
   /* PORTE1 is configured as SDHC0_D0 */
   PORT_SetPinConfig(PORTE, 1U, &porte1_config);

   const port_pin_config_t porte2_config = {/* Internal pull-up resistor is enabled */
                                                    kPORT_PullUp,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as SDHC0_DCLK */
                                                    kPORT_MuxAlt4,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
   /* PORTE2 is configured as SDHC0_DCLK */
   PORT_SetPinConfig(PORTE, 2U, &porte2_config);

   const port_pin_config_t porte3_config = {/* Internal pull-up resistor is enabled */
                                                    kPORT_PullUp,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as SDHC0_CMD */
                                                    kPORT_MuxAlt4,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
   /* PORTE3 is configured as SDHC0_CMD */
   PORT_SetPinConfig(PORTE, 3U, &porte3_config);

   const port_pin_config_t porte4_config = {/* Internal pull-up resistor is enabled */
                                                    kPORT_PullUp,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as SDHC0_D3 */
                                                    kPORT_MuxAlt4,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
   /* PORTE4 is configured as SDHC0_D3 */
   PORT_SetPinConfig(PORTE, 4U, &porte4_config);

   const port_pin_config_t porte5_config = {/* Internal pull-up resistor is enabled */
                                                    kPORT_PullUp,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as SDHC0_D2 */
                                                    kPORT_MuxAlt4,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
   /* PORTE5 is configured as SDHC0_D2 */
   PORT_SetPinConfig(PORTE, 5U, &porte5_config);

   const port_pin_config_t porte6_config = {/* Internal pull-up resistor is enabled */
                                                   kPORT_PullDown,
                                                   /* Fast slew rate is configured */
                                                   kPORT_FastSlewRate,
                                                   /* Passive filter is disabled */
                                                   kPORT_PassiveFilterDisable,
                                                   /* Open drain is disabled */
                                                   kPORT_OpenDrainDisable,
                                                   /* Low drive strength is configured */
                                                   kPORT_LowDriveStrength,
                                                   /* Pin is configured as PTB5 */
                                                   kPORT_MuxAsGpio,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   kPORT_UnlockRegister};
   /* PORTE6 is configured as PTE6 */
   PORT_SetPinConfig(PORTE, 6U, &porte6_config);

} /* GPIO_CONFIG */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  ff_memalloc                                                          */
/*                                                                       */
/*  Allocate a memory block                                              */
/*                                                                       */
/*  In    : msize                                                        */
/*  Out   : none                                                         */
/*  Return: Returns pointer to the allocated memory block                */
/*************************************************************************/
void *ff_memalloc (UINT msize)
{
   return(xmalloc(XM_ID_FS, msize));
} /* ff_memalloc */

/*************************************************************************/
/*  ff_memfree                                                           */
/*                                                                       */
/*  Free a memory block                                                  */
/*                                                                       */
/*  In    : mblock                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_memfree (void *mblock)
{
   xfree(mblock);
} /* ff_memfree */

/*************************************************************************/
/*  ff_mutex_create                                                      */
/*                                                                       */
/*  Create a Synchronization Object                                      */
/*                                                                       */
/*  In    : vol,  corresponding logical drive being processed.           */
/*  Out   : none                                                         */
/*  Return: 0 = Error / 1 = OK                                           */
/*************************************************************************/
int ff_mutex_create (int vol)
{
   OS_SemaCreate(&Mutex[vol], 1, 1);

   return(1);
} /* ff_mutex_create */

/*************************************************************************/
/*  ff_mutex_delete                                                      */
/*                                                                       */
/*  Delete a Synchronization Object                                      */
/*                                                                       */
/*  In    : vol,  corresponding logical drive being processed.           */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_mutex_delete (int vol)
{
   OS_SemaDelete(&Mutex[vol]);
} /* ff_mutex_delete */

/*************************************************************************/
/*  ff_mutex_take                                                        */
/*                                                                       */
/*  Request Grant to Access the Volume                                   */
/*                                                                       */
/*  In    : vol,  corresponding logical drive being processed.           */
/*  Out   : none                                                         */
/*  Return: 1 = Got a grant / 0 =  Could not get a grant                 */
/*************************************************************************/
int ff_mutex_take (int vol)
{
   OS_SemaWait(&Mutex[vol], OS_WAIT_INFINITE);

   return(1);
} /* ff_mutex_take */

/*************************************************************************/
/*  ff_mutex_give                                                         */
/*                                                                       */
/*  Request Grant to Access the Volume                                   */
/*                                                                       */
/*  In    : vol,  corresponding logical drive being processed.           */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_mutex_give (int vol)
{
   OS_SemaSignal(&Mutex[vol]);
} /* ff_mutex_give */

/*************************************************************************/
/*  disk_initialize                                                      */
/*                                                                       */
/*  Initialize a Drive                                                   */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: Status of Disk Functions                                     */
/*************************************************************************/
DSTATUS disk_initialize (uint8_t pdrv)
{
   static int InitDone = FALSE;

   if (DiskStatus & STA_NODISK)
   {
      /* No card in the socket */
      return(DiskStatus);
   }

   SD_LED_ON();

   if (0 == pdrv)
   {
      if (FALSE == InitDone)
      {
         InitDone = TRUE;

         GPIO_CONFIG();

         /* Clear data first */
         memset(&SDCard, 0x00, sizeof(SDCard));

         DiskStatus |= STA_NOINIT;

      } /* end if (FALSE == InitDone) */


      /*
       * Initialize host interface
       */
      if (false == SDCard.isHostReady)
      {
         BOARD_SD_Config(&SDCard, NULL, 5, NULL);

         SD_HostInit(&SDCard);
         SD_SetCardPower(&SDCard, false); /*lint !e747*/
      }


      if (DiskStatus & STA_NOINIT)
      {
         if (true == SD_IsCardPresent(&SDCard))
         {
            /* Reset host */
            SD_HostDoReset(&SDCard);
            SD_SetCardPower(&SDCard, true);  /*lint !e747*/

            if (kStatus_Success != SD_CardInit(&SDCard))
            {
               SD_CardDeinit(&SDCard);
               memset(&SDCard, 0U, sizeof(SDCard));

               DiskStatus |= STA_NOINIT;
            }
            else
            {
               term_printf("SD busClock Hz: %d\r\n\r\n", SDCard.busClock_Hz);
               DiskStatus &= ~STA_NOINIT;

               /* It looks like a delay is needed for the swissbit card */
               OS_TimeDly(OS_MS_2_TICKS(SWISSBIT_DELAY_MS));
            }
         }
      }

   } /* end if (0 == pdrv) */

   SD_LED_OFF();

   return(DiskStatus);
} /* disk_initialize */

/*************************************************************************/
/*  disk_removed                                                         */
/*                                                                       */
/*  Medium was removed                                                   */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void disk_removed (BYTE pdrv)
{
   if (0 == pdrv)
   {
      /* Remove power first */
      SD_SetCardPower(&SDCard, false); /*lint !e747*/

      /* DeInit card */
      SD_CardDeinit(&SDCard);
      SDCard.isHostReady = false;

      DiskStatus |= (STA_NODISK | STA_NOINIT);
   }

} /* disk_removed */

/*************************************************************************/
/*  disk_status                                                          */
/*                                                                       */
/*  Get Drive Status                                                     */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: Status of Disk Functions                                     */
/*************************************************************************/
DSTATUS disk_status (uint8_t pdrv)
{
   if (0 == pdrv)
   {
      DiskStatus |= STA_NODISK;

      if (true == SD_IsCardPresent(&SDCard))
      {
         DiskStatus &= ~STA_NODISK;
      }
   }
   else
   {
      return(STA_NOINIT);
   }

   return(DiskStatus);
} /* disk_status */

/*************************************************************************/
/*  disk_read                                                            */
/*                                                                       */
/*  Read Sector(s)                                                       */
/*                                                                       */
/*  In    : pdrv,   physical drive nmuber to identify the drive          */
/*          buff,   data buffer to store read data                       */
/*          sector, sector address in LBA                                */
/*          count,  number of sectors to read                            */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
   DRESULT  Result = RES_ERROR;

   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }

      SD_LED_ON();

      if (kStatus_Success == SD_ReadBlocks(&SDCard, buff, sector, count))
      {
         Result = RES_OK;
      }

      SD_LED_OFF();
   }

   return(Result);
} /* disk_read */

/*************************************************************************/
/*  disk_write                                                           */
/*                                                                       */
/*  Write Sector(s)                                                      */
/*                                                                       */
/*  In    : pdrv,   physical drive nmuber to identify the drive          */
/*          buff,   data to be written                                   */
/*          sector, sector address in LBA                                */
/*          count,  number of sectors to write                           */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
   DRESULT  Result = RES_ERROR;

   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }

      SD_LED_ON();

      if (kStatus_Success == SD_WriteBlocks(&SDCard, buff, sector, count))
      {
         Result = RES_OK;
      }

      SD_LED_OFF();
   }

   return(Result);
} /* disk_write */

/*************************************************************************/
/*  disk_ioctl                                                           */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*          cmd,  control code                                           */
/*          buff, buffer to send/receive control data                    */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_ioctl (uint8_t pdrv, uint8_t cmd, void *buff)
{
   DRESULT Result = RES_PARERR;

   if ((0 == pdrv) && (buff != NULL))
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }

      SD_LED_ON();

      switch(cmd)
      {
         case CTRL_SYNC:         /* Make sure that no pending write process */
            Result = RES_OK;
            break;

         case GET_SECTOR_COUNT:  /* Get number of sectors on the disk (DWORD) */
            *(DWORD*)buff = SDCard.blockCount;
            Result = RES_OK;
            break;

         case GET_SECTOR_SIZE:   /* Get R/W sector size (WORD) */
            *(WORD*)buff = (WORD)SDCard.blockSize;
            Result = RES_OK;
            break;

         case GET_BLOCK_SIZE:    /* Get erase block size in unit of sector (DWORD) */
            *(DWORD*)buff = SDCard.csd.eraseSectorSize;
            Result = RES_OK;
            break;

         default:
            Result = RES_PARERR;
            break;
      }

      SD_LED_OFF();
   }

   return(Result);
} /* disk_ioctl */

/*************************************************************************/
/*  get_fattime                                                          */
/*                                                                       */
/*  Gets Time from RTC                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Time                                                         */
/*************************************************************************/
DWORD get_fattime (void)
{
   return(0);
}

void SDHC_IRQHandler(void)
{
   SDHC_DriverIRQHandler();
}

/*** EOF ***/
