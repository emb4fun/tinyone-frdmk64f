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
#define __XMEMPOOL_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include "flash.h"
#include "terminal.h"

#include "fsl_flash.h"
#include "fsl_ftfx_cache.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/* Flash driver Structure */
static flash_config_t flashDriver;

/* Flash cache driver Structure */
static ftfx_cache_config_t cacheDriver;

static uint32_t FlashBlockBase  = 0;
static uint32_t FlashTotalSize  = 0;
static uint32_t FlashSectorSize = 0;

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  flash_Init                                                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: 0 == OK / error cause                                        */
/*************************************************************************/
int flash_Init (void)
{
   int      rc = -1;
   status_t res;

   ftfx_security_state_t securityStatus = kFTFx_SecurityStateNotSecure;

   /* Clean up Flash, Cache driver Structure*/
   memset(&flashDriver, 0, sizeof(flash_config_t));
   memset(&cacheDriver, 0, sizeof(ftfx_cache_config_t));

   /* Setup flash driver structure for device and initialize variables. */
   res = FLASH_Init(&flashDriver);
   if (res != kStatus_FTFx_Success) goto end;

   /* Setup flash cache driver structure for device and initialize variables. */
   res = FTFx_CACHE_Init(&cacheDriver);
   if (res != kStatus_FTFx_Success) goto end;

   /* Get flash properties*/
   FLASH_GetProperty(&flashDriver, kFLASH_PropertyPflash0BlockBaseAddr, &FlashBlockBase);
   FLASH_GetProperty(&flashDriver, kFLASH_PropertyPflash0TotalSize,     &FlashTotalSize);
   FLASH_GetProperty(&flashDriver, kFLASH_PropertyPflash0SectorSize,    &FlashSectorSize);

   /* Check security status. */
   res = FLASH_GetSecurityState(&flashDriver, &securityStatus);
   if (res != kStatus_FTFx_Success) 
   {
      term_printf("Error FLASH_GetSecurityState\r\n");
      goto end;   
   }
   
   /* Test pflash basic opeation only if flash is unsecure. */
   if (kFTFx_SecurityStateNotSecure == securityStatus)
   {
      /* Pre-preparation work about flash Cache/Prefetch/Speculation. */
      FTFx_CACHE_ClearCachePrefetchSpeculation(&cacheDriver, true);
            
      rc = 0;
   }
   else
   {
      term_printf("Error flash is secure\r\n");
   }

end:

   return(rc);
} /* flash_Init */

/*************************************************************************/
/*  flash_EraseSector                                                    */
/*                                                                       */
/*  In    : dAddress                                                     */
/*  Out   : none                                                         */
/*  Return: 0 == OK / error cause                                        */
/*************************************************************************/
int flash_EraseSector (uint32_t dAddress)
{
   int      rc = -1;
   status_t res;

#if defined(FSL_SUPPORT_ERASE_SECTOR_NON_BLOCKING) && FSL_SUPPORT_ERASE_SECTOR_NON_BLOCKING
   res = FLASH_EraseSectorNonBlocking(&flashDriver, dAddress, kFTFx_ApiEraseKey);
   if (kStatus_FTFx_Success != res) goto end;
   
   /* Before programming the flash, check whether the erase sector command is completed,*/
   /* and get the flash status. */
   res = FLASH_GetCommandState();
   if (kStatus_FTFx_Success != res) goto end;

#else
   res = FLASH_Erase(&flashDriver, dAddress, FlashSectorSize, kFTFx_ApiEraseKey);
   if (kStatus_FTFx_Success != res) goto end;
#endif

   /* Verify sector if it's been erased. */
   res = FLASH_VerifyErase(&flashDriver, dAddress, FlashSectorSize, kFTFx_MarginValueUser);
   if (kStatus_FTFx_Success != res)  goto end;
   
   rc = 0;
   
end:   
   
   return(rc);
} /* flash_EraseSector */

/*************************************************************************/
/*  flash_Program                                                        */
/*                                                                       */
/*  In    : dAddress, pBuffer                                            */
/*  Out   : none                                                         */
/*  Return: 0 == OK / error cause                                        */
/*************************************************************************/
int flash_Program (uint32_t dAddress, uint8_t *pBuffer)
{
   int      rc = -1;
   status_t res;
   uint32_t failAddr;
   uint32_t failDat;

   /* Program user buffer into flash*/
   res = FLASH_Program(&flashDriver, dAddress, pBuffer, FlashSectorSize);
   if (kStatus_FTFx_Success != res) goto end;

   /* Verify programming by Program Check command with user margin levels */
   res = FLASH_VerifyProgram(&flashDriver, dAddress, FlashSectorSize, pBuffer,
                             kFTFx_MarginValueUser, &failAddr, &failDat);
   if (kStatus_FTFx_Success != res) goto end;

   rc = 0;

end:   
   
   return(rc);
} /* flash_Program */

/*************************************************************************/
/*  flash_End                                                            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void flash_End (void)
{
   /* Post-preparation work about flash Cache/Prefetch/Speculation. */
   FTFx_CACHE_ClearCachePrefetchSpeculation(&cacheDriver, false);

} /* flash_End */

/*** EOF ***/
