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
*  01.05.2018  mifi  First Version, tested with a FRDM-K64F board.
**************************************************************************/
#if !defined(__ENET_K64_H__)
#define __ENET_K64_H__

/**************************************************************************
*  Includes
**************************************************************************/
#include <stdint.h>

/**************************************************************************
*  Global Definitions
**************************************************************************/

#define ENET_RXBD_NUM      (6)
#define ENET_TXBD_NUM      (4)
#define ENET_RXBUFF_SIZE   (1520)
#define ENET_TXBUFF_SIZE   (1520)

/**************************************************************************
*  Macro Definitions
**************************************************************************/

/**************************************************************************
*  Functions Definitions
**************************************************************************/

void     enet_Init (uint8_t UseRMII);
void     enet_Start (void);

uint32_t enet_GetIRQStatus (void);
void     enet_ClearIRQStatus(uint32_t dStatus);

void     enet_MACAddressConfig (uint8_t *pBuffer);
void     enet_UpdateLink (uint8_t bLinkStatus);

int      enet_MulticastAddFilter (uint32_t dAddress);
int      enet_MulticastDelFilter (uint32_t dAddress);

uint16_t enet_PHYRegRead (uint16_t wAddress, uint16_t wReg);
void     enet_PHYRegWrite (uint16_t wAddress, uint16_t wReg, uint16_t wValue);

uint8_t *enet_RxFrameGet (uint16_t *pSize);
void     enet_RxFrameReady (void);
void     enet_RxOverrun (void);

uint8_t *enet_TxFrameGet (void);
void     enet_TxFrameSend(uint16_t wSize);
void     enet_TxUnderrun (void);

#endif /* !__ENET_K64_H__ */

/*** EOF ***/
