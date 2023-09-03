/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if !defined(__ETHERNETIF_H__)
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"
#include "fsl_enet.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ENET_RXBD_NUM         (5)
#define ENET_TXBD_NUM         (3)

#define ENET_RXBUFF_SIZE      (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE      (ENET_FRAME_MAX_FRAMELEN)

/* Interrupt priorities. */
#define ENET_PRIORITY         (6U)
#define ENET_1588_PRIORITY    (5U)

/**
 * This function should be passed as a parameter to netif_add()
 * if you initialize the first ENET interface.
 */
err_t ethernetif_init(struct netif *netif);
err_t ethernetif1_init(struct netif *netif);

/*
 * Ethernet "software" IRQ handler.
 */
void ENET_DriverIRQHandler (void);
void ENET2_DriverIRQHandler (void);

#endif /* __ETHERNETIF_H__ */
/*** EOF ***/
