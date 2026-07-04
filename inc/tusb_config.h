/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define CFG_TUD_ENABLED         (1)

// Legacy RHPORT configuration
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT        (0)
#endif

//------------------------
// DEVICE CONFIGURATION //
//------------------------

// 7 interfaces CDC:
// CDC 0: debug
// CDC 1: configuración full duplex
// CDC 2: datos ADS canales 1 y 2
// CDC 3: datos ADS canales 3 y 4
// CDC 4: datos ADS canales 5 y 6
// CDC 5: datos ADS canales 7 y 8
// CDC 6: RLD
#define CFG_TUD_CDC             (7)

#define CFG_TUD_CDC_RX_BUFSIZE  (64)
#define CFG_TUD_CDC_TX_BUFSIZE  (64)
#define CFG_TUD_CDC_EP_BUFSIZE  (64)

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  (64)
#endif

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
