/*
 * Copyright (C) 2019 Kugu Home GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_fsk FSK modulation
 * @ingroup     net
 * @brief       FSK modulation header definitions
 * @{
 *
 * @file
 * @brief       FSK modulation header definitions
 *
 * @author      Francisco Acosta <f.acosta.ext@kugu-home.com>
 */

#ifndef NET_FSK_H
#define NET_FSK_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup net_fsk_conf  FSK modulation compile configurations
 * @ingroup  config
 * @{
 */
/**
 * @name    FSK defaults
 * @{
 */

/** @brief Frequency step in Hz */
#ifndef FSK_FREQ_STEP_DEFAULT
#define FSK_FREQ_STEP_DEFAULT      (61.03515625)              /**< Step frequency between channels */
#endif
#ifndef FSK_CHANNEL_DEFAULT
#define FSK_CHANNEL_DEFAULT        (868950000UL)              /**< Default FSK frequency, 868.95MHz (Europe) */
#endif
#ifndef FSK_HF_CHANNEL_DEFAULT
#define FSK_HF_CHANNEL_DEFAULT     (860000000UL)              /**< Use to calibrate RX chain for LF and HF bands */
#endif
#ifndef FSK_BITRATE_DEFAULT
#define FSK_BITRATE_DEFAULT        (32768UL)                 /**< Bitrate at 32.768kpbs */
#endif
#ifndef FSK_FREQ_DEV_DEFAULT
#define FSK_FREQ_DEV_DEFAULT       (38000UL)                  /**< 38KHz frequency deviation */
#endif
#ifndef FSK_BANDWIDTH_DEFAULT
#define FSK_BANDWIDTH_DEFAULT      (FSK_BITRATE_DEFAULT + 2 \
		                             * FSK_FREQ_DEV_DEFAULT)  /**< Bandwidth calculation */
#endif
#ifndef FSK_AFC_BANDWIDTH_DEFAULT
#define FSK_AFC_BANDWIDTH_DEFAULT  (200000UL)                 /**< 20KHz AFC Bandwidth */
#endif
#ifndef FSK_PAYLOADLENGTH_DEFAULT
#define FSK_PAYLOADLENGTH_DEFAULT  (255)                      /**< Default payload length */
#endif
#ifndef FSK_SYNCWORD_SYNCVALUE1
#define FSK_SYNCWORD_SYNCVALUE1    (0x54)                     /**< Sync value for testing purposes (wM-Bus) */
#endif
#ifndef FSK_SYNCWORD_SYNCVALUE2
#define FSK_SYNCWORD_SYNCVALUE2    (0x3D)                     /**< Sync value for testing purposes (wM-Bus) */
#endif
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* NET_FSK_H */
/** @} */
