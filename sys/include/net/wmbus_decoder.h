/*
 * Copyright (C) 2019 Kugu Home GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_wmbus Wireless M-Bus decoder
 * @ingroup     net
 * @brief       This module decodes bytes coming from a FSK modulated
 *              signal in a 3 out of 6 encoding
 * @{
 * @file
 * @brief       Wireless M-Bus decoder
 *
 * @author      Francisco Acosta <f.acosta.ext@kugu-home.com>
 *
 */

#ifndef NET_WMBUS_DECODER_H
#define NET_WMBUS_DECODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Polynomial for CRC calculation on wireless M-Bus frames.
 */
#define WMBUS_CRC_POLY                 0x3D65

enum {
    WMBUS_DECODER_OK = 0,
    WMBUS_DECODER_CODING_ERROR,
    WMBUS_DECODER_CRC_ERROR,
};

/**
 * @brief   Decode an encoded 3 out of 6 wireless M-Bus frame. Checks
 *          for CRC and 3 out of 6 errors.
 *
 * Data is fed into the decode function and a buffer is filled with the
 * decoded data. In case of an error the decoding halts and gives back
 * an error code.
 *
 * @param[in]  wmbus_encoded       The encoded data frame, at least
 *                                 3 byte long.
 * @param[out] wmbus_decoded       The decoded wireles M-bus frame,
 *                                 at least 1 byte long plus 2 bytes
 *                                 CRC.
 * @param[in]  wmbus_dec_len       The length of the decoded frame.
 *                                 This is the first 3 encoded bytes
 *                                 of the beginning of a frame.
 *
 * @return                         WMBUS_DECODER_OK on success,
 *                                 WMBUS_DECODER_CODING_ERROR on 3 out
 *                                 of 6 error,
 *                                 WMBUS_DECODER_CRC_ERROR on CRC error.
 */
uint16_t wmbus_decoder_tmode(uint8_t *wmbus_encoded, uint8_t *wmbus_decoded, uint16_t wmbus_dec_len);

#ifdef __cplusplus
}
#endif
#endif /* NET_WMBUS_DECODER_H */
/** @} */
