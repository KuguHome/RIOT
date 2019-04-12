/*
 * Copyright (C) 2019 Kugu Home GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_3outof6 3 out of 6 encoder/decoder
 * @ingroup     sys
 * @brief       3 out of 6 decoder for FSK modulated signals
 *              (mostly for wireless M-Bus).
 * @{
 * @file
 * @brief       3 out of 6 encoder/decoder
 *
 * @author      Francisco Acosta <f.acosta.ext@kugu-home.com>
 *
 */

#ifndef THREE_OUT_OF_SIX_H
#define THREE_OUT_OF_SIX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Table for decoding a 6-bit "3 out of 6" encoded data into 4-bit
 *         data. The value 0xFF indicates invalid "3 out of 6" coding
 */
extern uint8_t three_out_of_six_decode_tab[64];

/**
 * @brief  Table for encoding a 4-bit data into 6-bit "3 out of 6" coded data.
 */
extern uint8_t three_out_of_six_encode_tab[16];

/**
 * @brief   List of errors on a 3 out of 6 encoded frame
 */
enum {
    /**
     * @brief   Correct encoding
     */
    THREE_OUT_OF_SIX_OK = 0,
    /**
     * @brief   Encoding error
     */
    THREE_OUT_OF_SIX_ERROR,
};

/**
 * @brief   Decode a 3 out of 6 encoded message
 *
 * Input data is mostly 24bit encoded, it outputs 16bit decoded data.
 * Thus, the function must be fed with at least a 3 byte wide array,
 * the output array must be at least 2 bytes wide in such case.
 *
 * @param[in]  encoded_data        The input data array, at least 3 byte long.
 * @param[out] decoded_data        The output data array, at least 2 bytes long.
 *
 * @return                         THREE_OUT_OF_SIX_OK on success,
 *                                 THREE_OUT_OF_SIX_ERROR on error.
 */
uint8_t three_out_of_six_decode(uint8_t *encoded_data, uint8_t *decoded_data);

/**
 * @brief   Encode a 3 out of 6 encoded message
 *
 * Input data is 16 bit decoded, it outputs 24 bit encoded data.
 * Thus, the function must be fed with at least a 2 byte wide array,
 * the output array must be at least 3 bytes wide in such case.
 *
 * @param[in]  decoded_data        The output data array, at least 2 bytes long.
 * @param[out] encoded_data        The input data array, at least 3 byte long.
 *
 */
void three_out_of_six_encode(uint8_t *decoded_data, uint8_t *encoded_data);

#ifdef __cplusplus
}
#endif
#endif /* THREE_OUT_OF_SIX_H */
/** @} */
