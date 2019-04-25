/*
 * Copyright (C) 2019 Kugu Home GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_wmbus
 * @{
 *
 * @file
 * @brief       Wireless M-Bus decoder
 *
 * @author      Francisco Acosta <f.acosta.ext@kugu-home.com>
 *
 * @}
 */

#include <stdio.h>

#include "net/wmbus_decoder.h"
#include "checksum/ucrc16.h"
#include "three_out_of_six.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

uint16_t wmbus_decoder_tmode(uint8_t *wmbus_encoded, uint8_t *wmbus_decoded, uint16_t wmbus_dec_len)
{
    int16_t bytes_remaining = wmbus_dec_len;
    int16_t bytes_encoded = 0;
    uint8_t decoding_status;
    uint16_t crc = 0;              /* Current CRC value */
    uint8_t crc_field = 0;         /* Current fields are a CRC field */

    /* Decode packet */
    while (bytes_remaining)
    {
        /*
         * If the length of the decoded data is odd, perform only the
         * decoding of the two last bytes
         */
        if (bytes_remaining == 1) {
            uint8_t last_bytes[2];
            last_bytes[0] = three_out_of_six_decode_tab[((*(wmbus_encoded + 1) & 0xF0) >> 4) | ((*wmbus_encoded & 0x03) << 4)];
            last_bytes[1] = three_out_of_six_decode_tab[((*wmbus_encoded & 0xFC) >> 2)];
            *wmbus_decoded = (last_bytes[1] << 4) | (last_bytes[0]);

            if ( (last_bytes[0] == 0xFF) | (last_bytes[1] == 0xFF)) {
                return (WMBUS_DECODER_CODING_ERROR);
            }

            bytes_remaining  -= 1;
            bytes_encoded    += 1;

            /* The last byte the low byte of the CRC field */
            if ((~crc & 0xFF) != *(wmbus_decoded )) {
                DEBUG("[wbus_decoder] CRC Error on byte: 0x%01X\n", *wmbus_decoded);
                DEBUG("[wmbus_decoder] CRC Error: 0x%02X\n", (~crc & 0xFF));
                return (WMBUS_DECODER_CRC_ERROR);
            }
        }
        else {
            decoding_status = three_out_of_six_decode(wmbus_encoded, wmbus_decoded);

            /* Check for valid 3 out of 6 decoding */
            if (decoding_status != THREE_OUT_OF_SIX_OK) {
                return (WMBUS_DECODER_CODING_ERROR);
            }

            bytes_remaining -= 2;
            bytes_encoded  += 2;


            // Check if current field is CRC fields
            // - Field 10 + 18*n
            // - Less than 2 bytes
            if (bytes_remaining == 0) {
                crc_field = 1;
            }
            else if ( bytes_encoded > 10 ) {
                crc_field = !((bytes_encoded - 12) % 18);
            }

            /* If it's the last byte, the first 10 bytes or a block
             *  of 16 bytes, check CRC
             */
            if (crc_field) {
                if ((~crc & 0xFF) != *(wmbus_decoded + 1 )) {
                    DEBUG("[wbus_decoder] CRC Error on byte: 0x%01X\n", *wmbus_decoded + 1);
                    DEBUG("[wmbus_decoder] CRC Error: 0x%02X\n", (~crc & 0xFF));
                    return (WMBUS_DECODER_CRC_ERROR);
                }
                if (((~crc >> 8) & 0xFF) != *wmbus_decoded) {
                    DEBUG("[wbus_decoder] CRC Error on byte: 0x%01X\n", *wmbus_decoded);
                    DEBUG("[wmbus_decoder] CRC Error: 0x%02X\n", ((~crc >> 8) & 0xFF));
                    return (WMBUS_DECODER_CRC_ERROR);
                }

                crc_field = 0;
                crc = 0;
            }
            else if (bytes_remaining == 1) { /* If 1 bytes left, the field is the high byte of the CRC */
                crc = ucrc16_calc_be(wmbus_decoded, 1, 0x3D65, crc);
                /*The packet byte is a CRC-field */
                if (((~crc >> 8) & 0xFF) != *(wmbus_decoded + 1)) {
                    DEBUG("[wbus_decoder] CRC Error on byte: 0x%01X\n", *wmbus_decoded + 1);
                    DEBUG("[wmbus_decoder] CRC Error: 0x%02X\n", ((~crc >> 8) & 0xFF));
                    return (WMBUS_DECODER_CRC_ERROR);
                }
            }
            else { /* Perform CRC calculation */
                crc = ucrc16_calc_be(wmbus_decoded, 1, 0x3D65, crc);
                DEBUG("[wbus_decoder] CRC on byte: 0x%01X\n", *wmbus_decoded);
                DEBUG("[wmbus_decoder] CRC High: 0x%02X\n", crc);
                DEBUG("[wmbus_decoder] CRC complement: 0x%02X\n", ~crc);
                crc = ucrc16_calc_be(wmbus_decoded + 1, 1, 0x3D65, crc);
                DEBUG("[wbus_decoder] CRC on byte: 0x%01X\n", *wmbus_decoded + 1);
                DEBUG("[wmbus_decoder] CRC Low: 0x%02X\n", crc);
                DEBUG("[wmbus_decoder] CRC complement: 0x%02X\n", ~crc);
            }

            wmbus_encoded += 3;
            wmbus_decoded += 2;
        }
    }
    return (WMBUS_DECODER_OK);
}
