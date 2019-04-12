/*
 * Copyright (C) 2019 Kugu Home GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_3outof6
 * @{
 *
 * @file
 * @brief       3 out of 6 decoder/encoder
 *
 * @author      Francisco Acosta <f.acosta.ext@kugu-home.com>
 *
 * @}
 */

#include <stdio.h>

#include "three_out_of_six.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

                                         /* 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 */
uint8_t three_out_of_six_decode_tab[64] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                         /* 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F */
                                            0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0x01, 0x02, 0xFF,
                                         /* 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 */
                                            0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x00, 0xFF,
                                         /* 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F */
                                            0xFF, 0x05, 0x06, 0xFF, 0x04, 0xFF, 0xFF, 0xFF,
                                         /* 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 */
                                            0xFF, 0xFF, 0xFF, 0x0B, 0xFF, 0x09, 0x0A, 0xFF,
                                         /* 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F */
                                            0xFF, 0x0F, 0xFF, 0xFF, 0x08, 0xFF, 0xFF, 0xFF,
                                         /* 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37 */
                                            0xFF, 0x0D, 0x0E, 0xFF, 0x0C, 0xFF, 0xFF, 0xFF,
                                         /* 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F */
                                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

                                         /* 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 */
uint8_t three_out_of_six_encode_tab[16] = { 0x16, 0x0D, 0x0E, 0x0B, 0x1C, 0x19, 0x1A, 0x13,
                                         /* 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F */
                                            0x2C, 0x25, 0x26, 0x23, 0x34, 0x31, 0x32, 0x29,
};


uint8_t three_out_of_six_decode(uint8_t *encoded_data, uint8_t *decoded_data)
{
    uint8_t data[4];

    DEBUG("[3outof6] decoding 0x%02X\n", *encoded_data);

    data[0] = three_out_of_six_decode_tab[(*(encoded_data + 2) & 0x3F)];
    data[1] = three_out_of_six_decode_tab[((*(encoded_data + 2) & 0xC0) >> 6) | ((*(encoded_data + 1) & 0x0F) << 2)];
    data[2] = three_out_of_six_decode_tab[((*(encoded_data + 1) & 0xF0) >> 4) | ((*encoded_data & 0x03) << 4)];
    data[3] = three_out_of_six_decode_tab[((*encoded_data & 0xFC) >> 2)];


    /* 0xFF represents invalid data, if so discard it */
    if ((data[0] == 0xFF) | (data[1] == 0xFF) |
        (data[2] == 0xFF) | (data[3] == 0xFF) ) {
        DEBUG("[3outof6] data[0]: 0x%02X\n", data[0]);
        DEBUG("[3outof6] data[1]: 0x%02X\n", data[1]);
        DEBUG("[3outof6] data[2]: 0x%02X\n", data[2]);
        DEBUG("[3outof6] data[3]: 0x%02X\n", data[3]);
        return THREE_OUT_OF_SIX_ERROR;
    }

    /* Shift the encoded values into a byte buffer */
    *decoded_data = (data[3] << 4) | (data[2]);
    *(decoded_data + 1) = (data[1] << 4) | (data[0]);

    return THREE_OUT_OF_SIX_OK;
}

void three_out_of_six_encode(uint8_t *uncoded_data, uint8_t *encoded_data)
{
    uint8_t data[4] = { 0 };

    DEBUG("[3outof6] encoding 0x%02X\n", *uncoded_data);

    data[0] = three_out_of_six_encode_tab[*(uncoded_data + 1) & 0x0F];
    data[1] = three_out_of_six_encode_tab[(*(uncoded_data + 1) >> 4) & 0x0F];

    data[2] = three_out_of_six_encode_tab[(*uncoded_data) & 0x0F];
    data[3] = three_out_of_six_encode_tab[((*uncoded_data) >> 4) & 0x0F];


    /* Shift the encoded 6-bit values into a byte buffer */
    *(encoded_data + 0) = (data[3] << 2) | (data[2] >> 4);
    *(encoded_data + 1) = (data[2] << 4) | (data[1] >> 2);
    *(encoded_data + 2) = (data[1] << 6) | data[0];

}
