/*
 * Copyright (C) 2019 Kugu Home GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Tests three_out_of_six encoder/decoder module.
 *
 * @author      Francisco Acosta <f.acosta.ext@kugu-home.com>
 *
 * @}
 */

#include <stdio.h>

#include "three_out_of_six.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/*
 * Encoded data from a real-world wireless M-Bus signal
 */
static uint8_t encoded[] = { 0x36, 0x57, 0x1C, 0x39, 0xC3, 0x8B, 0xB1, 0x35,
                             0x93, 0x38, 0xB7, 0x0E, 0x34, 0xD7, 0x13, 0xC9,
                             0x96, 0x63, 0x98, 0xD5, 0x8D, 0x37, 0x25, 0x8B,
                             0x35, 0xA9, 0xB1, 0xC6, 0x6B, 0x0B, 0x2E, 0x56,
                             0x6C, 0x3A, 0x92, 0xCD, 0x99, 0x3C, 0x93, 0xC4,
                             0xBD, 0x26, 0x59, 0xAA, 0x53,
};

/*
 * Decoded data of the previous array, from reference sources
 */

static uint8_t result[] = { 0x19, 0x44, 0x24, 0x23, 0x87, 0x07, 0x23, 0x42,
                            0x11, 0x47, 0xE5, 0x5B, 0xA1, 0x01, 0x1E, 0x03,
                            0x16, 0xAD, 0xDA, 0x83, 0x39, 0x58, 0x2F, 0x31,
                            0xA7, 0xE7, 0xD3, 0xCA, 0x06, 0xF7,
};

static uint8_t decoded[sizeof(encoded)];

static void _print_bytes(uint8_t *bytes, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        printf("0x%02X, ", bytes[i]);
    }

    puts("");
}

int main(void)
{
    unsigned i = 0;
    unsigned j = 0;
    uint16_t bytes_remaining = sizeof(result);
    uint16_t bytes_decoded = 0;

    printf("Decoding %d bytes of data...\n", bytes_remaining);
    _print_bytes(encoded, sizeof(encoded));

    while (bytes_remaining) {
        int error = 0;
        error = three_out_of_six_decode(&encoded[i], &decoded[j]);

        if (error == THREE_OUT_OF_SIX_ERROR) {
            puts("Some error occurred!");
        }

        bytes_remaining -= 2;
        bytes_decoded  += 2;

        i += 3;
        j += 2;
    }

    for (size_t i = 0; i < sizeof(result); i++) {
        if (decoded[i] != result[i]) {
            printf("[FAILURE]\n");
            return -1;
        }
    }

    printf("Decoded %d length data...\n", bytes_decoded);
    _print_bytes(decoded, sizeof(result));

    puts("[SUCCESS]");

    return 0;
}
