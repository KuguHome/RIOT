/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 * @file
 * @brief       Test application for SX127X modem driver
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "thread.h"
#include "xtimer.h"
#include "shell.h"
#include "shell_commands.h"

#include "net/netdev.h"
#include "net/lora.h"

#include "three_out_of_six.h"

#include "board.h"

#include "sx127x_internal.h"
#include "sx127x_params.h"
#include "sx127x_netdev.h"
#include "sx127x_registers.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define SX127X_LORA_MSG_QUEUE   (16U)
#define SX127X_STACKSIZE        (THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR            (0x3456)

static char stack[SX127X_STACKSIZE];
//static char sx1272_continuous_stack[THREAD_STACKSIZE_DEFAULT];
static kernel_pid_t _recv_pid;
//static kernel_pid_t _sx127x_cont_pid;

static char message[32];
static uint8_t fsk_raw_bytes[290];
static uint16_t fsk_raw_len;
static sx127x_t sx127x;

static uint32_t ps_detector = 0;
static uint8_t data = 0;
static uint8_t bit_cnt = 0;
static uint8_t l_field_raw[3];
static uint8_t l_field_raw_cnt = 0;
static uint8_t raw_data_cnt = 0;
static uint8_t wmbus_l_field[2];

static const uint32_t ACCESS_CODE = 0b0101010101010000111101U;
static const uint32_t ACCESS_CODE_BITMASK = 0x3FFFFFU;

                             /* 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 */
uint8_t fsk_reg_settings [] = { 0x0D, 0x03, 0xD4, 0x02, 0x6F, 0xD9, 0x3C, 0xCD,
                             /* 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10 */
                                0x0F, 0x19, 0x2B, 0x20, 0x0E, 0x02, 0x0A, 0xFF,
                             /* 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18 */
                                0xEC, 0x02, 0x0B, 0x28, 0x0C, 0x12, 0x47, 0x32,
                             /* 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20 */
                                0x3E, 0x00, 0x00, 0x00, 0xFF, 0x65, 0x8A, 0x00,
                             /* 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28 */
                                0x00, 0x00, 0x00, 0x0F, 0x00, 0x18, 0x91, 0x54,
                             /* 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30 */
                                0x3D, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x10,
                             /* 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 */
                                0x40, 0x1B, 0x00, 0x00, 0x8F, 0x00, 0x00, 0x00,
                                0xF5, 0x20, 0x02, 0x00, 0x02, 0xDB, 0x26, 0x01,
                                0xF0, 0x22, 0x13, 0x0E, 0x5B, 0xDB, 0x24, 0x00,
                                0x01, 0x3A, 0x2E, 0x00, 0x03, 0x00, 0x00, 0x00,
                                0x00, 0x04, 0x23, 0x00, 0x00, 0x00, 0x00, 0x09,
                                0x05, 0x84, 0x0B, 0xD0, 0x0B, 0xD0, 0x32, 0x2B,
                                0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
                                0xE0, 0x00, 0x0C, 0x00, 0x13, 0x26, 0x01, 0x0B,
                                0x5C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


static uint16_t byte_size(uint16_t packet_size)
{
    uint16_t tmode_var;

    // T-mode
    // Data is 3 out of 6 coded
    tmode_var = (3 * packet_size) / 2;

    if (packet_size % 2) {
        return (tmode_var + 1);
    }
    else {
        return (tmode_var);
    }
}

static uint16_t packet_size (uint8_t l_field)
{
  uint16_t nr_bytes;
  uint8_t  nr_blocks;

  // The 2 first blocks contains 25 bytes when excluding CRC and the L-field
  // The other blocks contains 16 bytes when excluding the CRC-fields
  // Less than 26 (15 + 10)
  if ( l_field < 26 )
    nr_blocks = 2;
  else
    nr_blocks = (((l_field - 26) / 16) + 3);

  // Add all extra fields, excluding the CRC fields
  nr_bytes = l_field + 1;

  // Add the CRC fields, each block is contains 2 CRC bytes
  nr_bytes += (2 * nr_blocks);

  return (nr_bytes);
}

static void _print_bytes(uint8_t *bytes, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        printf("%02X", bytes[i]);
    }

    puts("");
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    sx127x_t *dev = (sx127x_t*) netdev;
    uint8_t enc_l_field[3];
    uint8_t l_field[2];
    uint16_t size = 0;
    uint16_t discarded = 0;

    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
            /* Process the payload */
            if (len == 0 && buf == NULL) {
                DEBUG("Read length first...\n");
//                if ((dev->settings.fsk.flags & SX127X_FIX_LEN_FLAG) == true) {
                    /* Read only the length value */
                while (!(sx127x_reg_read(dev, SX127X_REG_IRQFLAGS2) & SX127X_RF_IRQFLAGS2_FIFOEMPTY)) {
                    sx127x_read_fifo(dev, &fsk_raw_bytes[discarded], 1);
                    discarded++;
                }
                    DEBUG("Raw FSK: ");
                    _print_bytes(fsk_raw_bytes, discarded);
                    DEBUG("%d bytes\n", discarded);
                    memcpy(enc_l_field, fsk_raw_bytes, 3);
                    if (three_out_of_six_decode(enc_l_field, l_field) == THREE_OUT_OF_SIX_OK) {
                        dev->settings.fsk.pkt_handler.size = l_field[0];
                        fsk_raw_len = byte_size(packet_size(l_field[0]));
                        size = fsk_raw_len;
                        DEBUG("[sx127x] netdev->_recv: received %d bytes\n", dev->settings.fsk.pkt_handler.size);
                        DEBUG("[sx127x] netdev->_recv: decoded L-Field %d\n", fsk_raw_len);
                        break;
                    }
                    else {
                        DEBUG("Decoding error! Cleaning FIFO\n");
                        discarded = 0;
                        while (!(sx127x_reg_read(dev, SX127X_REG_IRQFLAGS2) & SX127X_RF_IRQFLAGS2_FIFOEMPTY)) {
                            sx127x_read_fifo(dev, &fsk_raw_bytes[discarded], 1);
                            discarded++;
                        }
                        DEBUG("FIFO emptied: %d bytes\n", discarded);
                        _print_bytes(fsk_raw_bytes, discarded);
                        memset(fsk_raw_bytes, 0, sizeof(fsk_raw_bytes));
                    }
//                }
//                else {
//                    dev->settings.fsk.pkt_handler.size = sx127x_reg_read(dev, SX127X_REG_PAYLOADLENGTH);
//                    break;
//                }
            }
            else {
                if ((dev->settings.fsk.flags & SX127X_RX_FSK_CONTINUOUS_FLAG) == false) {
                    //DEBUG("[sx127x] netdev: Packet mode -> going IDLE\n");
                    //sx127x_set_standby(dev);
                }
                else {
                    /* Restart RX */
                    sx127x_reg_write(dev, SX127X_REG_RXCONFIG,
                                     sx127x_reg_read(dev, SX127X_REG_RXCONFIG) |
                                     SX127X_RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                }

                while (!(sx127x_reg_read(dev, SX127X_REG_IRQFLAGS2) & SX127X_RF_IRQFLAGS2_FIFOEMPTY)) {
                    sx127x_read_fifo(dev, buf + dev->settings.fsk.pkt_handler.nb_bytes,
                            dev->settings.fsk.pkt_handler.size -
                            dev->settings.fsk.pkt_handler.nb_bytes);

                    dev->settings.fsk.pkt_handler.nb_bytes += (dev->settings.fsk.pkt_handler.size -
                            dev->settings.fsk.pkt_handler.nb_bytes);
                }
                sx127x_read_fifo(dev, buf, len);
                size = len;
            }

            netdev_sx127x_packet_info_t *packet_info = info;
            packet_info->rssi = dev->settings.fsk.pkt_handler.rssi_value;

            /* Reset status */
            dev->settings.fsk.flags &= ~SX127X_PREAMBLE_DETECTED_FLAG;
            dev->settings.fsk.flags &= ~SX127X_SYNC_WORD_DETECTED_FLAG;
            dev->settings.fsk.pkt_handler.nb_bytes = 0;
            dev->settings.fsk.pkt_handler.size = 0;
            uint8_t state = NETOPT_STATE_RX;
            netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));

        break;
        case SX127X_MODEM_LORA:
            break;
        default:
            break;
    }

    return size;
}

static void _dio1_data_recv(void *arg)
{
    /* Get interrupt context */
    sx127x_t *dev = (sx127x_t *) arg;

    uint8_t bit = gpio_read(dev->params.dio2_pin);

    if (!data) {
        ps_detector = (ps_detector << 1) | bit;
    }
    else {
        if (l_field_raw_cnt < 3) {
            if (bit_cnt < 8) {
                l_field_raw[l_field_raw_cnt] = (l_field_raw[l_field_raw_cnt] << 1) | bit;
                bit_cnt++;
            }
            else {
                l_field_raw_cnt++;
                bit_cnt = 0;
            }

            if (l_field_raw_cnt == 3) {
                if (three_out_of_six_decode(l_field_raw, wmbus_l_field) != THREE_OUT_OF_SIX_ERROR) {
                    printf("L-Field: %d\n", wmbus_l_field[0]);
                    fsk_raw_len = byte_size(packet_size(wmbus_l_field[0]));
                    printf("wM-Bus frame length: %d\n", fsk_raw_len);
                }
                else {
                    puts("Error decoding L-Field!");
                    _print_bytes(l_field_raw, sizeof(l_field_raw));
                    raw_data_cnt = 0;
                    bit_cnt = 0;
                    data = 0;
                    memset(l_field_raw, 0, sizeof(l_field_raw));
                    l_field_raw_cnt = 0;
                    memset(wmbus_l_field, 0, sizeof(wmbus_l_field));
                }
            }
        }
        else {
            if (fsk_raw_len) {
                if (bit_cnt < 8) {
                    fsk_raw_bytes[raw_data_cnt] = (fsk_raw_bytes[raw_data_cnt] << 1) | bit;
                    bit_cnt++;
                }
                else {
                    raw_data_cnt++;
                    fsk_raw_len--;
                    bit_cnt = 0;
                }
            }
            else {
                printf("received raw data: %d bytes\n", raw_data_cnt);
                _print_bytes(fsk_raw_bytes, raw_data_cnt);
                raw_data_cnt = 0;
                bit_cnt = 0;
                data = 0;
                memset(l_field_raw, 0, sizeof(l_field_raw));
                l_field_raw_cnt = 0;
                memset(wmbus_l_field, 0, sizeof(wmbus_l_field));
            }
        }
    }

    if (((ps_detector & ACCESS_CODE_BITMASK) == ACCESS_CODE) && data == 0) {
        ps_detector = 0;
        data = 1;
        printf("[main] preamble + sync word detected!\n");
    }
}

int fsk_init_cmd(int argc, char **argv)
{
    //netdev_t *netdev = (netdev_t*) &sx127x;
    int res;

    if (argc < 1) {
        puts("Usage: init_fsk <mode (packet, continuous)>");
        return -1;
    }

    if (strstr(argv[1], "continuous") != NULL) {
        puts("Initialising FSK modem in continuous mode");
        sx127x_init_fsk_settings(&sx127x, SX127X_FSK_CONTINUOUS_MODE);
    }
    else if (strstr(argv[1], "packet") != NULL) {
        puts("Initialising FSK modem in packet mode");
        sx127x_init_fsk_settings(&sx127x, SX127X_FSK_PACKET_MODE);
    }
    else {
        puts("Usage: init_fsk <mode (packet, continuous)>");
        return -1;
    }


    if (sx127x.settings.fsk.flags & SX127X_RX_FSK_CONTINUOUS_FLAG) {
        printf("[main] setting continuous mode...\n");
        res = gpio_init_int(sx127x.params.dio1_pin, GPIO_IN, GPIO_RISING, _dio1_data_recv, &sx127x);
        if (res < 0) {
            printf("[main] error: failed to initialize DIO1 pin\n");
            return 0;
        }

        res = gpio_init(sx127x.params.dio2_pin, GPIO_IN);
        if (res < 0) {
            printf("[main] error: failed to initialize DIO2 pin\n");
            return 0;
        }

//        _sx127x_cont_pid = thread_create(sx1272_continuous_stack,
//                                         sizeof(sx1272_continuous_stack),
//                                         THREAD_PRIORITY_MAIN - 1,
//                                         THREAD_CREATE_STACKTEST,
//                                         sx127x_continuous_thread, NULL,
//                                         "sx1272_cont_thread");

//        if (_sx127x_cont_pid <= KERNEL_PID_UNDEF) {
//            puts("Creation of sx127x continuous thread failed");
//            return -1;
//        }

    }


    /* Switch to RX state */
//    uint8_t state = NETOPT_STATE_RX;
//    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));
//
//    printf("Listen mode set\n");

    return 0;
}

int lora_setup_cmd(int argc, char **argv) {

    if (argc < 4) {
        puts("usage: setup "
             "<bandwidth (125, 250, 500)> "
             "<spreading factor (7..12)> "
             "<code rate (5..8)>");
        return -1;
    }

    /* Check bandwidth value */
    int bw = atoi(argv[1]);
    uint8_t lora_bw;
    switch (bw) {
        case 125:
            puts("setup: setting 125KHz bandwidth");
            lora_bw = LORA_BW_125_KHZ;
            break;

        case 250:
            puts("setup: setting 250KHz bandwidth");
            lora_bw = LORA_BW_250_KHZ;
            break;

        case 500:
            puts("setup: setting 500KHz bandwidth");
            lora_bw = LORA_BW_500_KHZ;
            break;

        default:
            puts("[Error] setup: invalid bandwidth value given, "
                 "only 125, 250 or 500 allowed.");
            return -1;
    }

    /* Check spreading factor value */
    uint8_t lora_sf = atoi(argv[2]);
    if (lora_sf < 7 || lora_sf > 12) {
        puts("[Error] setup: invalid spreading factor value given");
        return -1;
    }

    /* Check coding rate value */
    int cr = atoi(argv[3]);;
    if (cr < 5 || cr > 8) {
        puts("[Error ]setup: invalid coding rate value given");
        return -1;
    }
    uint8_t lora_cr = (uint8_t)(cr - 4);

    /* Configure radio device */
    netdev_t *netdev = (netdev_t*) &sx127x;
    netdev->driver->set(netdev, NETOPT_BANDWIDTH,
                        &lora_bw, sizeof(lora_bw));
    netdev->driver->set(netdev, NETOPT_SPREADING_FACTOR,
                        &lora_sf, sizeof(lora_sf));
    netdev->driver->set(netdev, NETOPT_CODING_RATE,
                        &lora_cr, sizeof(lora_cr));

    puts("[Info] setup: configuration set with success");

    return 0;
}

int random_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    netdev_t *netdev = (netdev_t*) &sx127x;
    printf("random: number from sx127x: %u\n",
           (unsigned int) sx127x_random((sx127x_t*) netdev));

    /* reinit the transceiver to default values */
    if (sx127x.settings.modem == SX127X_MODEM_FSK) {
        sx127x_init_fsk_settings((sx127x_t*) netdev, SX127X_FSK_PACKET_MODE);
    }
    else {
        sx127x_init_lora_settings((sx127x_t*) netdev);
    }

    return 0;
}

int register_cmd(int argc, char **argv)
{
    if (argc < 2) {
        puts("usage: register <get | set>");
        return -1;
    }

    if (strstr(argv[1], "get") != NULL) {
        if (argc < 3) {
            puts("usage: register get <all | allinline | regnum>");
            return -1;
        }

        if (strcmp(argv[2], "all") == 0) {
            puts("- listing all registers -");
            uint8_t reg = 0, data = 0;
            /* Listing registers map */
            puts("Reg   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
            for (unsigned i = 0; i <= 7; i++) {
                printf("0x%02X ", i << 4);

                for (unsigned j = 0; j <= 15; j++, reg++) {
                    data = sx127x_reg_read(&sx127x, reg);
                    printf("%02X ", data);
                }
                puts("");
            }
            puts("-done-");
            return 0;
        }
        else if (strcmp(argv[2], "allinline") == 0) {
            puts("- listing all registers in one line -");
            /* Listing registers map */
            for (uint16_t reg = 0; reg < 256; reg++) {
                printf("%02X ", sx127x_reg_read(&sx127x, (uint8_t) reg));
            }
            puts("- done -");
            return 0;
        }
        else {
            long int num = 0;
            /* Register number in hex */
            if (strstr(argv[2], "0x") != NULL) {
                num = strtol(argv[2], NULL, 16);
            }
            else {
                num = atoi(argv[2]);
            }

            if (num >= 0 && num <= 255) {
                printf("[regs] 0x%02X = 0x%02X\n",
                       (uint8_t) num,
                       sx127x_reg_read(&sx127x, (uint8_t) num));
            }
            else {
                puts("regs: invalid register number specified");
                return -1;
            }
        }
    }
    else if (strstr(argv[1], "set") != NULL) {
        if (argc < 4) {
            puts("usage: register set <regnum> <value>");
            return -1;
        }

        long num, val;

        /* Register number in hex */
        if (strstr(argv[2], "0x") != NULL) {
            num = strtol(argv[2], NULL, 16);
        }
        else {
            num = atoi(argv[2]);
        }

        /* Register value in hex */
        if (strstr(argv[3], "0x") != NULL) {
            val = strtol(argv[3], NULL, 16);
        }
        else {
            val = atoi(argv[3]);
        }

        sx127x_reg_write(&sx127x, (uint8_t) num, (uint8_t) val);
    }
    else {
        puts("usage: register get <all | allinline | regnum>");
        return -1;
    }

    return 0;
}

int register_check_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("Listing registers...");

    for (uint8_t reg = 1; reg < 127; reg++) {
        if (sx127x_reg_read(&sx127x, reg) != fsk_reg_settings[reg - 1]){
            printf("Mismatch register: 0x%02X -> 0x%02X ; 0x%02X\n", reg,
                    sx127x_reg_read(&sx127x, reg), fsk_reg_settings[reg - 1]);
        }
    }
    puts("");

    return 0;
}

int send_cmd(int argc, char **argv)
{
    if (argc <= 1) {
        puts("usage: send <payload>");
        return -1;
    }

    printf("sending \"%s\" payload (%u bytes)\n",
           argv[1], (unsigned)strlen(argv[1]) + 1);

    iolist_t iolist = {
        .iol_base = argv[1],
        .iol_len = (strlen(argv[1]) + 1)
    };

    netdev_t *netdev = (netdev_t*) &sx127x;
    if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
        puts("Cannot send: radio is still transmitting");
    }

    return 0;
}

int listen_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    netdev_t *netdev = (netdev_t*) &sx127x;

    if (sx127x.settings.modem == SX127X_MODEM_LORA) {
        /* Switch to continuous listen mode */
        const netopt_enable_t single = false;
        netdev->driver->set(netdev, NETOPT_SINGLE_RECEIVE, &single, sizeof(single));
        const uint32_t timeout = 0;
        netdev->driver->set(netdev, NETOPT_RX_TIMEOUT, &timeout, sizeof(timeout));

        /* Switch to RX state */
        uint8_t state = NETOPT_STATE_RX;
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));
    }
    else if (sx127x.settings.modem == SX127X_MODEM_FSK) {
        /* Switch to RX state */
        uint8_t state = NETOPT_STATE_RX;
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));
//        sx127x_set_state(&sx127x, SX127X_RF_RX_RUNNING);
    }

    printf("Listen mode set\n");

    return 0;
}

int channel_cmd(int argc, char **argv)
{
    if(argc < 2) {
        puts("usage: channel <get|set>");
        return -1;
    }

    netdev_t *netdev = (netdev_t*) &sx127x;
    uint32_t chan;
    if (strstr(argv[1], "get") != NULL) {
        netdev->driver->get(netdev, NETOPT_CHANNEL_FREQUENCY, &chan, sizeof(chan));
        printf("Channel: %i\n", (int) chan);
        return 0;
    }

    if (strstr(argv[1], "set") != NULL) {
        if(argc < 3) {
            puts("usage: channel set <channel>");
            return -1;
        }
        chan = atoi(argv[2]);
        netdev->driver->set(netdev, NETOPT_CHANNEL_FREQUENCY, &chan, sizeof(chan));
        printf("New channel set\n");
    }
    else {
        puts("usage: channel <get|set>");
        return -1;
    }

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "setup",    "Initialize LoRa modulation settings",     lora_setup_cmd},
    { "random",   "Get random number from sx127x",           random_cmd },
    { "channel",  "Get/Set channel frequency (in Hz)",       channel_cmd },
    { "register", "Get/Set value(s) of registers of sx127x", register_cmd },
    { "send",     "Send raw payload string",                 send_cmd },
    { "listen",   "Start raw payload listener",              listen_cmd },
    { "init_fsk", "Initialise the radio in FSK mode",        fsk_init_cmd },
    { "register_check", "Check register settings",           register_check_cmd},
    { NULL, NULL, NULL }
};

static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;

        if (msg_send(&msg, _recv_pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
    }
    else {
        size_t len;
        netdev_sx127x_packet_info_t packet_info;
        switch (event) {
            case NETDEV_EVENT_RX_COMPLETE:
                if (sx127x.settings.modem == SX127X_MODEM_LORA) {
                    len = dev->driver->recv(dev, NULL, 0, 0);
                    dev->driver->recv(dev, message, len, &packet_info);
                    printf("{Payload: \"%s\" (%d bytes), RSSI: %i, SNR: %i, TOA: %" PRIu32 "}\n",
                            message, (int)len,
                            packet_info.rssi, (int)packet_info.snr,
                            sx127x_get_time_on_air((const sx127x_t*)dev, len));
                }
                else {
                    DEBUG("Receiving data...\n");
                    len = _recv(dev, NULL, 0, &packet_info);
                    /*_recv(dev, fsk_raw_bytes, len, &packet_info);
                    printf("{Payload: ");

                    for (unsigned i = 0; i < len; i++) {
                        printf("%02X", fsk_raw_bytes[i]);
                    }*/
                    printf(" (%d bytes), RSSI: %i\n", (int)len, packet_info.rssi);
                }
                break;

            case NETDEV_EVENT_TX_COMPLETE:
                sx127x_set_sleep(&sx127x);
                puts("Transmission completed");
                break;

            case NETDEV_EVENT_CAD_DONE:
                break;

            case NETDEV_EVENT_TX_TIMEOUT:
                sx127x_set_sleep(&sx127x);
                break;

            case NETDEV_EVENT_CRC_ERROR:
                puts("CRC error!\n");
                sx127x_set_state(&sx127x, SX127X_RF_IDLE);
                break;

            default:
                printf("Unexpected netdev event received: %d\n", event);
                break;
        }
    }
}

void *_recv_thread(void *arg)
{
    (void)arg;

    static msg_t _msg_q[SX127X_LORA_MSG_QUEUE];
    msg_init_queue(_msg_q, SX127X_LORA_MSG_QUEUE);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        }
        else {
            puts("Unexpected msg type");
        }
    }
}

int main(void)
{
    sx127x.params = sx127x_params[0];
    netdev_t *netdev = (netdev_t*) &sx127x;
    netdev->driver = &sx127x_driver;

    if (netdev->driver->init(netdev) < 0) {
        puts("Failed to initialize SX127x device, exiting");
        return 1;
    }

    netdev->event_callback = _event_cb;

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
