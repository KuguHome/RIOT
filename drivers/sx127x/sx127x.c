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
 * @ingroup     drivers_sx127x
 * @{
 * @file
 * @brief       Basic functionality of sx127x driver
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "xtimer.h"
#include "thread.h"

#include "periph/gpio.h"
#include "periph/spi.h"

#include "net/lora.h"
#include "net/fsk.h"

#include "sx127x.h"
#include "sx127x_internal.h"
#include "sx127x_registers.h"
#include "sx127x_netdev.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/* Internal functions */
static int _init_spi(sx127x_t *dev);
static int _init_gpios(sx127x_t *dev);
static void _init_timers(sx127x_t *dev);
static void _on_tx_timeout(void *arg);
static void _on_rx_timeout(void *arg);

/* SX127X DIO interrupt handlers initialization */
#ifndef SX127X_USE_DIO_MULTI
static void sx127x_on_dio0_isr(void *arg);
static void sx127x_on_dio1_isr(void *arg);
static void sx127x_on_dio2_isr(void *arg);
static void sx127x_on_dio3_isr(void *arg);
static void sx127x_on_dio4_isr(void *arg);
#else
static void sx127x_on_dio_multi_isr(void *arg);
#endif

void sx127x_setup(sx127x_t *dev, const sx127x_params_t *params)
{
    netdev_t *netdev = (netdev_t*) dev;
    netdev->driver = &sx127x_driver;
    dev->params = *params;
}

int sx127x_reset(const sx127x_t *dev)
{
    /*
     * This reset scheme complies with 7.2 chapter of the SX1272/1276 datasheet
     * See http://www.semtech.com/images/datasheet/sx1276.pdf for SX1276
     * See http://www.semtech.com/images/datasheet/sx1272.pdf for SX1272
     *
     * 1. Set NReset pin to LOW for at least 100 us
     * 2. Set NReset in Hi-Z state
     * 3. Wait at least 5 milliseconds
     */

    /* Check if the reset pin is defined */
    if (dev->params.reset_pin == GPIO_UNDEF) {
        DEBUG("[sx127x] error: No reset pin defined.\n");
        return -SX127X_ERR_GPIOS;
    }

    gpio_init(dev->params.reset_pin, GPIO_OUT);

    /* Set reset pin to 0 */
    gpio_clear(dev->params.reset_pin);

    /* Wait 1 ms */
    xtimer_usleep(1000);

    /* Put reset pin in High-Z */
    gpio_init(dev->params.reset_pin, GPIO_IN);

    /* Wait 10 ms */
    xtimer_usleep(1000 * 10);

    return 0;
}

int sx127x_init(sx127x_t *dev)
{
    /* Do internal initialization routines */
    if (_init_spi(dev) < 0) {
        DEBUG("[sx127x] error: failed to initialize SPI\n");
        return -SX127X_ERR_SPI;
    }

    /* Check presence of SX127X */
    if (sx127x_check_version(dev) < 0) {
        DEBUG("[sx127x] error: no valid device found\n");
        return -SX127X_ERR_NODEV;
    }

    _init_timers(dev);
    xtimer_usleep(1000); /* wait 1 millisecond */

    sx127x_reset(dev);

#if defined(MODULE_SX1276)
    if(dev->settings.modem == SX127X_MODEM_LORA) {
        sx1276_rx_chain_calibration(dev, SX127X_HF_CHANNEL_LORA);
    }
    else {
        sx1276_rx_chain_calibration(dev, FSK_HF_CHANNEL_DEFAULT);
    }
#endif
    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_SLEEP);

    if (_init_gpios(dev) < 0) {
        DEBUG("[sx127x] error: failed to initialize GPIOs\n");
        return -SX127X_ERR_GPIOS;
    }

    return SX127X_INIT_OK;
}

void sx127x_init_lora_settings(sx127x_t *dev)
{
    DEBUG("[sx127x] initializing LoRa settings\n");
    sx127x_set_channel(dev, SX127X_CHANNEL_DEFAULT);
    sx127x_set_modem(dev, SX127X_MODEM_DEFAULT);
    sx127x_set_tx_power(dev, SX127X_RADIO_TX_POWER);
    sx127x_set_bandwidth(dev, LORA_BW_DEFAULT);
    sx127x_set_spreading_factor(dev, LORA_SF_DEFAULT);
    sx127x_set_coding_rate(dev, LORA_CR_DEFAULT);
    sx127x_set_crc(dev, LORA_PAYLOAD_CRC_ON_DEFAULT);
    sx127x_set_freq_hop(dev, LORA_FREQUENCY_HOPPING_DEFAULT);
    sx127x_set_hop_period(dev, LORA_FREQUENCY_HOPPING_PERIOD_DEFAULT);
    sx127x_set_fixed_header_len_mode(dev, LORA_FIXED_HEADER_LEN_MODE_DEFAULT);
    sx127x_set_iq_invert(dev, LORA_IQ_INVERTED_DEFAULT);
    sx127x_set_payload_length(dev, LORA_PAYLOAD_LENGTH_DEFAULT);
    sx127x_set_preamble_length(dev, LORA_PREAMBLE_LENGTH_DEFAULT);
    sx127x_set_symbol_timeout(dev, LORA_SYMBOL_TIMEOUT_DEFAULT);
    sx127x_set_rx_single(dev, SX127X_RX_SINGLE);
    sx127x_set_tx_timeout(dev, SX127X_TX_TIMEOUT_DEFAULT);
}

void sx127x_init_fsk_settings(sx127x_t *dev, uint8_t mode)
{
    DEBUG("[sx127x] initializing FSK settings\n");
    sx127x_reset(dev);
    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_SLEEP);
/**
 * Omit for now API settings, go straight and set the registers
 * manually
 */
    sx127x_set_modem(dev, SX127X_MODEM_FSK);
#if defined(MODULE_SX1276)
    sx127x_set_tx_power(dev, 20);
#else /* MODULE_SX1272 */
    sx127x_set_tx_power(dev, 14);
#endif
    sx127x_reg_write(dev, SX127X_REG_PARAMP, 0x19);
    //sx127x_set_lna(dev, SX127X_RF_LNA_GAIN_G3);
    sx127x_reg_write(dev, SX127X_REG_LNA, 0x20);
    sx127x_fsk_set_afc(dev, false);
    sx127x_reg_write(dev, SX127X_REG_FDEVMSB, 0x02);
    sx127x_reg_write(dev, SX127X_REG_FDEVLSB, 0x6F);
    sx127x_set_rxbw(dev, 0, FSK_BANDWIDTH_DEFAULT);
    //sx127x_set_afcbw(dev, FSK_AFC_BANDWIDTH_DEFAULT);
    sx127x_reg_write(dev, SX127X_REG_AFCBW, 0x0B);
    sx127x_reg_write(dev, SX127X_REG_FEIMSB, (sx127x_reg_read(dev, SX127X_REG_FEIMSB) | 0xFF));
    sx127x_reg_write(dev, SX127X_REG_FEILSB, (sx127x_reg_read(dev, SX127X_REG_FEIMSB) | 0x65));
    sx127x_reg_write(dev, SX127X_REG_OSC, 0x0F);
    sx127x_set_packetconfig1(dev, SX127X_RF_PACKETCONFIG1_PACKETFORMAT_FIXED,
                             SX127X_RF_PACKETCONFIG1_DCFREE_OFF,
                             SX127X_RF_PACKETCONFIG1_CRC_ON,
                             SX127X_RF_PACKETCONFIG1_CRCAUTOCLEAR_ON,
                             SX127X_RF_PACKETCONFIG1_ADDRSFILTERING_OFF,
                             SX127X_RF_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);
    switch (mode)
    {
    case SX127X_FSK_PACKET_MODE:
//        sx127x_set_dio_mapping1(dev, SX127X_RF_DIOMAPPING1_DIO0_00, /* PayloadReady */
//                                SX127X_RF_DIOMAPPING1_DIO1_00,      /* FifoLevel */
//                                SX127X_RF_DIOMAPPING1_DIO2_11,      /* SyncAddress */
//                                SX127X_RF_DIOMAPPING1_DIO3_00);     /* FifoEmpty */
//        sx127x_set_dio_mapping2(dev, SX127X_RF_DIOMAPPING2_DIO4_11, /* PreambleDetected */
//                                SX127X_RF_DIOMAPPING2_DIO5_11);     /* ModeReady */
//
//        sx127x_set_state(dev, SX127X_RF_IDLE);
//        for (size_t i = 0; i < sizeof(fsk_reg_settings); i++) {
//            DEBUG("Setting reg 0x%02X -> 0x%02X\n", i + 1, fsk_reg_settings[i]);
//            switch (i) {
//            case 0x11:
//            case 0x3C:
//            case 0x3E:
//            case 0x3F:
//            case 0x42:
//                break;
//            default:
//                sx127x_reg_write(dev, i + 1, fsk_reg_settings[i]);
//            }
//        }
//        sx127x_set_state(dev, SX127X_RF_RX_RUNNING);
        sx127x_reg_write(dev, SX127X_REG_PREAMBLELSB, 0x18);
        sx127x_set_syncconfig(dev, SX127X_RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON,
                              SX127X_RF_SYNCCONFIG_PREAMBLEPOLARITY_AA,
                              SX127X_RF_SYNCCONFIG_SYNC_ON,
                              SX127X_RF_SYNCCONFIG_FIFOFILL_SYNCADDR,
                              SX127X_RF_SYNCCONFIG_SYNCSIZE_2);
        sx127x_fsk_set_syncword(dev, FSK_SYNCWORD_SYNCVALUE1, 1);
        sx127x_fsk_set_syncword(dev, FSK_SYNCWORD_SYNCVALUE2, 2);
        sx127x_fsk_set_syncword(dev, 0x01, 3);
        sx127x_fsk_set_syncword(dev, 0x01, 4);
        sx127x_fsk_set_syncword(dev, 0x01, 5);
        sx127x_fsk_set_syncword(dev, 0x01, 6);
        sx127x_fsk_set_syncword(dev, 0x01, 7);
        sx127x_fsk_set_syncword(dev, 0x01, 8);
        sx127x_set_payload_length(dev, 0x1B);
        sx127x_set_fsk_mod_shaping(dev, SX127X_RF_OPMODE_MODULATIONSHAPING_01);
        sx127x_reg_write(dev, SX127X_REG_FIFOTHRESH,
                         ((sx127x_reg_read(dev, SX127X_REG_FIFOTHRESH) &
                         SX127X_RF_FIFOTHRESH_TXSTARTCONDITION_MASK &
                         SX127X_RF_FIFOTHRESH_FIFOTHRESHOLD_MASK) |
                         SX127X_RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY |
                         0x0F));
#if defined(MODULE_SX1272)
        sx127x_reg_write(dev, SX127X_REG_IMAGECAL, 0x02);
#endif
        sx127x_set_packetconfig2(dev, SX127X_RF_PACKETCONFIG2_WMBUS_CRC_DISABLE,
                                 SX127X_RF_PACKETCONFIG2_DATAMODE_PACKET,
                                 SX127X_RF_PACKETCONFIG2_IOHOME_OFF,
                                 SX127X_RF_PACKETCONFIG2_BEACON_OFF);
        sx127x_set_dio_mapping1(dev, SX127X_RF_DIOMAPPING1_DIO0_00, /* PayloadReady */
                                SX127X_RF_DIOMAPPING1_DIO1_00,      /* FifoLevel */
                                SX127X_RF_DIOMAPPING1_DIO2_00,      /* SyncAddress */
                                SX127X_RF_DIOMAPPING1_DIO3_01);     /* FifoEmpty */
        sx127x_set_dio_mapping2(dev, SX127X_RF_DIOMAPPING2_DIO4_11, /* PreambleDetected */
                                SX127X_RF_DIOMAPPING2_DIO5_11);     /* ModeReady */
        dev->settings.fsk.flags &= ~SX127X_PREAMBLE_DETECTED_FLAG;
        dev->settings.fsk.flags &= ~SX127X_SYNC_WORD_DETECTED_FLAG;
        sx127x_reg_write(dev, 0x70, 0x0B);
        break;

    case SX127X_FSK_CONTINUOUS_MODE:
        sx127x_set_syncconfig(dev, SX127X_RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_ON,
                              SX127X_RF_SYNCCONFIG_PREAMBLEPOLARITY_AA,
                              SX127X_RF_SYNCCONFIG_SYNC_ON,
                              SX127X_RF_SYNCCONFIG_FIFOFILL_SYNCADDR,
                              SX127X_RF_SYNCCONFIG_SYNCSIZE_2);
        sx127x_fsk_set_syncword(dev, FSK_SYNCWORD_SYNCVALUE1, 1);
        sx127x_fsk_set_syncword(dev, FSK_SYNCWORD_SYNCVALUE2, 2);
        sx127x_set_packetconfig2(dev, SX127X_RF_PACKETCONFIG2_WMBUS_CRC_DISABLE,
                                 SX127X_RF_PACKETCONFIG2_DATAMODE_CONTINUOUS,
                                 SX127X_RF_PACKETCONFIG2_IOHOME_OFF,
                                 SX127X_RF_PACKETCONFIG2_BEACON_OFF);
        sx127x_set_dio_mapping1(dev, SX127X_RF_DIOMAPPING1_DIO0_00, /* SyncAddress */
                                SX127X_RF_DIOMAPPING1_DIO1_00,      /* DClk -> GPIO interrupt must be enabled */
                                SX127X_RF_DIOMAPPING1_DIO2_11,      /* Data -> GPIO interrupt must be disabled */
                                SX127X_RF_DIOMAPPING1_DIO3_01);     /* RSSI/PreambleDetected */
        sx127x_set_dio_mapping2(dev, SX127X_RF_DIOMAPPING2_DIO4_11, /* ModeReady */
                                SX127X_RF_DIOMAPPING2_DIO5_11);     /* ModeReady */
        break;

    default:
        DEBUG("[sx127x] No mode selected, using default mode\n");
        break;

    }

    sx127x_fsk_set_preamble_detect(dev, SX127X_RF_PREAMBLEDETECT_DETECTOR_ON);
    sx127x_fsk_set_preamble_detector_size(dev, SX127X_RF_PREAMBLEDETECT_DETECTORSIZE_1);
    sx127x_fsk_set_preamble_detector_tol(dev, SX127X_RF_PREAMBLEDETECT_DETECTORTOL_10);
    sx127x_set_rx_trigger(dev, SX127X_RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);
    sx127x_set_map_preamble_detect(dev, SX127X_RF_DIOMAPPING2_MAP_RSSI);
    //sx127x_set_channel(dev, FSK_CHANNEL_DEFAULT);
    sx127x_reg_write(dev, SX127X_REG_FRFMSB, 0xD9);
    sx127x_reg_write(dev, SX127X_REG_FRFMID, 0x3C);
    sx127x_reg_write(dev, SX127X_REG_FRFLSB, 0xCD);
    sx127x_set_bitrate(dev, FSK_BITRATE_DEFAULT);
    //sx127x_set_freqdev(dev, FSK_FREQ_DEV_DEFAULT);
    sx127x_fsk_set_rssi_offset(dev, 0);
    sx127x_reg_write(dev, 0x48, 0x00);
    sx127x_reg_write(dev, 0x49, 0x01);
    sx127x_reg_write(dev, 0x55, 0x00);
    sx127x_reg_write(dev, 0x57, 0x00);
    sx127x_reg_write(dev, 0x64, 0x00);
    sx127x_reg_write(dev, 0x6C, 0x00);
    sx127x_reg_write(dev, 0x6D, 0x13);
    sx127x_reg_write(dev, 0x6E, 0x26);
    sx127x_reg_write(dev, 0x6F, 0x01);
//      sx127x_set_standby(dev);
#if defined(MODULE_SX1276)
    sx1276_rx_chain_calibration(dev, FSK_HF_CHANNEL_DEFAULT);
#endif
}

uint32_t sx127x_random(sx127x_t *dev)
{
    uint32_t rnd = 0;

    sx127x_set_modem(dev, SX127X_MODEM_LORA); /* Set LoRa modem ON */

    /* Disable LoRa modem interrupts */
    sx127x_reg_write(dev, SX127X_REG_LR_IRQFLAGSMASK, SX127X_RF_LORA_IRQFLAGS_RXTIMEOUT |
                     SX127X_RF_LORA_IRQFLAGS_RXDONE |
                     SX127X_RF_LORA_IRQFLAGS_PAYLOADCRCERROR |
                     SX127X_RF_LORA_IRQFLAGS_VALIDHEADER |
                     SX127X_RF_LORA_IRQFLAGS_TXDONE |
                     SX127X_RF_LORA_IRQFLAGS_CADDONE |
                     SX127X_RF_LORA_IRQFLAGS_FHSSCHANGEDCHANNEL |
                     SX127X_RF_LORA_IRQFLAGS_CADDETECTED);

    /* Set radio in continuous reception */
    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_RECEIVER);

    for (unsigned i = 0; i < 32; i++) {
        xtimer_usleep(1000); /* wait for the chaos */

        /* Non-filtered RSSI value reading. Only takes the LSB value */
        rnd |= ((uint32_t) sx127x_reg_read(dev, SX127X_REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    sx127x_set_sleep(dev);

    return rnd;
}

int sx127x_config_dio(sx127x_t *dev, gpio_t pin, gpio_mode_t mode,
                      gpio_flank_t flank, uint8_t dio, bool is_int)
{
    int res = -1;

    switch (dio) {
    case 0:
        if (is_int) {
            res = gpio_init_int(pin, mode, flank, sx127x_on_dio0_isr, dev);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO0 pin\n");
                return res;
            }
        }
        else {
            res = gpio_init(pin, mode);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO0 pin\n");
                return res;
            }
        }
        break;

    case 1:
        if (is_int) {
            res = gpio_init_int(pin, mode, flank, sx127x_on_dio1_isr, dev);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO1 pin\n");
                return res;
            }
        }
        else {
            res = gpio_init(pin, mode);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO1 pin\n");
                return res;
            }
        }
        break;

    case 2:
        if (is_int) {
            res = gpio_init_int(pin, mode, flank, sx127x_on_dio2_isr, dev);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO2 pin\n");
                return res;
            }
        }
        else {
            res = gpio_init(pin, mode);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO2 pin\n");
                return res;
            }
        }
        break;

    case 3:
        if (is_int) {
            res = gpio_init_int(pin, mode, flank, sx127x_on_dio3_isr, dev);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO3 pin\n");
                return res;
            }
        }
        else {
            res = gpio_init(pin, mode);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO3 pin\n");
                return res;
            }
        }
        break;


    case 4:
        if (is_int) {
            res = gpio_init_int(pin, mode, flank, sx127x_on_dio4_isr, dev);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO4 pin\n");
                return res;
            }
        }
        else {
            res = gpio_init(pin, mode);
            if (res < 0) {
                DEBUG("[sx127x] error: failed to initialize DIO4 pin\n");
                return res;
            }
        }
        break;
    }

    return res;
}

/**
 * IRQ handlers
 */
void sx127x_isr(netdev_t *dev)
{
    if (dev->event_callback) {
        dev->event_callback(dev, NETDEV_EVENT_ISR);
    }
}

static void sx127x_on_dio_isr(sx127x_t *dev, sx127x_flags_t flag)
{
    dev->irq |= flag;
    sx127x_isr((netdev_t *)dev);
}

#ifndef SX127X_USE_DIO_MULTI
static void sx127x_on_dio0_isr(void *arg)
{
    sx127x_on_dio_isr((sx127x_t*) arg, SX127X_IRQ_DIO0);
}

static void sx127x_on_dio1_isr(void *arg)
{
    sx127x_on_dio_isr((sx127x_t*) arg, SX127X_IRQ_DIO1);
}

static void sx127x_on_dio2_isr(void *arg)
{
    sx127x_on_dio_isr((sx127x_t*) arg, SX127X_IRQ_DIO2);
}

static void sx127x_on_dio3_isr(void *arg)
{
    sx127x_on_dio_isr((sx127x_t*) arg, SX127X_IRQ_DIO3);
}

static void sx127x_on_dio4_isr(void *arg)
{
    sx127x_on_dio_isr((sx127x_t*) arg, SX127X_IRQ_DIO4);
}
#else
static void sx127x_on_dio_multi_isr(void *arg)
{
    sx127x_on_dio_isr((sx127x_t*) arg, SX127X_IRQ_DIO_MULTI);
}
#endif

/* Internal event handlers */
static int _init_gpios(sx127x_t *dev)
{
    int res;

#ifndef SX127X_USE_DIO_MULTI
    /* Check if DIO0 pin is defined */
    if (dev->params.dio0_pin != GPIO_UNDEF) {
        res = sx127x_config_dio(dev, dev->params.dio0_pin, GPIO_IN, GPIO_RISING, 0, true);
    }
    else {
        DEBUG("[sx127x] error: no DIO0 pin defined\n");
        DEBUG("[sx127x] error: at least one interrupt should be defined\n");
        return SX127X_ERR_GPIOS;
    }

    /* Check if DIO1 pin is defined */
    if (dev->params.dio1_pin != GPIO_UNDEF) {
        res = sx127x_config_dio(dev, dev->params.dio1_pin, GPIO_IN, GPIO_RISING, 1, true);
        if (res < 0) {
            DEBUG("[sx127x] error: failed to initialize DIO1 pin\n");
            return res;
        }
    }

    /* check if DIO2 pin is defined */
    if (dev->params.dio2_pin != GPIO_UNDEF) {
        res = sx127x_config_dio(dev, dev->params.dio2_pin, GPIO_IN, GPIO_RISING, 2, true);
        if (res < 0) {
            DEBUG("[sx127x] error: failed to initialize DIO2 pin\n");
            return res;
        }
    }

    /* check if DIO3 pin is defined */
    if (dev->params.dio3_pin != GPIO_UNDEF) {
        res = sx127x_config_dio(dev, dev->params.dio3_pin, GPIO_IN, GPIO_RISING, 3, true);
        if (res < 0) {
            DEBUG("[sx127x] error: failed to initialize DIO3 pin\n");
            return res;
        }
    }

    /* check if DIO4 pin is defined */
    if (dev->params.dio4_pin != GPIO_UNDEF) {
        res = sx127x_config_dio(dev, dev->params.dio4_pin, GPIO_IN, GPIO_RISING, 4, true);
        if (res < 0) {
            DEBUG("[sx127x] error: failed to initialize DIO4 pin\n");
            return res;
        }
    }

#else
    if (dev->params.dio_multi_pin != GPIO_UNDEF) {
        DEBUG("[sx127x] info: Trying to initialize DIO MULTI pin\n");
        res = gpio_init_int(dev->params.dio_multi_pin, GPIO_IN, GPIO_RISING,
                                sx127x_on_dio_multi_isr, dev);
        if (res < 0) {
            DEBUG("[sx127x] error: failed to initialize DIO MULTI pin\n");
            return res;
        }

        DEBUG("[sx127x] info: DIO MULTI pin initialized successfully\n");
    }
    else {
        DEBUG("[sx127x] error: no DIO MULTI pin defined\n");
        DEBUG("[sx127x] error at least one interrupt should be defined\n");
        return SX127X_ERR_GPIOS;
    }
#endif

    return res;
}

static void _on_tx_timeout(void *arg)
{
    netdev_t *dev = (netdev_t *) arg;

    dev->event_callback(dev, NETDEV_EVENT_TX_TIMEOUT);
}

static void _on_rx_timeout(void *arg)
{
    netdev_t *dev = (netdev_t *) arg;

    dev->event_callback(dev, NETDEV_EVENT_RX_TIMEOUT);
}

static void _init_timers(sx127x_t *dev)
{
    dev->_internal.tx_timeout_timer.arg = dev;
    dev->_internal.tx_timeout_timer.callback = _on_tx_timeout;

    dev->_internal.rx_timeout_timer.arg = dev;
    dev->_internal.rx_timeout_timer.callback = _on_rx_timeout;
}

static int _init_spi(sx127x_t *dev)
{
    int res;

    /* Setup SPI for SX127X */
    res = spi_init_cs(dev->params.spi, dev->params.nss_pin);

    if (res != SPI_OK) {
        DEBUG("[sx127x] error: failed to initialize SPI_%i device (code %i)\n",
              dev->params.spi, res);
        return -1;
    }

    DEBUG("[sx127x] SPI_%i initialized with success\n", dev->params.spi);
    return 0;
}
