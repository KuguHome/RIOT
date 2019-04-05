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
 * @brief       Implementation of get and set functions for SX127X
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "net/lora.h"
#include "net/fsk.h"

#include "sx127x.h"
#include "sx127x_registers.h"
#include "sx127x_internal.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/**
 * @brief Computes the mantisse and exponent from the bandwitdh value
 *
 * @param [in]      bw_value Bandwidth value
 * @param [out]     mantisse Mantisse of the bandwidth value
 * @param [out]     exponent Exponent of the bandwidth value
 */
static void _compute_bw_mant_exp(sx127x_t *dev, uint32_t bw_value,
                                       uint8_t *mantisse,
                                       uint8_t *exponent)
{
    uint8_t tmp_ext = 0;
    uint8_t tmp_mant = 0;
    double tmp_bw = 0;
    double rx_bw_min = 10e6;

    for(tmp_ext = 0; tmp_ext < 8; tmp_ext++) {
        for(tmp_mant = 16; tmp_mant <= 24; tmp_mant += 4) {
            if(dev->settings.modem == SX127X_MODEM_FSK) {
                tmp_bw = (double)SX127X_XTAL_FREQ / (tmp_mant * (double)pow(2, tmp_ext + 2));
            }
            else {
                tmp_bw = (double)SX127X_XTAL_FREQ / (tmp_mant * (double)pow(2, tmp_ext + 3));
            }

            if(fabs(tmp_bw - bw_value) < rx_bw_min) {
                rx_bw_min = fabs(tmp_bw - bw_value);
                *mantisse = tmp_mant;
                *exponent = tmp_ext;
            }
        }
    }
}

static void _compute_bandwidth(sx127x_t *dev, uint32_t *bw, uint32_t bw_value)
{
    uint8_t mantisse = 0;
    uint8_t exponent = 0;

    _compute_bw_mant_exp(dev, bw_value, &mantisse, &exponent);

    switch(mantisse)
    {
        case 16:
            *bw |= (uint8_t)(0x00 | (exponent & 0x07));
            break;
        case 20:
            *bw |= (uint8_t)(0x08 | (exponent & 0x07));
            break;
        case 24:
            *bw |= (uint8_t)(0x10 | (exponent & 0x07));
            break;
        default:
            /* should not happen */
            break;
    }

}

uint8_t sx127x_get_state(const sx127x_t *dev)
{
    return dev->settings.state;
}

void sx127x_set_state(sx127x_t *dev, uint8_t state)
{
#if ENABLE_DEBUG
    switch (state) {
    case SX127X_RF_IDLE:
        DEBUG("[sx127x] Change state: IDLE\n");
        break;
    case SX127X_RF_RX_RUNNING:
        DEBUG("[sx127x] Change state: RX\n");
        break;
    case SX127X_RF_TX_RUNNING:
        DEBUG("[sx127x] Change state: TX\n");
        break;
    default:
        DEBUG("[sx127x] Change state: UNKNOWN\n");
        break;
    }
#endif

    dev->settings.state = state;
}

void sx127x_set_modem(sx127x_t *dev, uint8_t modem)
{
    DEBUG("[sx127x] set modem: %d\n", modem >> 5);

    if ((sx127x_reg_read(dev, SX127X_REG_OPMODE) & SX127X_RF_LORA_OPMODE_LONGRANGEMODE_ON) != 0) {
        dev->settings.modem = SX127X_MODEM_LORA;
    }
    else {
        dev->settings.modem = SX127X_MODEM_FSK;
    }

    /* Skip if unchanged to avoid resetting the transceiver below (may end up
     * in crashes) */
    if (dev->settings.modem == modem) {
        DEBUG("[sx127x] already using modem: %d\n", modem);
        return;
    }

    dev->settings.modem = modem;

    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
            sx127x_reg_write(dev, SX127X_REG_OPMODE,
                             (sx127x_reg_read(dev, SX127X_REG_OPMODE) &
                              SX127X_RF_OPMODE_MODULATIONTYPE_MASK &
                              SX127X_RF_LORA_OPMODE_LONGRANGEMODE_MASK) |
                             SX127X_RF_OPMODE_MODULATIONTYPE_FSK |
                             SX127X_RF_LORA_OPMODE_LONGRANGEMODE_OFF);
            break;
        case SX127X_MODEM_LORA:
            sx127x_reg_write(dev, SX127X_REG_OPMODE,
                             (sx127x_reg_read(dev, SX127X_REG_OPMODE) &
                              SX127X_RF_LORA_OPMODE_LONGRANGEMODE_MASK) |
                             SX127X_RF_LORA_OPMODE_LONGRANGEMODE_ON);

            sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1, 0x00);
            sx127x_reg_write(dev, SX127X_REG_DIOMAPPING2, 0x00);
            break;
        default:
            break;
    }
}

uint8_t sx127x_get_syncword(const sx127x_t *dev)
{
    return sx127x_reg_read(dev, SX127X_REG_LR_SYNCWORD);
}

void sx127x_set_syncword(sx127x_t *dev, uint8_t syncword)
{
    DEBUG("[sx127x] Set syncword: %02x\n", syncword);

    sx127x_reg_write(dev, SX127X_REG_LR_SYNCWORD, syncword);
}

void sx127x_fsk_set_syncword(sx127x_t *dev, uint8_t syncword, uint8_t value)
{
    DEBUG("[sx127x] Set SyncValue[%d]: 0x%02X\n", value, syncword);
    switch (value) {
    case 1:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE1, syncword);
        break;

    case 2:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE2, syncword);
        break;

    case 3:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE3, syncword);
        break;

    case 4:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE4, syncword);
        break;

    case 5:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE5, syncword);
        break;

    case 6:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE6, syncword);
        break;

    case 7:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE7, syncword);
        break;

    case 8:
        sx127x_reg_write(dev, SX127X_REG_SYNCVALUE8, syncword);
        break;
    }

}

uint32_t sx127x_get_channel(const sx127x_t *dev)
{
    return (((uint32_t)sx127x_reg_read(dev, SX127X_REG_FRFMSB) << 16) |
            (sx127x_reg_read(dev, SX127X_REG_FRFMID) << 8) |
            (sx127x_reg_read(dev, SX127X_REG_FRFLSB))) * LORA_FREQUENCY_RESOLUTION_DEFAULT;
}

void sx127x_set_channel(sx127x_t *dev, uint32_t channel)
{
    DEBUG("[sx127x] Set channel: %" PRIu32 "\n", channel);

    /* Save current operating mode */
    dev->settings.channel = channel;

    channel = (uint32_t)((double) channel / (double)LORA_FREQUENCY_RESOLUTION_DEFAULT);

    /* Write frequency settings into chip */
    sx127x_reg_write(dev, SX127X_REG_FRFMSB, (uint8_t)((channel >> 16) & 0xFF));
    sx127x_reg_write(dev, SX127X_REG_FRFMID, (uint8_t)((channel >> 8) & 0xFF));
    sx127x_reg_write(dev, SX127X_REG_FRFLSB, (uint8_t)(channel & 0xFF));
}

uint32_t sx127x_get_time_on_air(const sx127x_t *dev, uint8_t pkt_len)
{
    uint32_t air_time = 0;

    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
            /* todo */
            break;
        case SX127X_MODEM_LORA:
        {
            double bw = 0.0;

            /* Note: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported. */
            switch (dev->settings.bandwidth) {
                case LORA_BW_125_KHZ:
                    bw = 125e3;
                    break;
                case LORA_BW_250_KHZ:
                    bw = 250e3;
                    break;
                case LORA_BW_500_KHZ:
                    bw = 500e3;
                    break;
                default:
                    DEBUG("Invalid bandwith: %ld\n", dev->settings.bandwidth);
                    break;
            }

            /* Symbol rate : time for one symbol [secs] */
            double rs = bw / (1 << dev->settings.lora.datarate);
            double ts = 1 / rs;

            /* time of preamble */
            double t_preamble = (dev->settings.preamble_len + 4.25) * ts;

            /* Symbol length of payload and time */
            double tmp =
                ceil(
                    (8 * pkt_len - 4 * dev->settings.lora.datarate + 28
                     + 16 * (dev->settings.lora.flags & SX127X_ENABLE_CRC_FLAG)
                     - (!(dev->settings.lora.flags & SX127X_ENABLE_FIXED_HEADER_LENGTH_FLAG) ? 20 : 0))
                    / (double) (4 * dev->settings.lora.datarate
                                - (((dev->settings.lora.flags & SX127X_LOW_DATARATE_OPTIMIZE_FLAG)
                                    > 0) ? 2 : 0)))
                * (dev->settings.lora.coderate + 4);
            double n_payload = 8 + ((tmp > 0) ? tmp : 0);
            double t_payload = n_payload * ts;

            /* Time on air */
            double t_on_air = t_preamble + t_payload;

            /* return milli seconds */
            air_time = floor(t_on_air * 1e3 + 0.999);
        }
        break;
    }

    return air_time;
}

void sx127x_set_sleep(sx127x_t *dev)
{
    DEBUG("[sx127x] Set sleep\n");

    /* Disable running timers */
    xtimer_remove(&dev->_internal.tx_timeout_timer);
    xtimer_remove(&dev->_internal.rx_timeout_timer);

    /* Put chip into sleep */
    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_SLEEP);
    sx127x_set_state(dev,  SX127X_RF_IDLE);
}

void sx127x_set_standby(sx127x_t *dev)
{
    DEBUG("[sx127x] Set standby\n");

    /* Disable running timers */
    xtimer_remove(&dev->_internal.tx_timeout_timer);
    xtimer_remove(&dev->_internal.rx_timeout_timer);

    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_STANDBY);
    sx127x_set_state(dev,  SX127X_RF_IDLE);
}

void sx127x_set_rx_trigger(sx127x_t *dev, uint8_t rx_trigger)
{
    DEBUG("[sx127x] Set RxTrigger 0x%02X\n", rx_trigger);

    sx127x_reg_write(dev, SX127X_REG_RXCONFIG, ((sx127x_reg_read(dev, SX127X_REG_RXCONFIG) &
                     SX127X_RF_RXCONFIG_RXTRIGER_MASK) |
                     rx_trigger));
}

void sx127x_set_map_preamble_detect(sx127x_t *dev, uint8_t map)
{
    DEBUG("[sx127x] Set MapPreambleDetect to 0x%02X\n", map);

    sx127x_reg_write(dev, SX127X_REG_DIOMAPPING2,
                     (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING2) &
                     SX127X_RF_DIOMAPPING2_MAP_MASK) |
                     map);
}

void sx127x_set_rx(sx127x_t *dev)
{
    DEBUG("[sx127x] Set RX\n");

    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
        {
            if (dev->settings.fsk.flags & SX127X_AFC_ON_FLAG) {
                sx127x_reg_write(dev, SX127X_REG_RXCONFIG,
                        ((sx127x_reg_read(dev, SX127X_REG_RXCONFIG) &
                                SX127X_RF_RXCONFIG_RESTARTRXONCOLLISION_MASK &
                                SX127X_RF_RXCONFIG_AFCAUTO_MASK &
                                SX127X_RF_RXCONFIG_AGCAUTO_MASK) |
                                SX127X_RF_RXCONFIG_RESTARTRXONCOLLISION_OFF |
                                SX127X_RF_RXCONFIG_AFCAUTO_ON |
                                SX127X_RF_RXCONFIG_AGCAUTO_ON));
            }
            else {
                sx127x_reg_write(dev, SX127X_REG_RXCONFIG,
                                 ((sx127x_reg_read(dev, SX127X_REG_RXCONFIG) &
                                 SX127X_RF_RXCONFIG_RESTARTRXONCOLLISION_MASK &
                                 SX127X_RF_RXCONFIG_RESTARTRXWITHPLLLOCK_MASK &
                                 SX127X_RF_RXCONFIG_AFCAUTO_MASK &
                                 SX127X_RF_RXCONFIG_AGCAUTO_MASK) |
                                 SX127X_RF_RXCONFIG_RESTARTRXONCOLLISION_ON |
                                 SX127X_RF_RXCONFIG_RESTARTRXWITHPLLLOCK |
                                 SX127X_RF_RXCONFIG_AFCAUTO_OFF |
                                 SX127X_RF_RXCONFIG_AGCAUTO_ON));
            }

//            if ((dev->settings.fsk.flags & SX127X_RX_FSK_CONTINUOUS_FLAG) == false) {
//                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
//                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
//                                 SX127X_RF_DIOMAPPING1_DIO0_MASK &
//                                 SX127X_RF_DIOMAPPING1_DIO1_MASK &
//                                 SX127X_RF_DIOMAPPING1_DIO2_MASK &
//                                 SX127X_RF_DIOMAPPING1_DIO3_MASK) |
//                                 SX127X_RF_DIOMAPPING1_DIO0_00 |  /* PayloadReady */
//                                 SX127X_RF_DIOMAPPING1_DIO1_00 |  /* FifoLevel */
//                                 SX127X_RF_DIOMAPPING1_DIO2_11 |  /* SyncAddress */
//                                 SX127X_RF_DIOMAPPING1_DIO3_00);  /* FifoEmpty */
//                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING2,
//                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING2) &
//                                 SX127X_RF_DIOMAPPING2_DIO4_MASK &
//                                 SX127X_RF_DIOMAPPING2_DIO5_MASK &
//                                 SX127X_RF_DIOMAPPING2_MAP_MASK) |
//                                 SX127X_RF_DIOMAPPING2_DIO4_11 |            /* PreambleDetect */
//                                 SX127X_RF_DIOMAPPING2_DIO5_11 |            /* ModeReady */
//                                 SX127X_RF_DIOMAPPING2_MAP_PREAMBLEDETECT); /* PreambleDetect */
//            }
//            else {
//                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
//                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
//                                 SX127X_RF_DIOMAPPING1_DIO0_MASK &
//                                 SX127X_RF_DIOMAPPING1_DIO1_MASK &
//                                 SX127X_RF_DIOMAPPING1_DIO2_MASK &
//                                 SX127X_RF_DIOMAPPING1_DIO3_MASK) |
//                                 SX127X_RF_DIOMAPPING1_DIO0_11 |  /* Disabled */
//                                 SX127X_RF_DIOMAPPING1_DIO1_00 |  /* DClk -> GPIO interrupt must be enabled */
//                                 SX127X_RF_DIOMAPPING1_DIO2_00 |  /* Data -> GPIO interrupt must be disabled */
//                                 SX127X_RF_DIOMAPPING1_DIO3_10);  /* Disabled */
//
//                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING2,
//                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING2) &
//                                 SX127X_RF_DIOMAPPING2_DIO4_MASK &
//                                 SX127X_RF_DIOMAPPING2_DIO5_MASK &
//                                 SX127X_RF_DIOMAPPING2_MAP_MASK) |
//                                 SX127X_RF_DIOMAPPING2_DIO4_00 |            /* ModeReady */
//                                 SX127X_RF_DIOMAPPING2_DIO5_10 |            /* PreambleDetect */
//                                 SX127X_RF_DIOMAPPING2_MAP_RSSI); /* PreambleDetect */
//            }

        }
        sx127x_set_op_mode(dev, SX127X_RF_OPMODE_RECEIVER);
        break;
        case SX127X_MODEM_LORA:
        {
            sx127x_reg_write(dev, SX127X_REG_LR_INVERTIQ,
                             ((sx127x_reg_read(dev, SX127X_REG_LR_INVERTIQ) &
                               SX127X_RF_LORA_INVERTIQ_TX_MASK &
                               SX127X_RF_LORA_INVERTIQ_RX_MASK) |
                              ((dev->settings.lora.flags & SX127X_IQ_INVERTED_FLAG) ? SX127X_RF_LORA_INVERTIQ_RX_ON :SX127X_RF_LORA_INVERTIQ_RX_OFF) |
                              SX127X_RF_LORA_INVERTIQ_TX_OFF));
            sx127x_reg_write(dev, SX127X_REG_LR_INVERTIQ2,
                             ((dev->settings.lora.flags & SX127X_IQ_INVERTED_FLAG) ? SX127X_RF_LORA_INVERTIQ2_ON : SX127X_RF_LORA_INVERTIQ2_OFF));

#if defined(MODULE_SX1276)
            /* ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal */
            if (dev->settings.bandwidth < 9) {
                sx127x_reg_write(dev, SX127X_REG_LR_DETECTOPTIMIZE,
                                 sx127x_reg_read(dev, SX127X_REG_LR_DETECTOPTIMIZE) & 0x7F);
                sx127x_reg_write(dev, SX127X_REG_LR_TEST30, 0x00);
                switch (dev->settings.bandwidth) {
                    case LORA_BW_125_KHZ: /* 125 kHz */
                        sx127x_reg_write(dev, SX127X_REG_LR_TEST2F, 0x40);
                        break;
                    case LORA_BW_250_KHZ: /* 250 kHz */
                        sx127x_reg_write(dev, SX127X_REG_LR_TEST2F, 0x40);
                        break;

                    default:
                        break;
                }
            }
            else {
                sx127x_reg_write(dev, SX127X_REG_LR_DETECTOPTIMIZE,
                                 sx127x_reg_read(dev, SX127X_REG_LR_DETECTOPTIMIZE) | 0x80);
            }
#endif

            /* Setup interrupts */
            if (dev->settings.lora.flags & SX127X_CHANNEL_HOPPING_FLAG) {
                sx127x_reg_write(dev, SX127X_REG_LR_IRQFLAGSMASK,
                                 /* SX127X_RF_LORA_IRQFLAGS_RXTIMEOUT |
                                    SX127X_RF_LORA_IRQFLAGS_RXDONE |
                                    SX127X_RF_LORA_IRQFLAGS_PAYLOADCRCERROR | */
                                 SX127X_RF_LORA_IRQFLAGS_VALIDHEADER |
                                 SX127X_RF_LORA_IRQFLAGS_TXDONE |
                                 SX127X_RF_LORA_IRQFLAGS_CADDONE |
                                 /* SX127X_RF_LORA_IRQFLAGS_FHSSCHANGEDCHANNEL | */
                                 SX127X_RF_LORA_IRQFLAGS_CADDETECTED);

                /* DIO0=RxDone, DIO2=FhssChangeChannel */
                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
                                  SX127X_RF_LORA_DIOMAPPING1_DIO0_MASK &
                                  SX127X_RF_LORA_DIOMAPPING1_DIO2_MASK) |
                                 SX127X_RF_LORA_DIOMAPPING1_DIO0_00 |
                                 SX127X_RF_LORA_DIOMAPPING1_DIO2_00);
            }
            else {
                sx127x_reg_write(dev, SX127X_REG_LR_IRQFLAGSMASK,
                                 /* SX127X_RF_LORA_IRQFLAGS_RXTIMEOUT |
                                    SX127X_RF_LORA_IRQFLAGS_RXDONE |
                                    SX127X_RF_LORA_IRQFLAGS_PAYLOADCRCERROR | */
                                 SX127X_RF_LORA_IRQFLAGS_VALIDHEADER |
                                 SX127X_RF_LORA_IRQFLAGS_TXDONE |
                                 SX127X_RF_LORA_IRQFLAGS_CADDONE |
                                 SX127X_RF_LORA_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 SX127X_RF_LORA_IRQFLAGS_CADDETECTED);

                /* DIO0=RxDone */
                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
                                  SX127X_RF_LORA_DIOMAPPING1_DIO0_MASK) |
                                 SX127X_RF_LORA_DIOMAPPING1_DIO0_00);
            }

            sx127x_reg_write(dev, SX127X_REG_LR_FIFORXBASEADDR, 0);
            sx127x_reg_write(dev, SX127X_REG_LR_FIFOADDRPTR, 0);
        }

        if (dev->settings.lora.rx_timeout != 0) {
            xtimer_set(&(dev->_internal.rx_timeout_timer), dev->settings.lora.rx_timeout);
        }

        if (dev->settings.lora.flags & SX127X_RX_CONTINUOUS_FLAG) {
            sx127x_set_op_mode(dev, SX127X_RF_LORA_OPMODE_RECEIVER);
        }
        else {
            sx127x_set_op_mode(dev, SX127X_RF_LORA_OPMODE_RECEIVER_SINGLE);
        }
        /*
         * Enabling interrupts if they were disabled due to a
         * reception mode change.
         */
        gpio_irq_enable(dev->params.dio1_pin);
        gpio_irq_enable(dev->params.dio2_pin);
        break;
    }

    sx127x_set_state(dev, SX127X_RF_RX_RUNNING);
}

void sx127x_set_tx(sx127x_t *dev)
{
     switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
        {
            if ((dev->settings.fsk.flags & SX127X_RX_FSK_CONTINUOUS_FLAG) == false) {
                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
                                 SX127X_RF_DIOMAPPING1_DIO0_MASK &
                                 SX127X_RF_DIOMAPPING1_DIO1_MASK &
                                 SX127X_RF_DIOMAPPING1_DIO2_MASK &
                                 SX127X_RF_DIOMAPPING1_DIO3_MASK) |
                                 SX127X_RF_DIOMAPPING1_DIO0_10 | /* PacketSent */
                                 SX127X_RF_DIOMAPPING1_DIO1_00 | /* FifoLevel */
                                 SX127X_RF_DIOMAPPING1_DIO2_11 | /* FifoFull */
                                 SX127X_RF_DIOMAPPING1_DIO3_00); /* FifoEmpty */

                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING2,
                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING2) &
                                 SX127X_RF_DIOMAPPING2_DIO4_MASK &
                                 SX127X_RF_DIOMAPPING2_DIO5_MASK) |
                                 SX127X_RF_DIOMAPPING2_DIO4_00 | /* LowBat */
                                 SX127X_RF_DIOMAPPING2_DIO5_11); /* ModeReady */
            }
            else {
                /* TODO: set up continuous mode */
            }
        }
            break;
        case SX127X_MODEM_LORA:
        {
            if (dev->settings.lora.flags & SX127X_CHANNEL_HOPPING_FLAG) {
                sx127x_reg_write(dev, SX127X_REG_LR_IRQFLAGSMASK,
                                 SX127X_RF_LORA_IRQFLAGS_RXTIMEOUT |
                                 SX127X_RF_LORA_IRQFLAGS_RXDONE |
                                 SX127X_RF_LORA_IRQFLAGS_PAYLOADCRCERROR |
                                 SX127X_RF_LORA_IRQFLAGS_VALIDHEADER |
                                 /* RFLR_IRQFLAGS_TXDONE | */
                                 SX127X_RF_LORA_IRQFLAGS_CADDONE |
                                 /* RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | */
                                 SX127X_RF_LORA_IRQFLAGS_CADDETECTED);

                /* DIO0=TxDone, DIO2=FhssChangeChannel */
                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1 ) &
                                  SX127X_RF_LORA_DIOMAPPING1_DIO0_MASK &
                                  SX127X_RF_LORA_DIOMAPPING1_DIO2_MASK) |
                                 SX127X_RF_LORA_DIOMAPPING1_DIO0_01 |
                                 SX127X_RF_LORA_DIOMAPPING1_DIO2_00);
            }
            else
            {
                sx127x_reg_write(dev, SX127X_REG_LR_IRQFLAGSMASK,
                                 SX127X_RF_LORA_IRQFLAGS_RXTIMEOUT |
                                 SX127X_RF_LORA_IRQFLAGS_RXDONE |
                                 SX127X_RF_LORA_IRQFLAGS_PAYLOADCRCERROR |
                                 SX127X_RF_LORA_IRQFLAGS_VALIDHEADER |
                                 /* RFLR_IRQFLAGS_TXDONE | */
                                 SX127X_RF_LORA_IRQFLAGS_CADDONE |
                                 SX127X_RF_LORA_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 SX127X_RF_LORA_IRQFLAGS_CADDETECTED);

                /* DIO0=TxDone */
                sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1,
                                 (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
                                  SX127X_RF_LORA_DIOMAPPING1_DIO0_MASK) |
                                  SX127X_RF_LORA_DIOMAPPING1_DIO0_01);
            }
        }

        sx127x_set_state(dev, SX127X_RF_RX_RUNNING);
        if (dev->settings.lora.tx_timeout != 0) {
            xtimer_set(&(dev->_internal.tx_timeout_timer), dev->settings.lora.tx_timeout);
        }

        break;
    }

    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_TRANSMITTER );
}

uint8_t sx127x_get_max_payload_len(const sx127x_t *dev)
{
    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
        {
            uint8_t length = sx127x_reg_read(dev, SX127X_REG_PAYLOADLENGTH);
            return length;
        }

        case SX127X_MODEM_LORA:
            return sx127x_reg_read(dev, SX127X_REG_LR_PAYLOADMAXLENGTH);
    }

    /* should never be reached */
    return 0;
}

void sx127x_set_max_payload_len(const sx127x_t *dev, uint8_t maxlen)
{
    DEBUG("[sx127x] Set max payload len: %d\n", maxlen);

    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
            DEBUG("[sx127x] No max payload in FSK mode\n");
            break;

        case SX127X_MODEM_LORA:
            sx127x_reg_write(dev, SX127X_REG_LR_PAYLOADMAXLENGTH, maxlen);
            break;
    }
}

uint8_t sx127x_get_op_mode(const sx127x_t *dev)
{
    return sx127x_reg_read(dev, SX127X_REG_OPMODE) & ~SX127X_RF_OPMODE_MASK;
}

void sx127x_set_op_mode(const sx127x_t *dev, uint8_t op_mode)
{
#if ENABLE_DEBUG
    switch(op_mode) {
    case SX127X_RF_OPMODE_SLEEP:
        DEBUG("[sx127x] Set op mode: SLEEP\n");
        break;
    case SX127X_RF_OPMODE_STANDBY:
        DEBUG("[sx127x] Set op mode: STANDBY\n");
        break;
    case SX127X_RF_OPMODE_RECEIVER_SINGLE:
        DEBUG("[sx127x] Set op mode: RECEIVER SINGLE\n");
        break;
    case SX127X_RF_OPMODE_RECEIVER:
        DEBUG("[sx127x] Set op mode: RECEIVER\n");
        break;
    case SX127X_RF_OPMODE_TRANSMITTER:
        DEBUG("[sx127x] Set op mode: TRANSMITTER\n");
        break;
    default:
        DEBUG("[sx127x] Set op mode: UNKNOWN (%d)\n", op_mode);
        break;
    }
#endif

    /* Replace previous mode value and setup new mode value */
    sx127x_reg_write(dev, SX127X_REG_OPMODE,
                     (sx127x_reg_read(dev, SX127X_REG_OPMODE) & SX127X_RF_OPMODE_MASK) | op_mode);
}

uint8_t sx127x_get_bandwidth(const sx127x_t *dev)
{
    return dev->settings.bandwidth;
}

static void _low_datarate_optimize(sx127x_t *dev)
{
    if ( ((dev->settings.bandwidth == LORA_BW_125_KHZ) &&
          ((dev->settings.lora.datarate == LORA_SF11) ||
           (dev->settings.lora.datarate == LORA_SF12))) ||
         ((dev->settings.bandwidth == LORA_BW_250_KHZ) &&
          (dev->settings.lora.datarate == LORA_SF12))) {
        dev->settings.lora.flags |= SX127X_LOW_DATARATE_OPTIMIZE_FLAG;
    } else {
        dev->settings.lora.flags &= ~SX127X_LOW_DATARATE_OPTIMIZE_FLAG;
    }

#if defined(MODULE_SX1272)
    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG1,
                     (sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG1) &
                      SX127X_RF_LORA_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK) |
                     ((dev->settings.lora.flags & SX127X_LOW_DATARATE_OPTIMIZE_FLAG)));
#else /* MODULE_SX1276 */
    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG3,
                     (sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG3) &
                      SX127X_RF_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                     ((dev->settings.lora.flags & SX127X_LOW_DATARATE_OPTIMIZE_FLAG) << 3));
#endif
}

static void _update_bandwidth(const sx127x_t *dev)
{
    uint8_t config1_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG1);
#if defined(MODULE_SX1272)
    config1_reg &= SX1272_RF_LORA_MODEMCONFIG1_BW_MASK;
    switch (dev->settings.bandwidth) {
    case LORA_BW_125_KHZ:
        config1_reg |=  SX1272_RF_LORA_MODEMCONFIG1_BW_125_KHZ;
        break;
    case LORA_BW_250_KHZ:
        config1_reg |=  SX1272_RF_LORA_MODEMCONFIG1_BW_250_KHZ;
        break;
    case LORA_BW_500_KHZ:
        config1_reg |=  SX1272_RF_LORA_MODEMCONFIG1_BW_500_KHZ;
        break;
    default:
        DEBUG("Unsupported bandwidth, %ld", dev->settings.bandwidth);
        break;
    }
#else /* MODULE_SX1276 */
    config1_reg &= SX1276_RF_LORA_MODEMCONFIG1_BW_MASK;
    switch (dev->settings.bandwidth) {
    case LORA_BW_125_KHZ:
        config1_reg |= SX1276_RF_LORA_MODEMCONFIG1_BW_125_KHZ;
        break;
    case LORA_BW_250_KHZ:
        config1_reg |=  SX1276_RF_LORA_MODEMCONFIG1_BW_250_KHZ;
        break;
    case LORA_BW_500_KHZ:
        config1_reg |=  SX1276_RF_LORA_MODEMCONFIG1_BW_500_KHZ;
        break;
    default:
        DEBUG("[sx127x] Unsupported bandwidth, %ld\n",
              dev->settings.bandwidth);
        break;
    }
#endif
    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG1, config1_reg);
}

void sx127x_set_bandwidth(sx127x_t *dev, uint8_t bandwidth)
{
    DEBUG("[sx127x] Set bandwidth: %d\n", bandwidth);

    dev->settings.bandwidth = bandwidth;

    _update_bandwidth((const sx127x_t *)dev);

    _low_datarate_optimize(dev);

    /* ERRATA sensitivity tweaks */
    if ((dev->settings.bandwidth == LORA_BW_500_KHZ) &&
        (dev->settings.channel > SX127X_RF_MID_BAND_THRESH)) {
        /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
        sx127x_reg_write(dev, SX127X_REG_LR_TEST36, 0x02);
        sx127x_reg_write(dev, SX127X_REG_LR_TEST3A, 0x64);
    }
    else if (dev->settings.bandwidth == LORA_BW_500_KHZ) {
        /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
        sx127x_reg_write(dev, SX127X_REG_LR_TEST36, 0x02);
        sx127x_reg_write(dev, SX127X_REG_LR_TEST3A, 0x7F);
    }
    else {
        /* ERRATA 2.1 - Sensitivity Optimization with another Bandwidth */
        sx127x_reg_write(dev, SX127X_REG_LR_TEST36, 0x03);
    }
}

uint8_t sx127x_get_spreading_factor(const sx127x_t *dev)
{
    return dev->settings.lora.datarate;
}

void sx127x_set_spreading_factor(sx127x_t *dev, uint8_t datarate)
{
    DEBUG("[sx127x] Set spreading factor: %d\n", datarate);

    if (datarate == LORA_SF6 &&
        !(dev->settings.lora.flags & SX127X_ENABLE_FIXED_HEADER_LENGTH_FLAG)) {
        /* SF 6 is only valid when using explicit header mode */
        DEBUG("Spreading Factor 6 can only be used when explicit header "
              "mode is set, this mode is not supported by this driver."
              "Ignoring.\n");
        return;
    }

    dev->settings.lora.datarate = datarate;

    uint8_t config2_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG2);
    config2_reg &= SX127X_RF_LORA_MODEMCONFIG2_SF_MASK;
    config2_reg |= datarate << 4;
    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG2, config2_reg);

    _low_datarate_optimize(dev);

    switch(dev->settings.lora.datarate) {
    case LORA_SF6:
        sx127x_reg_write(dev, SX127X_REG_LR_DETECTOPTIMIZE,
                         SX127X_RF_LORA_DETECTIONOPTIMIZE_SF6);
        sx127x_reg_write(dev, SX127X_REG_LR_DETECTIONTHRESHOLD,
                         SX127X_RF_LORA_DETECTIONTHRESH_SF6);
        break;
    default:
        sx127x_reg_write(dev, SX127X_REG_LR_DETECTOPTIMIZE,
                         SX127X_RF_LORA_DETECTIONOPTIMIZE_SF7_TO_SF12);
        sx127x_reg_write(dev, SX127X_REG_LR_DETECTIONTHRESHOLD,
                         SX127X_RF_LORA_DETECTIONTHRESH_SF7_TO_SF12);
        break;
    }
}

uint8_t sx127x_get_coding_rate(const sx127x_t *dev)
{
    return dev->settings.lora.coderate;
}

void sx127x_set_coding_rate(sx127x_t *dev, uint8_t coderate)
{
    DEBUG("[sx127x] Set coding rate: %d\n", coderate);

    dev->settings.lora.coderate = coderate;
    uint8_t config1_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG1);

#if defined(MODULE_SX1272)
    config1_reg &= SX1272_RF_LORA_MODEMCONFIG1_CODINGRATE_MASK;
    config1_reg |= coderate << 3;
#else /* MODULE_SX1276 */
    config1_reg &= SX1276_RF_LORA_MODEMCONFIG1_CODINGRATE_MASK;
    config1_reg |= coderate << 1;
#endif

    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG1, config1_reg);
}

static inline void _set_flag(sx127x_t *dev, uint8_t flag, bool value)
{
    switch (dev->settings.modem) {
    case SX127X_MODEM_LORA:
        if (value) {
            dev->settings.lora.flags |= flag;
        }
        else {
            dev->settings.lora.flags &= ~flag;
        }
        break;
    case SX127X_MODEM_FSK:
        if (value) {
            dev->settings.fsk.flags |= flag;
        }
        else {
            dev->settings.fsk.flags &= ~flag;
        }
    }
}

bool sx127x_get_rx_single(const sx127x_t *dev)
{
    return !(dev->settings.lora.flags & SX127X_RX_CONTINUOUS_FLAG);
}

void sx127x_set_rx_single(sx127x_t *dev, bool single)
{
    DEBUG("[sx127x] Set RX single: %d\n", single);
    _set_flag(dev, SX127X_RX_CONTINUOUS_FLAG, !single);
}

bool sx127x_get_crc(const sx127x_t *dev)
{
#if defined(MODULE_SX1272)
    return (sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG1) &
            SX1272_RF_LORA_MODEMCONFIG1_RXPAYLOADCRC_MASK);
#else /* MODULE_SX1276 */
    return (sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG2) &
            SX1276_RF_LORA_MODEMCONFIG2_RXPAYLOADCRC_MASK);
#endif
}

void sx127x_set_crc(sx127x_t *dev, bool crc)
{
    DEBUG("[sx127x] Set CRC: %d\n", crc);

    switch (dev->settings.modem) {
    case SX127X_MODEM_LORA:
        _set_flag(dev, SX127X_ENABLE_CRC_FLAG, crc);

#if defined(MODULE_SX1272)
        uint8_t config2_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG1);
        config2_reg &= SX1272_RF_LORA_MODEMCONFIG1_RXPAYLOADCRC_MASK;
        config2_reg |= crc << 1;
        sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG1, config2_reg);
#else /* MODULE_SX1276 */
        uint8_t config2_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG2);
        config2_reg &= SX1276_RF_LORA_MODEMCONFIG2_RXPAYLOADCRC_MASK;
        config2_reg |= crc << 2;
        sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG2, config2_reg);
#endif
        break;
    case SX127X_MODEM_FSK:
        _set_flag(dev, SX127X_CRC_ON_FLAG, crc);
    }
}

uint8_t sx127x_get_hop_period(const sx127x_t *dev)
{
    return sx127x_reg_read(dev, SX127X_REG_LR_HOPPERIOD);
}

void sx127x_set_hop_period(sx127x_t *dev, uint8_t hop_period)
{
    DEBUG("[sx127x] Set Hop period: %d\n", hop_period);

    dev->settings.lora.freq_hop_period = hop_period;

    uint8_t tmp = sx127x_reg_read(dev, SX127X_REG_LR_PLLHOP);
    if (dev->settings.lora.flags & SX127X_CHANNEL_HOPPING_FLAG) {
        tmp |= SX127X_RF_LORA_PLLHOP_FASTHOP_ON;
        sx127x_reg_write(dev, SX127X_REG_LR_PLLHOP, tmp);
        sx127x_reg_write(dev, SX127X_REG_LR_HOPPERIOD, hop_period);
    }
}

bool  sx127x_get_fixed_header_len_mode(const sx127x_t *dev)
{
    return dev->settings.lora.flags & SX127X_ENABLE_FIXED_HEADER_LENGTH_FLAG;
}

void sx127x_set_fixed_header_len_mode(sx127x_t *dev, bool fixed_len)
{
    DEBUG("[sx127x] Set fixed header length: %d\n", fixed_len);

    _set_flag(dev, SX127X_ENABLE_FIXED_HEADER_LENGTH_FLAG, fixed_len);

    uint8_t config1_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG1);
#if defined(MODULE_SX1272)
    config1_reg &= SX1272_RF_LORA_MODEMCONFIG1_IMPLICITHEADER_MASK;
    config1_reg |= fixed_len << 2;
#else /* MODULE_SX1276 */
    config1_reg &= SX1276_RF_LORA_MODEMCONFIG1_IMPLICITHEADER_MASK;
    config1_reg |= fixed_len;
#endif
    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG1, config1_reg);
}

uint8_t sx127x_get_payload_length(const sx127x_t *dev)
{
    return sx127x_reg_read(dev, SX127X_REG_LR_PAYLOADLENGTH);;
}

void sx127x_set_payload_length(sx127x_t *dev, uint16_t len)
{
    DEBUG("[sx127x] Set payload len: %d\n", len);

    if (dev->settings.modem == SX127X_MODEM_FSK) {
        dev->settings.fsk.payload_len = len;
        sx127x_reg_write(dev, SX127X_REG_PACKETCONFIG2,
                         (sx127x_reg_read(dev, SX127X_REG_PACKETCONFIG2) &
                         SX127X_RF_PACKETCONFIG2_PAYLOADLENGTH_MSB_MASK) |
                         (len >> 8));
        sx127x_reg_write(dev, SX127X_REG_PAYLOADLENGTH, (uint8_t)len);
    }
    else if (dev->settings.modem == SX127X_MODEM_LORA) {
        sx127x_reg_write(dev, SX127X_REG_LR_PAYLOADLENGTH, len);
    }
}

static inline uint8_t sx127x_get_pa_select(const sx127x_t *dev)
{
    if (dev->params.paselect == SX127X_PA_BOOST) {
        return SX127X_RF_PACONFIG_PASELECT_PABOOST;
    }

    return SX127X_RF_PACONFIG_PASELECT_RFO;
}

uint8_t sx127x_get_tx_power(const sx127x_t *dev)
{
    return dev->settings.power;
}

void sx127x_set_tx_power(sx127x_t *dev, int8_t power)
{
    DEBUG("[sx127x] Set power: %d\n", power);

    dev->settings.power = power;

    uint8_t pa_config = sx127x_reg_read(dev, SX127X_REG_PACONFIG);
#if defined(MODULE_SX1272)
    uint8_t pa_dac = sx127x_reg_read(dev, SX1272_REG_PADAC);
#else /* MODULE_SX1276 */
    uint8_t pa_dac = sx127x_reg_read(dev, SX1276_REG_PADAC);
#endif

    pa_config = ((pa_config & SX127X_RF_PACONFIG_PASELECT_MASK) |
                 sx127x_get_pa_select(dev));

#if defined(MODULE_SX1276)
    /* max power is 14dBm */
    pa_config = (pa_config & SX127X_RF_PACONFIG_MAX_POWER_MASK) | 0x70;
#endif

    sx127x_reg_write(dev, SX127X_REG_PARAMP, SX127X_RF_PARAMP_0050_US);

    if ((pa_config & SX127X_RF_PACONFIG_PASELECT_PABOOST)
        == SX127X_RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            pa_dac = ((pa_dac & SX127X_RF_PADAC_20DBM_MASK) |
                      SX127X_RF_PADAC_20DBM_ON);
        } else {
            pa_dac = ((pa_dac & SX127X_RF_PADAC_20DBM_MASK) |
                      SX127X_RF_PADAC_20DBM_OFF);
        }
        if ((pa_dac & SX127X_RF_PADAC_20DBM_ON) == SX127X_RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }
            if (power > 20) {
                power = 20;
            }

            pa_config = ((pa_config & SX127X_RF_PACONFIG_OUTPUTPOWER_MASK) |
                         (uint8_t)((uint16_t)(power - 5) & 0x0F));
        } else {
            if (power < 2) {
                power = 2;
            }
            if (power > 17) {
                power = 17;
            }

            pa_config = ((pa_config & SX127X_RF_PACONFIG_OUTPUTPOWER_MASK) |
                         (uint8_t)((uint16_t)(power - 2) & 0x0F));
        }
    } else {
        if (power < -1) {
            power = -1;
        }
        if (power > 14) {
            power = 14;
        }

        pa_config = ((pa_config & SX127X_RF_PACONFIG_OUTPUTPOWER_MASK) |
                     (uint8_t)((uint16_t)(power + 1) & 0x0F));
    }

    sx127x_reg_write(dev, SX127X_REG_PACONFIG, pa_config);
#if defined(MODULE_SX1272)
    sx127x_reg_write(dev, SX1272_REG_PADAC, pa_dac);
#else /* MODULE_SX1276 */
    sx127x_reg_write(dev, SX1276_REG_PADAC, pa_dac);
#endif
}

uint16_t sx127x_get_preamble_length(const sx127x_t *dev)
{
    return dev->settings.preamble_len;
}

void sx127x_set_preamble_length(sx127x_t *dev, uint16_t preamble)
{
    DEBUG("[sx127x] Set preamble length: 0x%04x\n", preamble);

    dev->settings.preamble_len = preamble;

    sx127x_reg_write(dev, SX127X_REG_LR_PREAMBLEMSB,
                     (preamble >> 8) & 0xFF);
    sx127x_reg_write(dev, SX127X_REG_LR_PREAMBLELSB,
                     preamble & 0xFF);
}

void sx127x_set_rx_timeout(sx127x_t *dev, uint32_t timeout)
{
    DEBUG("[sx127x] Set RX timeout: %" PRIu32 "\n", timeout);

    dev->settings.lora.rx_timeout = timeout;
}

void sx127x_set_tx_timeout(sx127x_t *dev, uint32_t timeout)
{
    DEBUG("[sx127x] Set TX timeout: %" PRIu32 "\n", timeout);

    dev->settings.lora.tx_timeout = timeout;
}

void sx127x_set_symbol_timeout(sx127x_t *dev, uint16_t timeout)
{
    DEBUG("[sx127x] Set symbol timeout: %d\n", timeout);

    uint8_t config2_reg = sx127x_reg_read(dev, SX127X_REG_LR_MODEMCONFIG2);
    config2_reg &= SX127X_RF_LORA_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK;
    config2_reg |= (timeout >> 8) & ~SX127X_RF_LORA_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK;
    sx127x_reg_write(dev, SX127X_REG_LR_MODEMCONFIG2, config2_reg);
    sx127x_reg_write(dev, SX127X_REG_LR_SYMBTIMEOUTLSB, timeout & 0xFF);
}

bool sx127x_get_iq_invert(const sx127x_t *dev)
{
    return dev->settings.lora.flags & SX127X_IQ_INVERTED_FLAG;
}

void sx127x_set_iq_invert(sx127x_t *dev, bool iq_invert)
{
    DEBUG("[sx127x] Set IQ invert: %d\n", iq_invert);

    _set_flag(dev, SX127X_IQ_INVERTED_FLAG, iq_invert);

    sx127x_reg_write(dev, SX127X_REG_LR_INVERTIQ,
                     (sx127x_reg_read(dev, SX127X_REG_LR_INVERTIQ) &
                      SX127X_RF_LORA_INVERTIQ_RX_MASK &
                      SX127X_RF_LORA_INVERTIQ_TX_MASK) |
                      SX127X_RF_LORA_INVERTIQ_RX_OFF |
                     (iq_invert ? SX127X_RF_LORA_INVERTIQ_TX_ON : SX127X_RF_LORA_INVERTIQ_TX_OFF));

    sx127x_reg_write(dev, SX127X_REG_LR_INVERTIQ2,
                     (iq_invert ? SX127X_RF_LORA_INVERTIQ2_ON : SX127X_RF_LORA_INVERTIQ2_OFF));
}

void sx127x_set_freq_hop(sx127x_t *dev, bool freq_hop_on)
{
    DEBUG("[sx127x] Set freq hop: %d\n", freq_hop_on);

     _set_flag(dev, SX127X_CHANNEL_HOPPING_FLAG, freq_hop_on);
}

void sx127x_set_fsk_mod_shaping(sx127x_t *dev, uint8_t mode)
{
    DEBUG("[sx127x] Set modulation shaping: 0x%02x\n", mode >> 3);
    dev->settings.fsk.mod_shaping = mode;
    sx127x_reg_write(dev, SX127X_REG_OPMODE,
                     (sx127x_reg_read(dev, SX127X_REG_OPMODE) &
                      SX127X_RF_OPMODE_MODULATIONSHAPING_MASK) | mode);
}

uint8_t sx127x_get_fsk_mod_shaping(const sx127x_t *dev)
{
    return dev->settings.fsk.mod_shaping;
}

void sx127x_set_lna(sx127x_t *dev, uint8_t value)
{
    DEBUG("[sx127x] Set LNA: %d\n", value);
    dev->settings.lna = value;
    sx127x_reg_write(dev, SX127X_REG_LR_LNA,
                     (sx127x_reg_read(dev, SX127X_REG_LR_LNA) &
                      SX127X_RF_LNA_GAIN_MASK) | value);
}

void sx127x_set_syncconfig(sx127x_t *dev, uint8_t autorestart_rx_mode,
                           uint8_t preamble_polarity, uint8_t sync,
                           uint8_t fifo_fill_cond, uint8_t sync_size)
{
    DEBUG("[sx127x] Set autorestart rx mode: 0x%02X\n"
          "[sx127x] Set preamble polarity: 0x%02X\n"
          "[sx127x] Set sync: 0x%02X\n"
          "[sx127x] Set FifoFillCondition: 0x%02X\n"
          "[sx127x] Set sync size: 0x%02X\n",
          autorestart_rx_mode >> 6,
          preamble_polarity >> 5,
          sync >> 4,
          fifo_fill_cond >> 3,
          sync_size);
    dev->settings.fsk.sysconfig = (sx127x_reg_read(dev, SX127X_REG_SYNCCONFIG) &
                                  SX127X_RF_SYNCCONFIG_AUTORESTARTRXMODE_MASK &
                                  SX127X_RF_SYNCCONFIG_PREAMBLEPOLARITY_MASK &
                                  SX127X_RF_SYNCCONFIG_SYNC_MASK &
                                  SX127X_RF_SYNCCONFIG_FIFOFILL_MASK &
                                  SX127X_RF_SYNCCONFIG_SYNCSIZE_MASK) |
                                  autorestart_rx_mode |
                                  preamble_polarity |
                                  sync |
                                  fifo_fill_cond |
                                  sync_size;

    sx127x_reg_write(dev, SX127X_REG_SYNCCONFIG, dev->settings.fsk.sysconfig);
}

void sx127x_set_packetconfig1(sx127x_t *dev, uint8_t packet_format, uint8_t dcfree,
                              uint8_t crc, uint8_t crc_autoclear, uint8_t addrs_filtering,
                              uint8_t crc_whitening_type)
{
    DEBUG("[sx127x] Set packet format: 0x%02x\n"
          "[sx127x] Set DC free: 0x%02x\n"
          "[sx127x] Set CRC ON: 0x%02x\n"
          "[sx127x] Set CRC autoclear: 0x%02x\n"
          "[sx127x] Set Address filtering: 0x%02x\n"
          "[sx127x] Set CRC whitening type: 0x%02x\n",
                   packet_format,
                   dcfree,
                   crc,
                   crc_autoclear,
                   addrs_filtering,
                   crc_whitening_type);

    if (packet_format == SX127X_RF_PACKETCONFIG1_PACKETFORMAT_FIXED) {
        _set_flag(dev, SX127X_FIX_LEN_FLAG, true);
    }
    else {
        _set_flag(dev, SX127X_FIX_LEN_FLAG, false);
    }

    if (crc == SX127X_RF_PACKETCONFIG1_CRC_ON) {
        _set_flag(dev, SX127X_CRC_ON_FLAG, true);
    }
    else {
        _set_flag(dev, SX127X_CRC_ON_FLAG, false);
    }

    dev->settings.fsk.pktconfig1 = (sx127x_reg_read(dev, SX127X_REG_PACKETCONFIG1) &
                                                   SX127X_RF_PACKETCONFIG1_PACKETFORMAT_MASK &
                                                   SX127X_RF_PACKETCONFIG1_DCFREE_MASK &
                                                   SX127X_RF_PACKETCONFIG1_CRC_MASK &
                                                   SX127X_RF_PACKETCONFIG1_CRCAUTOCLEAR_MASK &
                                                   SX127X_RF_PACKETCONFIG1_ADDRSFILTERING_MASK &
                                                   SX127X_RF_PACKETCONFIG1_CRCWHITENINGTYPE_MASK) |
                                                   packet_format |
                                                   dcfree |
                                                   crc |
                                                   crc_autoclear |
                                                   addrs_filtering |
                                                   crc_whitening_type;

    sx127x_reg_write(dev, SX127X_REG_PACKETCONFIG1, dev->settings.fsk.pktconfig1);
}

uint8_t sx127x_get_packetconfig1(const sx127x_t *dev)
{
    return dev->settings.fsk.pktconfig1;
}

void sx127x_set_packetconfig2(sx127x_t *dev, uint8_t wmbus_crc_enable, uint8_t data_mode,
                              uint8_t io_home, uint8_t beacon)
{
    DEBUG("[sx127x] Set wM-Bus CRC: 0x%02x\n"
          "[sx127x] Set data mode: 0x%02x\n"
          "[sx127x] Set ioHome: 0x%02x\n"
          "[sx127x] Set beacon: 0x%02x\n",
          wmbus_crc_enable,
          data_mode,
          io_home,
          beacon);

    if (data_mode == SX127X_RF_PACKETCONFIG2_DATAMODE_CONTINUOUS) {
        _set_flag(dev, SX127X_RX_FSK_CONTINUOUS_FLAG, true);
        gpio_irq_disable(dev->params.dio1_pin);
        gpio_irq_disable(dev->params.dio2_pin);
        sx127x_config_dio(dev, dev->params.dio2_pin, GPIO_IN, GPIO_RISING, 2, false);
    }
    else {
        _set_flag(dev, SX127X_RX_FSK_CONTINUOUS_FLAG, false);
        sx127x_config_dio(dev, dev->params.dio2_pin, GPIO_IN, GPIO_RISING, 2, true);
    }

    dev->settings.fsk.pktconfig2 = (sx127x_reg_read(dev, SX127X_REG_PACKETCONFIG2) &
                                    SX127X_RF_PACKETCONFIG2_WMBUS_CRC_ENABLE_MASK &
                                    SX127X_RF_PACKETCONFIG2_DATAMODE_MASK &
                                    SX127X_RF_PACKETCONFIG2_IOHOME_MASK &
                                    SX127X_RF_PACKETCONFIG2_BEACON_MASK) |
                                    wmbus_crc_enable |
                                    data_mode |
                                    io_home |
                                    beacon;

    sx127x_reg_write(dev, SX127X_REG_PACKETCONFIG2, dev->settings.fsk.pktconfig2);
}

void sx127x_set_dio_mapping1(sx127x_t *dev, uint8_t dio0, uint8_t dio1,
                             uint8_t dio2, uint8_t dio3)
{
    DEBUG("[sx127x] Setting DIO 0 to: 0x%02x\n", dio0 >> 6);
    DEBUG("[sx127x] Setting DIO 1 to: 0x%02x\n", dio1 >> 4);
    DEBUG("[sx127x] Setting DIO 2 to: 0x%02x\n", dio2 >> 2);
    DEBUG("[sx127x] Setting DIO 3 to: 0x%02x\n", dio3);

    sx127x_reg_write(dev, SX127X_REG_DIOMAPPING1, (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
                     SX127X_RF_DIOMAPPING1_DIO0_MASK &
                     SX127X_RF_DIOMAPPING1_DIO1_MASK &
                     SX127X_RF_DIOMAPPING1_DIO2_MASK &
                     SX127X_RF_DIOMAPPING1_DIO3_MASK) |
                     dio0 |
                     dio1 |
                     dio2 |
                     dio3);
}

void sx127x_set_dio_mapping2(sx127x_t *dev, uint8_t dio4, uint8_t dio5)
{
    DEBUG("[sx127x] Setting DIO 4 to: 0x%02x\n", dio4 >> 6);
    DEBUG("[sx127x] Setting DIO 5 to: 0x%02x\n", dio5 >> 4);
    sx127x_reg_write(dev, SX127X_REG_DIOMAPPING2, (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING2) &
                         SX127X_RF_DIOMAPPING2_DIO4_MASK &
                         SX127X_RF_DIOMAPPING2_DIO5_MASK) |
                         dio4 |
                         dio5);
}

uint8_t sx127x_get_packetconfig2(const sx127x_t *dev)
{
    return dev->settings.fsk.pktconfig2;
}

void sx127x_set_bitrate(sx127x_t *dev, uint32_t bitrate)
{
    DEBUG("[sx127x] Set bitrate: %ld\n", bitrate);
    dev->settings.fsk.bitrate = bitrate;

    bitrate = (uint16_t)((double)SX127X_XTAL_FREQ / (double)bitrate);
    sx127x_reg_write(dev, SX127X_REG_BITRATEMSB, (uint8_t)((bitrate >> 8)));
    sx127x_reg_write(dev, SX127X_REG_BITRATELSB, (uint8_t)(bitrate & 0xFF));
}

uint32_t sx127x_get_bitrate(const sx127x_t *dev)
{
    return dev->settings.fsk.bitrate;
}

void sx127x_set_freqdev(sx127x_t *dev, uint32_t freq_dev)
{
    DEBUG("[sx127x] Set frequency deviation: %ld\n", freq_dev);
    dev->settings.fsk.freq_dev = freq_dev;

    freq_dev = (uint16_t)((double)freq_dev / (double)FSK_FREQ_STEP_DEFAULT);
    sx127x_reg_write(dev, SX127X_REG_FDEVMSB, (uint8_t)((freq_dev >> 8)));
    sx127x_reg_write(dev, SX127X_REG_FDEVLSB, (uint8_t)((freq_dev & 0xFF)));
}

uint32_t sx127x_get_freqdev(const sx127x_t *dev)
{
    return dev->settings.fsk.freq_dev;
}

void sx127x_set_rxbw(sx127x_t *dev, uint32_t value, uint32_t rx_bw_value)
{
    DEBUG("[sx127x] Set RX bandwidth value: %ld\n", rx_bw_value);
    dev->settings.bandwidth = (uint8_t)value & 0x60;

    _compute_bandwidth(dev, &dev->settings.bandwidth, rx_bw_value);

    sx127x_reg_write(dev, SX127X_REG_RXBW, dev->settings.bandwidth);
    dev->settings.bandwidth = rx_bw_value;
}

void sx127x_set_afcbw(sx127x_t *dev, uint32_t afc_bw_value)
{
    DEBUG("[sx127x] Set AFC bandwidth value: %ld\n", afc_bw_value);
    dev->settings.fsk.bandwidth_afc = 0;

    _compute_bandwidth(dev, &dev->settings.fsk.bandwidth_afc, afc_bw_value);

    sx127x_reg_write(dev, SX127X_REG_AFCBW, dev->settings.fsk.bandwidth_afc);
    dev->settings.fsk.bandwidth_afc = afc_bw_value;
}

void sx127x_fsk_set_rssi_offset(sx127x_t *dev, int8_t offset)
{
    dev->settings.fsk.rssi_offset = offset;

    if(offset < 0) {
        offset = (~offset & 0x1F);
        offset += 1;
        offset = -offset;
    }

    sx127x_reg_write(dev, SX127X_REG_RSSICONFIG,
                     (sx127x_reg_read(dev, SX127X_REG_RSSICONFIG)) |
                       (uint8_t)((offset & 0x1F) << 3));
}

int8_t sx127x_fsk_get_rssi_offset(const sx127x_t *dev)
{
    return dev->settings.fsk.rssi_offset;
}

void sx127x_fsk_set_afc(sx127x_t *dev, bool value)
{
    _set_flag(dev, SX127X_AFC_ON_FLAG, value);
}

void sx127x_fsk_set_preamble_detect(sx127x_t *dev, uint8_t value)
{
    DEBUG("[sx127x] Setting preamble detect: 0x%02X\n", value >> 7);

    dev->settings.fsk.preamble_detect = (sx127x_reg_read(dev, SX127X_REG_PREAMBLEDETECT) &
                                         SX127X_RF_PREAMBLEDETECT_DETECTOR_MASK) |
                                         value;
    sx127x_reg_write(dev, SX127X_REG_PREAMBLEDETECT, dev->settings.fsk.preamble_detect);
}

void sx127x_fsk_set_preamble_detector_size(sx127x_t *dev, uint8_t size)
{
    DEBUG("[sx127x] Setting preamble detector size: 0x%02X\n", size >> 5);

    dev->settings.fsk.preamble_detect = (sx127x_reg_read(dev, SX127X_REG_PREAMBLEDETECT) &
                                             SX127X_RF_PREAMBLEDETECT_DETECTORSIZE_MASK) |
                                             size;

    sx127x_reg_write(dev, SX127X_REG_PREAMBLEDETECT, dev->settings.fsk.preamble_detect);
}

void sx127x_fsk_set_preamble_detector_tol(sx127x_t *dev, uint8_t value)
{
    DEBUG("[sx127x] Setting preamble detector tolerance: 0x%02X\n", value);

        dev->settings.fsk.preamble_detect = (sx127x_reg_read(dev, SX127X_REG_PREAMBLEDETECT) &
                                             SX127X_RF_PREAMBLEDETECT_DETECTORTOL_MASK) |
                                             value;
        sx127x_reg_write(dev, SX127X_REG_PREAMBLEDETECT, dev->settings.fsk.preamble_detect);
}
