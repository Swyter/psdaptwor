/**
 * | psdaptwor - an experimental PSVR2 to PC adaptor | firmware
 * | created by Swyter <swyterzone+psdaptwor@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "fusb302_defines.h"

/*      Main microcontroller chip: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
                                   https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf

        Type-C PD controller chip: https://www.onsemi.com/pdf/datasheet/fusb302b-d.pdf
                                   https://github.com/Ralim/usb-pd/
                                   https://www.ti.com/lit/an/slva704/slva704.pdf (good overview of how the I2C protocol works)
                                   https://github.com/ReclaimerLabs/FUSB302/blob/master/FUSB302.cpp (The Google ChromeOS EC implementation; very clean and well-commented)

   M-LVDS to TTL transceiver chip: http://www.ti.com/lit/gpn/SN65MLVD200 */

enum psdaptwor_pins
{
    PSDAPT_PIN_HMD_TYC_SDA           =  0, /* swy: I2C connection to FUSB302B Type-C Power Delivery control chip */
    PSDAPT_PIN_HMD_TYC_SCL           =  1,
    PSDAPT_PIN_HMD_TYC_INT           =  2,
          
    PSDAPT_PIN_PC_LVDS_R             =  3, /* swy: M-LVDS transceiver data in-out for both DisplayPort AUX sides */
    PSDAPT_PIN_PC_LVDS_RE            =  4,
    PSDAPT_PIN_PC_LVDS_DR            =  5,
    PSDAPT_PIN_PC_LVDS_D             =  6,
          
    PSDAPT_PIN_HMD_LVDS_R            =  7,
    PSDAPT_PIN_HMD_LVDS_RE           =  8,
    PSDAPT_PIN_HMD_LVDS_DR           =  9,
    PSDAPT_PIN_HMD_LVDS_D            = 10,

    /* -- */

    PSDAPT_PIN_PC_HPD                = 11, /* swy: DisplayPort hot-plug detection pulse signal */
//  PSDAPT_PIN_12_UNUSED             = 12,
//  PSDAPT_PIN_13_UNUSED             = 13,

    PSDAPT_PIN_LED_CONN_WRONG_ORIENT = 14, /* swy: status LEDS; wrong-type-c-orientation is orange */
    PSDAPT_PIN_LED_HMD_IS_READY      = 15, /*                           headset-is-ready is green  */

    PSDAPT_PIN_HMD_ENABLE_VBUS_12V   = 16, /* swy: control the power switches by software as needed to enable */
    PSDAPT_PIN_HMD_ENABLE_VBUS       = 17, /*      the higher 12V DC jack voltage/5V from PC/off              */

    PSDAPT_PIN_HMD_USB2_P            = 18, /* swy: connected to the headset's USB 2.0 +/- pair, optional, in case we want to */
    PSDAPT_PIN_HMD_USB2_N            = 19, /*      relay it to the host PC via PIO, or inspect the billboard device          */
//  PSDAPT_PIN_20_UNUSED             = 20,
//  PSDAPT_PIN_21_UNUSED             = 21,

    PSDAPT_PIN_AUX_TERMINATION_N     = 22, /* swy: M-LVDS transceiver software-defined pull-up and pull-down resistors */
    PSDAPT_PIN_AUX_TERMINATION_P     = 23,
    PSDAPT_PIN_PC_AUX_TERMINATION_N  = 24,
    PSDAPT_PIN_PC_AUX_TERMINATION_P  = 25,

    PSDAPT_PIN_DC_JACK_VOLT_SENSE    = 26, /* swy: analog-to-digital converter pins */
//  PSDAPT_PIN_27_UNUSED             = 27,
    PSDAPT_PIN_ADC_OPT_A             = 28,
    PSDAPT_PIN_ADC_OPT_B             = 29,
};

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
 

/* swy: I2C register reading/writing functions from: https://forums.raspberrypi.com/viewtopic.php?p=2040653#p2040598 */
// Read bytes from a register
int i2c_read(
    i2c_inst_t *i2c,   // i2c0 or i2c1
    uint8_t i2c_addr,  // 7 bit I2C address
    uint8_t reg,       // Number of the register to read from
    uint8_t *rx_data,  // Pointer to block of bytes for data read
    uint8_t len        // Number of bytes of data to read
) {
    if (i2c_write_blocking(i2c, i2c_addr, &reg, 1, true) != 1) {
        return PICO_ERROR_GENERIC;
    }

    if (i2c_read_blocking(i2c, i2c_addr, rx_data, len, false) != len) {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int i2c_read_byte(
    i2c_inst_t *i2c,   // i2c0 or i2c1
    uint8_t i2c_addr,  // 7 bit I2C address
    uint8_t reg,       // Number of the register to read from
    uint8_t *rx_data   // Pointer to block of bytes for data read
) {
    return i2c_read(i2c, i2c_addr, reg, rx_data, 1);
}

// Write bytes to a register
int i2c_write(
    i2c_inst_t *i2c,         // i2c0 or i2c1
    uint8_t i2c_addr,        // 7 bit I2C address 
    uint8_t reg,             // Number of the register to write to
    const uint8_t *tx_data,  // Pointer to block of bytes to write
    uint8_t len              // Length of block of bytes to write
) {
    uint8_t tx_len = len + 1;
    uint8_t tx_buf[tx_len];
    tx_buf[0]  = reg;
    memcpy(&tx_buf[1], tx_data, len);

    if (i2c_write_blocking(i2c, i2c_addr, tx_buf, tx_len, false) != tx_len) {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int i2c_write_byte(
    i2c_inst_t *i2c,         // i2c0 or i2c1
    uint8_t i2c_addr,        // 7 bit I2C address 
    uint8_t reg,             // Number of the register to write to
    uint8_t tx_byte          // Byte to write
) {
    uint8_t tx_buf = tx_byte; return i2c_write(i2c, i2c_addr, reg, &tx_buf, 1);
}


char *fusb_debug_register(uint8_t reg, uint8_t reg_data)
{
    uint8_t reg_data_orig = reg_data;

#define CASE_PRINT(_REG) \
    case _REG: printf("[%s]: ", &#_REG[5]);

#define PRINT_REG(ARG_PREFIX, ARG_NAME) /* swy: if the flag is set, print it, to see if we need to print a | separator make sure the current flag is set and there's any set data underneath that will be processed afterwards, or we'll just print the number */ \
    printf("%s%s", (reg_data & ARG_PREFIX##_##ARG_NAME) ? #ARG_NAME : "", (reg_data & ARG_PREFIX##_##ARG_NAME) && (reg_data & (ARG_PREFIX##_##ARG_NAME - 1)) ? "|" : ""); reg_data &= ~ARG_PREFIX##_##ARG_NAME;

#define CASE_END() \
    if (reg_data != 0 || reg_data_orig==reg_data) printf("%#x, ", reg_data); /* swy: if there's any remaining unparsed data, or no flags have been processed, i.e. it's zero, print the number */ \
                                             else printf(  ", "); \
    break;

    switch (reg)
    {
        default:
            printf("[??? Unknown register %X], ", reg);
            break;

        CASE_PRINT(FUSB_SWITCHES0) {
            PRINT_REG(FUSB_SWITCHES0, CC2_PU_EN)
            PRINT_REG(FUSB_SWITCHES0, CC1_PU_EN)
            PRINT_REG(FUSB_SWITCHES0, VCONN_CC2)
            PRINT_REG(FUSB_SWITCHES0, VCONN_CC1)
            PRINT_REG(FUSB_SWITCHES0, MEAS_CC2 )
            PRINT_REG(FUSB_SWITCHES0, MEAS_CC1 )
            PRINT_REG(FUSB_SWITCHES0, CC2_PD_EN)
            PRINT_REG(FUSB_SWITCHES0, CC1_PD_EN)
            CASE_END()
        }

        CASE_PRINT(FUSB_SWITCHES1) {
            PRINT_REG(FUSB_SWITCHES1, POWERROLE)
            PRINT_REG(FUSB_SWITCHES1, SPECREV1 )
            PRINT_REG(FUSB_SWITCHES1, SPECREV0 )
            PRINT_REG(FUSB_SWITCHES1, DATAROLE )
            PRINT_REG(FUSB_SWITCHES1, AUTO_CRC )
            PRINT_REG(FUSB_SWITCHES1, TXCC2_EN )
            PRINT_REG(FUSB_SWITCHES1, TXCC1_EN )
            CASE_END()
        }

        CASE_PRINT(FUSB_CONTROL0) {
            PRINT_REG(FUSB_CONTROL0, TX_FLUSH )
            PRINT_REG(FUSB_CONTROL0, INT_MASK )
            PRINT_REG(FUSB_CONTROL0, HOST_CUR1)
            PRINT_REG(FUSB_CONTROL0, HOST_CUR0)
            PRINT_REG(FUSB_CONTROL0, AUTO_PRE )
            PRINT_REG(FUSB_CONTROL0, TX_START )
            CASE_END()
        }

        CASE_PRINT(FUSB_CONTROL1) {
            PRINT_REG(FUSB_CONTROL1, ENSOP2DB  )
            PRINT_REG(FUSB_CONTROL1, ENSOP1DB  )
            PRINT_REG(FUSB_CONTROL1, BIST_MODE2)
            PRINT_REG(FUSB_CONTROL1, RX_FLUSH  )
            PRINT_REG(FUSB_CONTROL1, ENSOP2    )
            PRINT_REG(FUSB_CONTROL1, ENSOP1    )
            CASE_END()
        }

        CASE_PRINT(FUSB_CONTROL2) {
            PRINT_REG(FUSB_CONTROL2, TOG_SAVE_PWR1)
            PRINT_REG(FUSB_CONTROL2, TOG_SAVE_PWR0)
            PRINT_REG(FUSB_CONTROL2, TOG_RD_ONLY  )
            PRINT_REG(FUSB_CONTROL2, WAKE_EN      )
            PRINT_REG(FUSB_CONTROL2, MODE1        )
            PRINT_REG(FUSB_CONTROL2, MODE0        )
            PRINT_REG(FUSB_CONTROL2, TOGGLE       )
            CASE_END()
        }

        CASE_PRINT(FUSB_CONTROL3) {
            PRINT_REG(FUSB_CONTROL3, SEND_HARD_RESET)
            PRINT_REG(FUSB_CONTROL3, BIST_TMODE     )
            PRINT_REG(FUSB_CONTROL3, AUTO_HARDRESET )
            PRINT_REG(FUSB_CONTROL3, AUTO_SOFTRESET )
            PRINT_REG(FUSB_CONTROL3, N_RETRIES1     )
            PRINT_REG(FUSB_CONTROL3, N_RETRIES0     )
            PRINT_REG(FUSB_CONTROL3, AUTO_RETRY     )
            CASE_END()
        }

        CASE_PRINT(FUSB_POWER) {
            PRINT_REG(FUSB_POWER, PWR3)
            PRINT_REG(FUSB_POWER, PWR2)
            PRINT_REG(FUSB_POWER, PWR1)
            PRINT_REG(FUSB_POWER, PWR0)
            CASE_END()
        }

        CASE_PRINT(FUSB_STATUS0A) {
            PRINT_REG(FUSB_STATUS0A, SOFTFAIL )
            PRINT_REG(FUSB_STATUS0A, RETRYFAIL)
            PRINT_REG(FUSB_STATUS0A, POWER3   )
            PRINT_REG(FUSB_STATUS0A, POWER2   )
            PRINT_REG(FUSB_STATUS0A, SOFTRST  )
            PRINT_REG(FUSB_STATUS0A, HARDRST  )
            CASE_END()
        }

        CASE_PRINT(FUSB_STATUS1A) {
            PRINT_REG(FUSB_STATUS1A, TOGSS_SHIFT)
            PRINT_REG(FUSB_STATUS1A, TOGSS      )
            PRINT_REG(FUSB_STATUS1A, RXSOP2DB   )
            PRINT_REG(FUSB_STATUS1A, RXSOP1DB   )
            PRINT_REG(FUSB_STATUS1A, RXSOP      )
            CASE_END()
        }

        CASE_PRINT(FUSB_INTERRUPTA) {
            PRINT_REG(FUSB_INTERRUPTA, I_OCP_TEMP )
            PRINT_REG(FUSB_INTERRUPTA, I_TOGDONE  )
            PRINT_REG(FUSB_INTERRUPTA, I_SOFTFAIL )
            PRINT_REG(FUSB_INTERRUPTA, I_RETRYFAIL)
            PRINT_REG(FUSB_INTERRUPTA, I_HARDSENT )
            PRINT_REG(FUSB_INTERRUPTA, I_TXSENT   )
            PRINT_REG(FUSB_INTERRUPTA, I_SOFTRST  )
            PRINT_REG(FUSB_INTERRUPTA, I_HARDRST  )
            CASE_END()
        }

        CASE_PRINT(FUSB_INTERRUPTB) {
            PRINT_REG(FUSB_INTERRUPTB, I_GCRCSENT)
            CASE_END()
        }

        CASE_PRINT(FUSB_STATUS0) {
            PRINT_REG(FUSB_STATUS0, VBUSOK  )
            PRINT_REG(FUSB_STATUS0, ACTIVITY)
            PRINT_REG(FUSB_STATUS0, COMP    )
            PRINT_REG(FUSB_STATUS0, CRC_CHK )
            PRINT_REG(FUSB_STATUS0, ALERT   )
            PRINT_REG(FUSB_STATUS0, WAKE    )
            PRINT_REG(FUSB_STATUS0, BC_LVL1 )
            PRINT_REG(FUSB_STATUS0, BC_LVL0 )
            CASE_END()
        }

        CASE_PRINT(FUSB_INTERRUPT) {
            PRINT_REG(FUSB_INTERRUPT, I_VBUSOK   )
            PRINT_REG(FUSB_INTERRUPT, I_ACTIVITY )
            PRINT_REG(FUSB_INTERRUPT, I_COMP_CHNG)
            PRINT_REG(FUSB_INTERRUPT, I_CRC_CHK  )
            PRINT_REG(FUSB_INTERRUPT, I_ALERT    )
            PRINT_REG(FUSB_INTERRUPT, I_WAKE     )
            PRINT_REG(FUSB_INTERRUPT, I_COLLISION)
            PRINT_REG(FUSB_INTERRUPT, I_BC_LVL   )
            CASE_END()
        }

    }
#undef CASE_END
#undef PRINT_REG
#undef CASE_PRINT
}

void fusb_interrupt_callback(uint gpio, uint32_t event_mask)
{
    printf("[i] USB-C controller interrupt request: %x %x\b", gpio, event_mask); /* swy: clear interrupt registers by reading them */
    uint8_t rxdata; //i2c_read(i2c_default, FUSB302B_ADDR, FUSB_INTERRUPTA, &rxdata, 1); fusb_debug_register(FUSB_INTERRUPTA, rxdata); stdio_flush();
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_INTERRUPTB, &rxdata, 1); fusb_debug_register(FUSB_INTERRUPTB, rxdata); stdio_flush();
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_INTERRUPT,  &rxdata, 1); fusb_debug_register(FUSB_INTERRUPT,  rxdata);
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0,    &rxdata, 1); fusb_debug_register(FUSB_STATUS0,    rxdata);
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0A,   &rxdata, 1); fusb_debug_register(FUSB_STATUS0A,   rxdata);
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0,  &rxdata, 1); fusb_debug_register(FUSB_SWITCHES0,  rxdata);
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1,  &rxdata, 1); fusb_debug_register(FUSB_SWITCHES1,  rxdata);
                    //i2c_read(i2c_default, FUSB302B_ADDR, FUSB_POWER,      &rxdata, 1); fusb_debug_register(FUSB_POWER,      rxdata); stdio_flush();
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_CONTROL0,   &rxdata, 1); fusb_debug_register(FUSB_CONTROL0,   rxdata); puts(NULL); stdio_flush();
                    //i2c_read(i2c_default, FUSB302B_ADDR, FUSB_CONTROL1,   &rxdata, 1); fusb_debug_register(FUSB_CONTROL1,   rxdata); stdio_flush();
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_CONTROL2,   &rxdata, 1); fusb_debug_register(FUSB_CONTROL2,   rxdata); stdio_flush();
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_CONTROL3,   &rxdata, 1); fusb_debug_register(FUSB_CONTROL3,   rxdata); stdio_flush();

    //printf("[i] ---\n");

    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_FIFOS, &rxdata, 1); printf("FUSB_FIFOS rxdata: %#x\n", rxdata);
    return;
}


int fusb_measure_against(int mdac_val)
{
    uint8_t rxdata, ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, FUSB_MEASURE_MEAS_VBUS | (mdac_val & 0b111111)); //printf("f write ret: %#x, counter: %x, comp volt: %f\n", ret, mdac_val, mdac_val * 0.420f);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &rxdata, 1); //printf("i2c_read FUSB_STATUS0: %#x COMP: %u\n", rxdata, (rxdata & FUSB_STATUS0_COMP));
    return (rxdata & FUSB_STATUS0_COMP);
}

int fusb_measure_vbus_bsearch_comp(const void *a, const void *b)
{
    long unsigned int index = (long unsigned int) b;

    if ((fusb_measure_against(index) & FUSB_STATUS0_COMP) && index < (FUSB_MEASURE_MDAC + 1))
        return (fusb_measure_against(index + 1) == 0) ? 0 : +1;

    return -1;
}

float fusb_measure_vbus(void)
{
    uint8_t counter = 0, lastrxdata = 0, rxdata = 0, ret = 0;

    uint8_t switchesBackup; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &switchesBackup, 1); //printf("b read FUSB_SWITCHES0: %#x\n", switchesBackup);
    uint8_t measureBackup;  i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &measureBackup,  1); //printf("b read FUSB_MEASURE: %#x\n", measureBackup);

    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, switchesBackup & 0b11110011); //printf("f write ret: %#x\n", ret);

    void *bsearch_ret = bsearch(NULL, NULL, FUSB_MEASURE_MDAC + 1, 1, fusb_measure_vbus_bsearch_comp); counter = (uint32_t) bsearch_ret;

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   measureBackup);
    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, switchesBackup);

    return counter * 0.420f;
}

int fusb_compare_cc1_and_cc2(void)
{
    uint8_t counter = 0, lastrxdata = 0, rxdata = 0, ret = 0, cc1, cc2;
    uint8_t switchesBackup; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &switchesBackup, 1); printf("b read FUSB_SWITCHES0: %#x\n", switchesBackup);
    uint8_t measureBackup;  i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &measureBackup,  1); printf("b read FUSB_MEASURE: %#x\n", measureBackup);

    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, FUSB_SWITCHES0_CC1_PD_EN | FUSB_SWITCHES0_CC2_PD_EN | FUSB_SWITCHES0_MEAS_CC1); printf("cc1 write ret: %#x\n", ret);
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, 0); printf("cc1 write ret: %#x\n", ret);
    sleep_ms(10);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &rxdata, 1); printf("cc1 i2c_read FUSB_STATUS0: %#x BC_LVL: %u\n", rxdata, (cc1 = rxdata & FUSB_STATUS0_BC_LVL));

    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, FUSB_SWITCHES0_CC1_PD_EN | FUSB_SWITCHES0_CC2_PD_EN | FUSB_SWITCHES0_MEAS_CC2); printf("cc2 write ret: %#x\n", ret);
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, 0); printf("cc2 write ret: %#x\n", ret);
    sleep_ms(10);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &rxdata, 1); printf("cc2 i2c_read FUSB_STATUS0: %#x BC_LVL: %u\n", rxdata, (cc2 = rxdata & FUSB_STATUS0_BC_LVL));


    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   measureBackup);
    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, switchesBackup);

    return cc1 > cc2;
}


enum tcpc_cc_pull {
    TYPEC_CC_RA = 0,
    TYPEC_CC_RP = 1,
    TYPEC_CC_RD = 2,
    TYPEC_CC_OPEN = 3,
};

enum tcpc_rp_value {
    TYPEC_RP_USB = 0,
    TYPEC_RP_1A5 = 1,
    TYPEC_RP_3A0 = 2,
    TYPEC_RP_RESERVED = 3,
};

int cc_polarity = -1;
int previous_pull = TYPEC_CC_RD;
bool vconn_enabled = false;
int dfp_toggling_on = 0;
int pulling_up = 0;

int togdone_pullup_cc1 = 0;
int togdone_pullup_cc2 = 0;


int set_polarity(int polarity)
{
    /* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
    uint8_t reg;

    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);

    /* clear VCONN switch bits */
    reg &= ~FUSB_SWITCHES0_VCONN_CC1;
    reg &= ~FUSB_SWITCHES0_VCONN_CC2;

    if (vconn_enabled) {
        /* set VCONN switch to be non-CC line */
        if (polarity)
            reg |= FUSB_SWITCHES0_VCONN_CC1;
        else
            reg |= FUSB_SWITCHES0_VCONN_CC2;
    }

    /* clear meas_cc bits (RX line select) */
    reg &= ~FUSB_SWITCHES0_MEAS_CC1;
    reg &= ~FUSB_SWITCHES0_MEAS_CC2;

    /* set rx polarity */
    if (polarity)
        reg |= FUSB_SWITCHES0_MEAS_CC2;
    else
        reg |= FUSB_SWITCHES0_MEAS_CC1;

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);

    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1, &reg);

    /* clear tx_cc bits */
    reg &= ~FUSB_SWITCHES1_TXCC1_EN;
    reg &= ~FUSB_SWITCHES1_TXCC2_EN;

    /* set tx polarity */
    if (polarity)
        reg |= FUSB_SWITCHES1_TXCC2_EN;
    else
        reg |= FUSB_SWITCHES1_TXCC1_EN;

    // Enable auto GoodCRC sending
    reg |= FUSB_SWITCHES1_AUTO_CRC;

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1, reg);

    /* Save the polarity for later */
    cc_polarity = polarity;

    return 0;
}

int set_vconn(int enable)
{
    /*
     * FUSB302 does not have dedicated VCONN Enable switch.
     * We'll get through this by disabling both of the
     * VCONN - CC* switches to disable, and enabling the
     * saved polarity when enabling.
     * Therefore at startup, set_polarity should be called first,
     * or else live with the default put into init.
     */
    uint8_t reg;

    /* save enable state for later use */
    vconn_enabled = enable;

    if (enable) {
        /* set to saved polarity */
        set_polarity(cc_polarity);
    } else {

        i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);

        /* clear VCONN switch bits */
        reg &= ~FUSB_SWITCHES0_VCONN_CC1;
        reg &= ~FUSB_SWITCHES0_VCONN_CC2;

        i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);
    }

    return 0;
}

int set_cc(int pull)
{
    uint8_t reg;

    /*
     * Ensure we aren't in the process of changing CC from the alert
     * handler, then cancel any pending toggle-triggered CC change.
     */
    dfp_toggling_on = 0;
    previous_pull = pull;

    /* NOTE: FUSB302 toggles a single pull-up between CC1 and CC2 */
    /* NOTE: FUSB302 Does not support Ra. */
    switch (pull) {
        case TYPEC_CC_RP:
            /* enable the pull-up we know to be necessary */
            i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);

            reg &= ~(FUSB_SWITCHES0_CC1_PU_EN |
                 FUSB_SWITCHES0_CC2_PU_EN |
                 FUSB_SWITCHES0_CC1_PD_EN |
                 FUSB_SWITCHES0_CC2_PD_EN |
                 FUSB_SWITCHES0_VCONN_CC1 |
                 FUSB_SWITCHES0_VCONN_CC2);

            reg |= FUSB_SWITCHES0_CC1_PU_EN |
                FUSB_SWITCHES0_CC2_PU_EN;

            if (vconn_enabled)
                reg |= togdone_pullup_cc1 ?
                       FUSB_SWITCHES0_VCONN_CC2 :
                       FUSB_SWITCHES0_VCONN_CC1;

            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);

            pulling_up = 1;
            dfp_toggling_on = 0;
            break;
        case TYPEC_CC_RD:
            /* Enable UFP Mode */

            /* turn off toggle */
            i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL2, &reg);
            reg &= ~FUSB_CONTROL2_TOGGLE;
            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL2, reg);

            /* enable pull-downs, disable pullups */
            i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);
            reg &= ~(FUSB_SWITCHES0_CC1_PU_EN);
            reg &= ~(FUSB_SWITCHES0_CC2_PU_EN);
            reg |=  (FUSB_SWITCHES0_CC1_PD_EN);
            reg |=  (FUSB_SWITCHES0_CC2_PD_EN);
            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);

            pulling_up = 0;
            dfp_toggling_on = 0;
            break;
        case TYPEC_CC_OPEN:
            /* Disable toggling */
            i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL2, &reg);
            reg &= ~FUSB_CONTROL2_TOGGLE;
            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL2, reg);

            /* Ensure manual switches are opened */
            i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);
            reg &= ~FUSB_SWITCHES0_CC1_PU_EN;
            reg &= ~FUSB_SWITCHES0_CC2_PU_EN;
            reg &= ~FUSB_SWITCHES0_CC1_PD_EN;
            reg &= ~FUSB_SWITCHES0_CC2_PD_EN;
            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);

            pulling_up = 0;
            dfp_toggling_on = 0;
            break;
        default:
            /* Unsupported... */
            return 0;
    }
    return 0;
}

int init()
{
    uint8_t reg;

    /* set default */
    cc_polarity = -1;
    previous_pull = TYPEC_CC_RD;

    /* set the voltage threshold for no connect detection (vOpen) */
    //this->mdac_vnc = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_VNC_MV);
    /* set the voltage threshold for Rd vs Ra detection */
    //this->mdac_rd = TCPC_REG_MEASURE_MDAC_MV(PD_SRC_DEF_RD_THRESH_MV);

    /* Restore default settings */
    uint32_t ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_RESET, FUSB_RESET_SW_RESET); printf("b write ret: %#x\n", ret);

    sleep_ms(1);

    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_DEVICE_ID, &reg, 1); printf("b read 0: %#x\n", reg);
    printf("revision ID: %#x, Product ID: %#x, Revision ID: %#x\n", reg >> FUSB_DEVICE_ID_VERSION_ID_SHIFT,  (reg & 0b1100) >> FUSB_DEVICE_ID_PRODUCT_ID_SHIFT, (reg & 0b0011) >> FUSB_DEVICE_ID_REVISION_ID_SHIFT);

    /* Turn on retries and set number of retries */
    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL3, &reg);
    reg |= FUSB_CONTROL3_AUTO_RETRY;
    reg |= FUSB_CONTROL3_N_RETRIES1 | FUSB_CONTROL3_N_RETRIES0;
    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL3, reg);

    /* Set interrupt masks */
    // Setting to 0 so interrupts are allowed
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MASK1, 0); printf("c write ret: %#x\n", ret);
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MASKA, 0); printf("d write ret: %#x\n", ret);
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MASKB, 0); printf("e write ret: %#x\n", ret);

    /* Set VCONN switch defaults */
    set_polarity(0);
    set_vconn(0);

    /* Turn on all power */
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_POWER, FUSB_POWER_PWR0 | FUSB_POWER_PWR1 | FUSB_POWER_PWR2 | FUSB_POWER_PWR3); printf("b write ret: %#x\n", ret);
    
    return 0;
}

/* bring the FUSB302 out of reset after Hard Reset signaling */
void pd_reset()
{
    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_RESET, FUSB_RESET_PD_RESET);
}

enum tcpc_cc_voltage_status {
    TYPEC_CC_VOLT_OPEN = 0,
    TYPEC_CC_VOLT_RA = 1,
    TYPEC_CC_VOLT_RD = 2,
    TYPEC_CC_VOLT_SNK_DEF = 5,
    TYPEC_CC_VOLT_SNK_1_5 = 6,
    TYPEC_CC_VOLT_SNK_3_0 = 7,
};

/* Convert BC LVL values (in FUSB302) to Type-C CC Voltage Status */
uint8_t convert_bc_lvl(uint8_t bc_lvl)
{
    /* assume OPEN unless one of the following conditions is true... */
    int ret = TYPEC_CC_VOLT_OPEN;

    if (pulling_up) {
        if (bc_lvl == 0x00)
            ret = TYPEC_CC_VOLT_RA;
        else if (bc_lvl < 0x3)
            ret = TYPEC_CC_VOLT_RD;
    } else {
        if (bc_lvl == 0x1)
            ret = TYPEC_CC_VOLT_SNK_DEF;
        else if (bc_lvl == 0x2)
            ret = TYPEC_CC_VOLT_SNK_1_5;
        else if (bc_lvl == 0x3)
            ret = TYPEC_CC_VOLT_SNK_3_0;
    }

    return ret;
}


/* Determine cc pin state for sink */
void detect_cc_pin_sink(int *cc1, int *cc2)
{
    uint8_t reg;
    int orig_meas_cc1;
    int orig_meas_cc2;
    uint8_t bc_lvl_cc1;
    uint8_t bc_lvl_cc2;

    /*
     * Measure CC1 first.
     */
    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);

    /* save original state to be returned to later... */
    if (reg & FUSB_SWITCHES0_MEAS_CC1)
        orig_meas_cc1 = 1;
    else
        orig_meas_cc1 = 0;

    if (reg & FUSB_SWITCHES0_MEAS_CC2)
        orig_meas_cc2 = 1;
    else
        orig_meas_cc2 = 0;

    /* Disable CC2 measurement switch, enable CC1 measurement switch */
    reg &= ~FUSB_SWITCHES0_MEAS_CC2;
    reg |= FUSB_SWITCHES0_MEAS_CC1;

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);

    /* CC1 is now being measured by FUSB302. */

    /* Wait on measurement */
    sleep_us(250);

    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &bc_lvl_cc1);

    /* mask away unwanted bits */
    bc_lvl_cc1 &= (FUSB_STATUS0_BC_LVL0 | FUSB_STATUS0_BC_LVL1);

    /*
     * Measure CC2 next.
     */

    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);

    /* Disable CC1 measurement switch, enable CC2 measurement switch */
    reg &= ~FUSB_SWITCHES0_MEAS_CC1;
    reg |= FUSB_SWITCHES0_MEAS_CC2;

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);

    /* CC2 is now being measured by FUSB302. */

    /* Wait on measurement */
    busy_wait_us(250);

    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &bc_lvl_cc2);

    /* mask away unwanted bits */
    bc_lvl_cc2 &= (FUSB_STATUS0_BC_LVL0 | FUSB_STATUS0_BC_LVL1);

    *cc1 = convert_bc_lvl(bc_lvl_cc1);
    *cc2 = convert_bc_lvl(bc_lvl_cc2);

    /* return MEAS_CC1/2 switches to original state */
    i2c_read_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg);
    if (orig_meas_cc1)
        reg |= FUSB_SWITCHES0_MEAS_CC1;
    else
        reg &= ~FUSB_SWITCHES0_MEAS_CC1;
    if (orig_meas_cc2)
        reg |= FUSB_SWITCHES0_MEAS_CC2;
    else
        reg &= ~FUSB_SWITCHES0_MEAS_CC2;

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg);
}

int main() {
    stdio_init_all();
    stdio_usb_init();
    stdio_usb_connected();
    printf("ADC Example, measuring GPIO26\n");

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init   (PSDAPT_PIN_LED_CONN_WRONG_ORIENT);
    gpio_set_dir(PSDAPT_PIN_LED_CONN_WRONG_ORIENT, GPIO_OUT);
    gpio_put    (PSDAPT_PIN_LED_CONN_WRONG_ORIENT, 1);

    gpio_init   (PSDAPT_PIN_LED_HMD_IS_READY);
    gpio_set_dir(PSDAPT_PIN_LED_HMD_IS_READY, GPIO_OUT);
    gpio_put    (PSDAPT_PIN_LED_HMD_IS_READY, 1);
#endif

    gpio_init   (PSDAPT_PIN_HMD_ENABLE_VBUS);
    gpio_set_dir(PSDAPT_PIN_HMD_ENABLE_VBUS, GPIO_OUT);
    gpio_put    (PSDAPT_PIN_HMD_ENABLE_VBUS,     0);

    gpio_init   (PSDAPT_PIN_HMD_ENABLE_VBUS_12V);
    gpio_set_dir(PSDAPT_PIN_HMD_ENABLE_VBUS_12V, GPIO_OUT);
    gpio_put    (PSDAPT_PIN_HMD_ENABLE_VBUS_12V, 0);

    gpio_init   (PSDAPT_PIN_PC_HPD);
    gpio_set_dir(PSDAPT_PIN_PC_HPD, GPIO_OUT);
    gpio_put    (PSDAPT_PIN_PC_HPD, 0);

    sleep_ms(3500);    

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);


    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    gpio_set_irq_enabled_with_callback(PSDAPT_PIN_HMD_TYC_INT, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, fusb_interrupt_callback);


    printf("\nI2C Bus Scan test\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
 
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
 
        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.
 
        // Skip over any reserved addresses.
        int ret = 0;
        uint8_t rxdata;
        absolute_time_t timeout = make_timeout_time_ms(50);

        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking_until(i2c_default, addr, &rxdata, 1, false, timeout);
 
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    printf("Done, for real. :-)\n");

    uint8_t rxdata, reg;

    init();
    pd_reset();

    int cc1_meas, cc2_meas;
    detect_cc_pin_sink(&cc1_meas, &cc2_meas);

    printf("CC1 level = ");
    switch (cc1_meas) {
    case TYPEC_CC_VOLT_OPEN:
        printf("Open");
        break;
    case TYPEC_CC_VOLT_RA:
        printf("Ra pull-down");
        break;
    case TYPEC_CC_VOLT_RD:
        printf("Rd pull-down");
        break;
    case TYPEC_CC_VOLT_SNK_DEF:
        printf("Connected with default power");
        break;
    case TYPEC_CC_VOLT_SNK_1_5:
        printf("Connected with 1.5A at 5V");
        break;
    case TYPEC_CC_VOLT_SNK_3_0:
        printf("Connected with 3.0A at 5V");
        break;
    default :
        printf("Unknown");
        break;
    }
    puts(NULL);

    printf("CC2 level = ");
    switch (cc2_meas) {
    case TYPEC_CC_VOLT_OPEN:
        printf("Open");
        break;
    case TYPEC_CC_VOLT_RA:
        printf("Ra pull-down");
        break;
    case TYPEC_CC_VOLT_RD:
        printf("Rd pull-down");
        break;
    case TYPEC_CC_VOLT_SNK_DEF:
        printf("Connected with default power");
        break;
    case TYPEC_CC_VOLT_SNK_1_5:
        printf("Connected with 1.5A at 5V");
        break;
    case TYPEC_CC_VOLT_SNK_3_0:
        printf("Connected with 3.0A at 5V");
        break;
    default :
        printf("Unknown");
        break;
    }
    puts(NULL);

    if (cc1_meas > cc2_meas) {
        set_polarity(0); printf("set_polarity(0)\n");
    } else {
        set_polarity(1); printf("set_polarity(1)\n");
    }

    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &rxdata, 1); printf("b read FUSB_SWITCHES0: %#x ", rxdata); fusb_debug_register(FUSB_SWITCHES1, rxdata); puts(NULL);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1, &rxdata, 1); printf("b read FUSB_SWITCHES1: %#x ", rxdata); fusb_debug_register(FUSB_SWITCHES1, rxdata); puts(NULL);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &rxdata, 1); printf("b read FUSB_MEASURE: %#x\n", rxdata);

    uint8_t switchesBackup; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &switchesBackup, 1); printf("b read FUSB_SWITCHES0: %#x ", switchesBackup); fusb_debug_register(FUSB_SWITCHES0, switchesBackup); puts(NULL);
    uint8_t measureBackup;  i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &measureBackup,  1); printf("b read FUSB_MEASURE: %#x\n", measureBackup);
    uint8_t counter = 0, cc1 = 0, cc2 = 0, cc1_is_bigger_than_cc2 = 0; float measured_vbus = 0.f; bool cc_tx_configured = 0;



    uint8_t buf[32] = {0x69};

    while (1) {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("[adc] Raw value: 0x%03x, measured voltage: %f V, actual pre-divided voltage: %f V\n", result, result * conversion_factor, result * (24.f / (1 << 12)));

        //measured_vbus = fusb_measure_vbus();
        //cc1_is_bigger_than_cc2 = fusb_compare_cc1_and_cc2();

        //printf("[fusb] measured USB-C VBUS: %f, CC1 > CC2: %i\n", measured_vbus, cc1_is_bigger_than_cc2);

        if (0) //(!cc_tx_configured)//measured_vbus > 3 && !cc_tx_configured)
        {
            uint8_t reg; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &reg, 1); fusb_debug_register(FUSB_SWITCHES0, reg); puts(NULL);
            /* Clear CC1/CC2 measure bits */
            reg &= ~FUSB_SWITCHES0_MEAS_CC1;
            reg &= ~FUSB_SWITCHES0_MEAS_CC2;

            if (1) reg |= FUSB_SWITCHES0_MEAS_CC1;
            else   reg |= FUSB_SWITCHES0_MEAS_CC2;

            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, reg); printf("cc1_is_bigger_than_cc2 cc1 write FUSB_SWITCHES1:"); fusb_debug_register(FUSB_SWITCHES1, reg); puts(NULL);

            i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1, &reg, 1); 
            reg |= FUSB_SWITCHES1_AUTO_CRC;
            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1, reg); printf("cc1_is_bigger_than_cc2 cc1 write FUSB_SWITCHES1:"); fusb_debug_register(FUSB_SWITCHES1, reg); puts(NULL);

            /* Flush the RX buffer */
            i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH); printf("f write FUSB_CONTROL1:\n");
            cc_tx_configured = 1;
        }


        gpio_put(PSDAPT_PIN_HMD_ENABLE_VBUS,     0);
        gpio_put(PSDAPT_PIN_HMD_ENABLE_VBUS_12V, 0);

        sleep_ms(500);


        
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_FIFOS, &buf[0], 1);
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_FIFOS, &buf[1], 1);
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_FIFOS, &buf[2], 1);
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_FIFOS, &buf[3], 1);  printf("b read FUSB_FIFOS: %#x %#x %#x %#x\n", buf[0], buf[1], buf[2], buf[3]);

        //printf("\nGPIO 0: %u 1: %u 2: %u 3: %u 4: %u 5: %u\n", gpio_get(0), gpio_get(1), gpio_get(2), gpio_get(4), gpio_get(5), gpio_get(6));


#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
        gpio_put(PSDAPT_PIN_LED_CONN_WRONG_ORIENT, 1); //(measured_vbus > 0.f && cc1_is_bigger_than_cc2) ? 1 : 0);

        gpio_put(PSDAPT_PIN_LED_HMD_IS_READY, 0);
        sleep_ms(250);
        gpio_put(PSDAPT_PIN_PC_HPD, 1);
        sleep_ms(2);
        gpio_put(PSDAPT_PIN_PC_HPD, 0);
        sleep_ms(250);
        gpio_put(PSDAPT_PIN_LED_CONN_WRONG_ORIENT, 0);
        gpio_put(PSDAPT_PIN_LED_HMD_IS_READY, measured_vbus > 0.f ? 1 : 0);
        sleep_ms(250);
        gpio_put(PSDAPT_PIN_PC_HPD, 0);
#endif

    }
}
