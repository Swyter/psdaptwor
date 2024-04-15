/**
 * | psdaptwor - an experimental PSVR2 to PC adaptor | firmware
 * | created by Swyter <swyterzone+psdaptwor@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
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


void fusb_interrupt_callback(uint gpio, uint32_t event_mask)
{
    printf("[i] USB-C controller interrupt request: %x %x\b", gpio, event_mask);
    uint8_t rxdata; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_INTERRUPTA, &rxdata, 1); printf("int read FUSB_INTERRUPTA: %#x  ", rxdata);
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0,    &rxdata, 1); printf("int read FUSB_STATUS0:    %#x  ", rxdata);
                    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0A,   &rxdata, 1); printf("int read FUSB_STATUS0A:   %#x\n", rxdata);

    return;
}


float fusb_measure_vbus(void)
{
    uint8_t switchesBackup; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &switchesBackup, 1); printf("b read FUSB_SWITCHES0: %#x\n", switchesBackup);
    uint8_t measureBackup;  i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &measureBackup,  1); printf("b read FUSB_MEASURE: %#x\n", measureBackup);
    uint8_t counter = 0, lastrxdata = 0, rxdata = 0, ret = 0;

    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, switchesBackup & 0b11110011); printf("f write ret: %#x\n", ret);
    sleep_ms(1);

    while (counter < 30)
    {
        lastrxdata = rxdata;

        ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, FUSB_MEASURE_MEAS_VBUS | (counter & 0b111111)); printf("f write ret: %#x, counter: %x, comp volt: %f\n", ret, counter, counter * 0.420f);

        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, &rxdata, 1); printf("i2c_read FUSB_MEASURE: %#x\n", rxdata);
        sleep_ms(1);
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &rxdata, 1); printf("i2c_read FUSB_STATUS0: %#x COMP: %u\n", rxdata, (rxdata & FUSB_STATUS0_COMP));

        if ((lastrxdata & FUSB_STATUS0_COMP) && !(rxdata & FUSB_STATUS0_COMP))
            break;

        counter++;
    }

    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   measureBackup);
    i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, switchesBackup);

    return counter * 0.420f;
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

    gpio_init(0);
    gpio_set_dir(0, GPIO_IN);

    gpio_init(1);
    gpio_set_dir(1, GPIO_IN);

    bool zero = gpio_get(0);
    bool one  = gpio_get(1);

    gpio_init(PSDAPT_PIN_HMD_ENABLE_VBUS);
    gpio_set_dir(PSDAPT_PIN_HMD_ENABLE_VBUS, GPIO_OUT);

    gpio_init(PSDAPT_PIN_HMD_ENABLE_VBUS_12V);
    gpio_set_dir(PSDAPT_PIN_HMD_ENABLE_VBUS_12V, GPIO_OUT);

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

    printf("Done, for real.\n");

    uint8_t rxdata; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_DEVICE_ID, &rxdata, 1);
    printf("read 0: %#x\n", rxdata);
    printf("revision ID: %#x, Product ID: %#x, Revision ID: %#x\n", rxdata >> FUSB_DEVICE_ID_VERSION_ID_SHIFT,  (rxdata & 0b1100) >> FUSB_DEVICE_ID_PRODUCT_ID_SHIFT, (rxdata & 0b0011) >> FUSB_DEVICE_ID_REVISION_ID_SHIFT);

    uint32_t ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_RESET, FUSB_RESET_SW_RES); printf("b write ret: %#x\n", ret);

    if (ret != PICO_OK)
        printf("[!] couldn't reset it\n");

    sleep_ms(10);

    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_DEVICE_ID, &rxdata, 1); printf("b read 0: %#x\n", rxdata);
    printf("revision ID: %#x, Product ID: %#x, Revision ID: %#x\n", rxdata >> FUSB_DEVICE_ID_VERSION_ID_SHIFT,  (rxdata & 0b1100) >> FUSB_DEVICE_ID_PRODUCT_ID_SHIFT, (rxdata & 0b0011) >> FUSB_DEVICE_ID_REVISION_ID_SHIFT);

    /* Turn on all power */
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_POWER, FUSB_POWER_PWR0 | FUSB_POWER_PWR1 | FUSB_POWER_PWR2 | FUSB_POWER_PWR3); printf("b write ret: %#x\n", ret);

    /* Set interrupt masks */
    // Setting to 0 so interrupts are allowed
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MASK1, 0); printf("c write ret: %#x\n", ret);
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MASKA, 0); printf("d write ret: %#x\n", ret);
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MASKB, 0); printf("e write ret: %#x\n", ret);

    /* Flush the TX buffer */
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL0, FUSB_CONTROL0_TX_FLUSH | 0x4); printf("f write ret: %#x\n", ret);

    /* Flush the RX buffer */
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH); printf("f write ret: %#x\n", ret);

    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_CONTROL3, FUSB_CONTROL3_N_RETRIES | FUSB_CONTROL3_AUTO_RETRY); printf("f write ret: %#x\n", ret);


    /* Reset the PD logic */
    ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_RESET, FUSB_RESET_PD_RESET); printf("f write ret: %#x\n", ret);

    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &rxdata, 1); printf("b read FUSB_SWITCHES0: %#x\n", rxdata);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES1, &rxdata, 1); printf("b read FUSB_SWITCHES1: %#x\n", rxdata);
    i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &rxdata, 1); printf("b read FUSB_MEASURE: %#x\n", rxdata);

    uint8_t switchesBackup; i2c_read(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, &switchesBackup, 1); printf("b read FUSB_SWITCHES0: %#x\n", switchesBackup);
    uint8_t measureBackup;  i2c_read(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   &measureBackup,  1); printf("b read FUSB_MEASURE: %#x\n", measureBackup);
    uint8_t counter = 0, cc1 = 0, cc2 = 0;
    while (1) {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, measured voltage: %f V, actual pre-divided voltage: %f V\n", result, result * conversion_factor, result * (24.f / (1 << 12)));

        printf("[o] measured USB-C VBUS: %f\n", fusb_measure_vbus());

        ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, FUSB_SWITCHES0_PDWN_1 | FUSB_SWITCHES0_PDWN_2 | FUSB_SWITCHES0_MEAS_CC1); printf("cc1 write ret: %#x\n", ret);
        ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, 0); printf("cc1 write ret: %#x\n", ret);
        sleep_ms(10);
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &rxdata, 1); printf("cc1 i2c_read FUSB_STATUS0: %#x BC_LVL: %u\n", rxdata, (cc1 = rxdata & FUSB_STATUS0_BC_LVL));

        ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, FUSB_SWITCHES0_PDWN_1 | FUSB_SWITCHES0_PDWN_2 | FUSB_SWITCHES0_MEAS_CC2); printf("cc2 write ret: %#x\n", ret);
        ret = i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE, 0); printf("cc2 write ret: %#x\n", ret);
        sleep_ms(10);
        i2c_read(i2c_default, FUSB302B_ADDR, FUSB_STATUS0, &rxdata, 1); printf("cc2 i2c_read FUSB_STATUS0: %#x BC_LVL: %u\n", rxdata, (cc2 = rxdata & FUSB_STATUS0_BC_LVL));


        i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_MEASURE,   measureBackup);
        i2c_write_byte(i2c_default, FUSB302B_ADDR, FUSB_SWITCHES0, switchesBackup);

        

        gpio_put(PSDAPT_PIN_HMD_ENABLE_VBUS,     0);
        gpio_put(PSDAPT_PIN_HMD_ENABLE_VBUS_12V, 0);
        printf("gpio_is_pulled_down: %x ", gpio_is_pulled_down(PSDAPT_PIN_HMD_ENABLE_VBUS));
        printf("gpio_is_pulled_down: %x ", gpio_is_pulled_down(PSDAPT_PIN_HMD_ENABLE_VBUS_12V));

        sleep_ms(500);

        printf("\nGPIO 0: %u 1: %u 2: %u 3: %u 4: %u 5: %u\n", gpio_get(0), gpio_get(1), gpio_get(2), gpio_get(4), gpio_get(5), gpio_get(6));


#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
        gpio_put(PSDAPT_PIN_LED_CONN_WRONG_ORIENT, (cc2 > cc1) ? 1 : 0);

        gpio_put(PSDAPT_PIN_LED_HMD_IS_READY, 0);
        sleep_ms(250);
        //gpio_put(PSDAPT_PIN_LED_CONN_WRONG_ORIENT, 0);
        gpio_put(PSDAPT_PIN_LED_HMD_IS_READY, 1);
        sleep_ms(250);
#endif

    }
}
