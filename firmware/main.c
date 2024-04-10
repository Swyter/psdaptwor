/**
 * | psdaptwor - an experimental PSVR2 to PC adaptor | firmware
 * | created by Swyter <swyterzone+psdaptor@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "pico/binary_info.h"
#include "hardware/i2c.h"

/*      Main microcontroller chip: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
                                   https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf

        Type-C PD controller chip: https://www.onsemi.com/pdf/datasheet/fusb302b-d.pdf
                                   https://github.com/Ralim/usb-pd/

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
 

int main() {
    stdio_init_all();
    stdio_usb_init();
    stdio_usb_connected();
    printf("ADC Example, measuring GPIO26\n");

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init   (LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put    (LED_PIN, 1);

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

    

    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);


    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    //gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    //gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
 
    printf("\nI2C Bus Scan\n");
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
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            //ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
 
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    printf("Done.\n");


    while (1) {
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        sleep_ms(500);

        printf("\nGPIO 0: %u 1: %u 2: %u 3: %u 4: %u 5: %u\n", gpio_get(0), gpio_get(1), gpio_get(2), gpio_get(4), gpio_get(5), gpio_get(6));


#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
        gpio_put(LED_PIN, 1);
        gpio_put(PSDAPT_PIN_LED_HMD_IS_READY, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        gpio_put(PSDAPT_PIN_LED_HMD_IS_READY, 1);
        sleep_ms(250);
#endif

    }
}
