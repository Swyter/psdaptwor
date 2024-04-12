/**
 * | psdaptwor - an experimental PSVR2 to PC adaptor | custom RP2040 board pinout and boot-up defines
 * | created by Swyter <swyterzone+psdaptor@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

#ifndef _BOARDS_SWYTER_PSDAPTWOR_RP2040_H
#define _BOARDS_SWYTER_PSDAPTWOR_RP2040_H

// For board detection
#define SWYTER_PSDAPTWOR_RP2040

// On some samples, the xosc can take longer to stabilize than is usual
#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

//------------- UART -------------//
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0 /* swy: we need to use [UART HW BLOCK 0] for GPIO12 and GPIO13, it's the only function that can be used in those pins; [UART HW BLOCK 1] won't work here */
#endif

#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12 /* swy: GPIO12 */
#endif

#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13 /* swy: GPIO13 */
#endif

//------------- LED -------------//
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 14 /* swy: GPIO14, also known as PSDAPT_PIN_LED_CONN_WRONG_ORIENT */
#endif

//------------- I2C -------------//
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0 /* swy: important: we need to use [I2C HW Block 0], which is the only function that works for GPIO0 and GPIO1. [I2C HW Block 1] won't do anything unless you change pins; see the diagram in gpio.h */
#endif

#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 0 /* swy: GPIO0 */
#endif

#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 1 /* swy: GPIO1 */
#endif

//------------- FLASH -------------//

// Use slower generic flash access
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 4
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024) /* swy: 16 MiB for the W25Q128JVSIQ flash chip, or 128 Mbits in manufacturer lingo */
#endif

// All boards have B1 RP2040
#ifndef PICO_RP2040_B0_SUPPORTED
#define PICO_RP2040_B0_SUPPORTED 0
#endif

#endif
