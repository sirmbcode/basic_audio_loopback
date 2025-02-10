/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/spi.h"

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

/**
    GPIO 5 (pin 7) Chip select
    GPIO 6 (pin 9) SCK/spi0_sclk
    GPIO 7 (pin 10) MOSI/spi0_tx
    GPIO 2 (pin 4) GPIO output for timing ISR
    3.3v (pin 36) -> VCC on DAC
    GND (pin 3)  -> GND on DAC

 */

// SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

// DAC config
// Values output to DAC
volatile uint16_t DAC_output_0;
volatile uint16_t DAC_output_1;

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// DAC parameters (see the DAC datasheet)
// A-channel and B-Channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// ADC config
volatile int curr_dac_val;

// INTERRUPT CODE -----------------
//--------------------------------
const float conversion_factor = (3.3f / (1 << 12)); // conversion val

/// \tag::get_time[]
// Simplest form of getting 64 bit time from the timer.
// It isn't safe when called from 2 cores because of the latching
// so isn't implemented this way in the sdk
static uint64_t get_time(void)
{
    // Reading low latches the high value
    uint32_t lo = timer_hw->timelr;
    uint32_t hi = timer_hw->timehr;
    return ((uint64_t)hi << 32u) | lo;
}
/// \end::get_time[]

/// \tag::alarm_standalone[]
// Use alarm 0
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)
static volatile bool alarm_fired; // bool for while loop, not sure if i need

volatile uint16_t result;
volatile float calcVal;
//************************** */
// interrupt routine
// IRQ!!!!!!!!!
//************************** */
static void alarm_irq(void)
{
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Assume alarm 0 has fired
    printf("Alarm IRQ fired\n");
    alarm_fired = true;

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    DAC_output_0 = adc_read();

    // Mask with DAC control bits
    DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

    // SPI write (no spinlock b/c of SPI buffer)
    spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
}

// setup for interrupt
static void alarm_in_us(uint32_t delay_us)
{
    // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq); // Set irq handler for alarm irq
    irq_set_enabled(ALARM_IRQ, true);                // Enable the alarm irq
    // Enable interrupt in block and at processor

    // Alarm is only 32 bits so if trying to delay more
    // than that, need to be careful and keep track of the upper
    // bits
    uint64_t target = timer_hw->timerawl + delay_us;

    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

// MAIN LOOP-----------------------
//--------------------------------

int main()
{
    // basic inits
    stdio_init_all();

    // ADC config
    // Make sure GPIO is high-impedance, no pullups etc

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0); // ADC input 0 (GPIO26)

    // SPI CONFIG
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    //
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    printf("Setting up:\n");
    sleep_ms(10000);

    while (1)
    {
        alarm_fired = false; // why do i need this
        alarm_in_us(35);     // load timer

        // Wait for alarm to fire here
        while (!alarm_fired)
        {
        }
        calcVal = DAC_output_0 * conversion_factor;
        printf("voltage: %f V\n", calcVal);
    }
}
