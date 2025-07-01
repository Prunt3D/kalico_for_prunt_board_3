// Analog to digital (ADC) on stm32h7 and similar chips
//
// Copyright (C) 2020 Konstantin Vogel <konstantin.vogel@gmx.net>
// Copyright (C) 2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown

#define ADC_INVALID_PIN 0xFF

#define ADC_TEMPERATURE_PIN 0xfe
DECL_ENUMERATION("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

DECL_CONSTANT("ADC_MAX", 32767);

#define ADCIN_BANK_SIZE 20

// ADC timing
#define ADC_CKMODE 0b11
#define ADC_ATICKS 0b101
#define ADC_ATICKS_H723_ADC3 0b111
// stm32g4: clock=37.5Mhz, Tsamp=92.5, Tconv=105, Oversampling=8, total=22.4us

// Handle register name differences between chips
#if CONFIG_MACH_STM32H723
  #define PCSEL PCSEL_RES0
#elif CONFIG_MACH_STM32G4
  #define ADC_CCR_TSEN ADC_CCR_VSENSESEL
#endif

struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    // Find pin in adc_pins table
    int chan;

    if (pin == PRUNT_GPIO_THERMISTOR_START + 0) {
        chan = 8;
    } else if (pin == PRUNT_GPIO_THERMISTOR_START + 1) {
        chan = 9;
    } else if (pin == PRUNT_GPIO_THERMISTOR_START + 2) {
        chan = 3;
    } else if (pin == PRUNT_GPIO_THERMISTOR_START + 3) {
        chan = 2;
    } else if (pin == ADC_TEMPERATURE_PIN) {
        chan = 16;
    } else {
        shutdown("Not a valid ADC pin");
    }

    // Determine which ADC block to use and enable its clock
    ADC_TypeDef *adc;
    ADC_Common_TypeDef *adc_common;
#ifdef ADC3
    if (chan >= 2 * ADCIN_BANK_SIZE) {
        chan -= 2 * ADCIN_BANK_SIZE;
        adc = ADC3;
#if CONFIG_MACH_STM32G4
        adc_common = ADC345_COMMON;
#else
        adc_common = ADC3_COMMON;
#endif
    } else
#endif
#ifdef ADC2
    if (chan >= ADCIN_BANK_SIZE) {
        chan -= ADCIN_BANK_SIZE;
        adc = ADC2;
        adc_common = ADC12_COMMON;
    } else
#endif
    {
        adc = ADC1;
        adc_common = ADC12_COMMON;
    }
    if (!is_enabled_pclock((uint32_t)adc_common))
        enable_pclock((uint32_t)adc_common);
    MODIFY_REG(adc_common->CCR, ADC_CCR_CKMODE_Msk,
               ADC_CKMODE << ADC_CCR_CKMODE_Pos);

    // Enable the ADC
    if (!(adc->CR & ADC_CR_ADEN)) {
        // Switch on voltage regulator and wait for it to stabilize
        uint32_t cr = ADC_CR_ADVREGEN;
        adc->CR = cr;
        uint32_t end = timer_read_time() + timer_from_us(20);
        while (timer_is_before(timer_read_time(), end))
            ;

        // Setup chip specific flags
        uint32_t aticks = ADC_ATICKS;
#if CONFIG_MACH_STM32H7
        if (CONFIG_MACH_STM32H723 && adc == ADC3) {
            aticks = ADC_ATICKS_H723_ADC3;
        } else {
            // Use linear calibration on stm32h7
            cr |= ADC_CR_ADCALLIN;
            // Set boost mode on stm32h7 (adc clock is at 25Mhz)
            cr |= 0b10 << ADC_CR_BOOST_Pos;
            // Set 12bit samples on the stm32h7
            adc->CFGR = ADC_CFGR_JQDIS | (0b110 << ADC_CFGR_RES_Pos);
        }
#endif
        // Use 8x oversampling.
        adc->CFGR2 = 0b010 << ADC_CFGR2_OVSR_Pos | ADC_CFGR2_ROVSE | ADC_CFGR2_JOVSE;

        // Perform adc calibration
        adc->CR = cr | ADC_CR_ADCAL;
        while (adc->CR & ADC_CR_ADCAL)
            ;

        // Enable ADC
        adc->ISR = ADC_ISR_ADRDY;
        adc->ISR; // Dummy read to make sure write is flushed
        while (!(adc->CR & ADC_CR_ADEN))
            adc->CR |= ADC_CR_ADEN;
        while (!(adc->ISR & ADC_ISR_ADRDY))
            ;

        // Set ADC clock cycles sample time for every channel
        uint32_t av = (aticks           | (aticks << 3)  | (aticks << 6)
                       | (aticks << 9)  | (aticks << 12) | (aticks << 15)
                       | (aticks << 18) | (aticks << 21) | (aticks << 24)
                       | (aticks << 27));
        adc->SMPR1 = av;
        adc->SMPR2 = av;
    }

    if (pin == ADC_TEMPERATURE_PIN) {
        adc_common->CCR |= ADC_CCR_TSEN;
    } else {
        gpio_peripheral(PRUNT_GPIO_TO_GPIO(pin), GPIO_ANALOG, 0);
    }

    // Setup preselect (connect) channel on stm32h7
#if CONFIG_MACH_STM32H7
    adc->PCSEL |= (1 << chan);
#endif
    return (struct gpio_adc){ .adc = adc, .chan = chan };
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    ADC_TypeDef *adc = g.adc;
    uint32_t cr = adc->CR;
    if (cr & ADC_CR_ADSTART)
        goto need_delay;
    if (adc->ISR & ADC_ISR_EOC) {
        if (adc->SQR1 == (g.chan << ADC_SQR1_SQ1_Pos))
            return 0;
        goto need_delay;
    }
    // Start sample
    adc->SQR1 = (g.chan << ADC_SQR1_SQ1_Pos);
    adc->CR = cr | ADC_CR_ADSTART;
need_delay:
    return timer_from_us(25);
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    ADC_TypeDef *adc = g.adc;
    return adc->DR;
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    ADC_TypeDef *adc = g.adc;
    irqstatus_t flag = irq_save();
    if (adc->SQR1 == (g.chan << ADC_SQR1_SQ1_Pos)) {
        uint32_t cr = adc->CR;
        if (cr & ADC_CR_ADSTART)
            adc->CR = (cr & ~ADC_CR_ADSTART) | ADC_CR_ADSTP;
        if (adc->ISR & ADC_ISR_EOC)
            gpio_adc_read(g);
    }
    irq_restore(flag);
}
