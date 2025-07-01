// GPIO functions on stm32f4
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // ffs
#include "board/irq.h" // irq_save
#include "command.h" // DECL_ENUMERATION_RANGE
#include "gpio.h" // gpio_out_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown

DECL_ENUMERATION_RANGE("pin", "GPIO_THERMISTOR_1", PRUNT_GPIO_THERMISTOR_START, 4);
DECL_ENUMERATION_RANGE("pin", "GPIO_STEPPER_STEP_1", PRUNT_GPIO_STEPPER_STEP_START, 6);
DECL_ENUMERATION_RANGE("pin", "GPIO_STEPPER_DIR_1", PRUNT_GPIO_STEPPER_DIR_START, 6);
DECL_ENUMERATION_RANGE("pin", "GPIO_HEATER_PWM_1", PRUNT_GPIO_HEATER_PWM_START, 2);
DECL_ENUMERATION_RANGE("pin", "GPIO_ENDSTOP_1", PRUNT_GPIO_ENDSTOP_START, 4);
DECL_ENUMERATION_RANGE("pin", "GPIO_FAN_HS_PWM_1", PRUNT_GPIO_FAN_HS_PWM_START, 4);
DECL_ENUMERATION_RANGE("pin", "GPIO_FAN_LS_PWM_1", PRUNT_GPIO_FAN_LS_PWM_START, 4);
DECL_ENUMERATION_RANGE("pin", "GPIO_FAN_TACH_1", PRUNT_GPIO_FAN_TACH_START, 4);
DECL_ENUMERATION("pin", "GPIO_TMC_UART", PRUNT_GPIO_TMC_UART);

GPIO_TypeDef * const digital_regs[] = {
    ['A' - 'A'] = GPIOA, GPIOB, GPIOC,
#ifdef GPIOD
    ['D' - 'A'] = GPIOD,
#endif
#ifdef GPIOE
    ['E' - 'A'] = GPIOE,
#endif
#ifdef GPIOF
    ['F' - 'A'] = GPIOF,
#endif
#ifdef GPIOG
    ['G' - 'A'] = GPIOG,
#endif
#ifdef GPIOH
    ['H' - 'A'] = GPIOH,
#endif
#ifdef GPIOI
    ['I' - 'A'] = GPIOI,
#endif
};

// Convert a register and bit location back to an integer pin identifier
static int
regs_to_pin(GPIO_TypeDef *regs, uint32_t bit)
{
    int i;
    for (i=0; i<ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return GPIO('A' + i, ffs(bit)-1);
    return 0;
}

// Verify that a gpio is a valid pin
static int
gpio_valid(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin);
    return port < ARRAY_SIZE(digital_regs) && digital_regs[port];
}

struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (pin < PRUNT_GPIO_STEPPER_STEP_START || pin >= PRUNT_GPIO_ENDSTOP_START) {
        shutdown("Not an output pin");
    }

    pin = PRUNT_GPIO_TO_GPIO(pin);

    if (!gpio_valid(pin))
        shutdown("Not an output pin");
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    gpio_clock_enable(regs);
    struct gpio_out g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_out_reset(g, val);
    return g;
}

void
gpio_out_reset(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << 16;
    gpio_peripheral(pin, GPIO_OUTPUT, 0);
    irq_restore(flag);
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    GPIO_TypeDef *regs = g.regs;
    regs->ODR ^= g.bit;
}

void
gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

void
gpio_out_write(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = g.regs;
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << 16;
}


struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (pin < PRUNT_GPIO_TMC_UART_INTERNAL || pin >= PRUNT_GPIO_TMC_UART) {
        shutdown("Not an input pin");
    }

    pin = PRUNT_GPIO_TO_GPIO(pin);

    if (!gpio_valid(pin))
        shutdown("Not a valid input pin");
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_in g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up);
    return g;
}

void
gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    GPIO_TypeDef *regs = g.regs;
    return !!(regs->IDR & g.bit);
}
