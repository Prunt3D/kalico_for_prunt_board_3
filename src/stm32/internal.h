#ifndef __STM32_INTERNAL_H
#define __STM32_INTERNAL_H
// Local definitions for STM32 code

#include "autoconf.h" // CONFIG_MACH_STM32F1

#if CONFIG_MACH_STM32F0
#include "stm32f0xx.h"
#elif CONFIG_MACH_STM32F1
#include "stm32f1xx.h"
#elif CONFIG_MACH_STM32F2
#include "stm32f2xx.h"
#elif CONFIG_MACH_STM32F4
#include "stm32f4xx.h"
#elif CONFIG_MACH_STM32F7
#include "stm32f7xx.h"
#elif CONFIG_MACH_STM32G0
#include "stm32g0xx.h"
#elif CONFIG_MACH_STM32G4
#include "stm32g4xx.h"
#elif CONFIG_MACH_STM32H7
#include "stm32h7xx.h"
#elif CONFIG_MACH_STM32L4
#include "stm32l4xx.h"
#endif

#define PRUNT_GPIO_THERMISTOR_START 0
#define PRUNT_GPIO_STEPPER_STEP_START 4
#define PRUNT_GPIO_STEPPER_DIR_START 10
#define PRUNT_GPIO_HEATER_PWM_START 16
#define PRUNT_GPIO_FAN_HS_PWM_START 18
#define PRUNT_GPIO_FAN_LS_PWM_START 22
#define PRUNT_GPIO_TMC_UART_INTERNAL 26
#define PRUNT_GPIO_ENDSTOP_START 27
#define PRUNT_GPIO_FAN_TACH_START 31
#define PRUNT_GPIO_TMC_UART 35

// gpio.c
extern GPIO_TypeDef * const digital_regs[];
#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

#define PRUNT_GPIO_TO_GPIO(PRUNT_GPIO) \
    (((int[]){ \
        [PRUNT_GPIO_THERMISTOR_START]   = GPIO('C',  2), GPIO('C',  3), GPIO('A',  3), GPIO('A',  2), \
        [PRUNT_GPIO_STEPPER_STEP_START] = GPIO('A', 11), GPIO('A',  8), GPIO('C',  9), GPIO('B', 13), GPIO('C',  7), GPIO('B', 15), \
        [PRUNT_GPIO_STEPPER_DIR_START]  = GPIO('A', 10), GPIO('A',  9), GPIO('C',  8), GPIO('B', 12), GPIO('C',  6), GPIO('B', 14), \
        [PRUNT_GPIO_HEATER_PWM_START]   = GPIO('B',  2), GPIO('B',  5), \
        [PRUNT_GPIO_FAN_HS_PWM_START]   = GPIO('A',  5), GPIO('A',  6), GPIO('B',  9), GPIO('A', 15), \
        [PRUNT_GPIO_FAN_LS_PWM_START]   = GPIO('A',  3), GPIO('A',  4), GPIO('B',  7), GPIO('A', 14), \
        [PRUNT_GPIO_TMC_UART_INTERNAL]  = GPIO('B',  3), \
        [PRUNT_GPIO_ENDSTOP_START]      = GPIO('B',  6), GPIO('C', 13), GPIO('C', 14), GPIO('C', 15), \
        [PRUNT_GPIO_FAN_TACH_START]     = GPIO('A',  7), GPIO('B',  0), GPIO('B',  1), GPIO('B', 11), \
        [PRUNT_GPIO_TMC_UART]           = GPIO('B',  3) \
    })[PRUNT_GPIO])

// gpioperiph.c
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_HIGH_SPEED 0x200
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);

// clockline.c
void enable_pclock(uint32_t periph_base);
int is_enabled_pclock(uint32_t periph_base);

// dfu_reboot.c
void dfu_reboot(void);
void dfu_reboot_check(void);

// stm32??.c
struct cline { volatile uint32_t *en, *rst; uint32_t bit; };
struct cline lookup_clock_line(uint32_t periph_base);
uint32_t get_pclock_frequency(uint32_t periph_base);
void gpio_clock_enable(GPIO_TypeDef *regs);

#endif // internal.h
