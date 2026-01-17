#pragma once

#define SMOL_SERVO_PHASES                   (3)
#define SMOL_SERVO_PWM_PER_LOOP             (3)

#define SMOL_SERVO_LOOP_FREQ_HZ             (20000)
#define SMOL_SERVO_LOOP_PERIOD_CLOCKS       (7500)

#define SMOL_SERVO_BRIDGE_PWM_FREQ_HZ       (60000)
#define SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS (2500)

#define SMOL_SERVO_ADC_FREQ_HZ              (240000)
#define SMOL_SERVO_ADC_PERIOD_CLOCKS        (625)

#define SMOL_SERVO_ADC_ADVANCE_COUNT         (150)

#define SMOL_SERVO_PWM_SLICE_MASK ((1u << PWMA_SLICE) | (1u << PWMB_SLICE) | (1u << PWMC_SLICE) | (1u << PWMR_SLICE) | (1u << PWM_ADC_SLICE)| (1u << PWM_SERVO_LOOP_SLICE))
// #define SMOL_SERVO_PWM_SLICE_MASK (0xFFFFFFFF)

#define BRIDGE_PWM_WRAP_IRQ PWM_IRQ_WRAP_0
#define ADC_IRQ ADC_IRQ_FIFO

#define ADC_REF_VOLTAGE 3.3f
#define ADC_REF_RESISTANCE 100.0e6f
#define ADC_DIV_LEG 5.1e3f
// consider ADC resistance to R1 of voltage divider
/**
 * VX_-----------o---------
 *               |
 *             [R_2]
 *               |
 * VSENSE-o------o------.
 *        |      |      |
 *     [R_ADC] [R_1] [Z 3.3V]
 *        |      |      |
 * GND_---o------o------o--
 * 
 * VSENSE = VX * (R_1 || R_ADC) / ((R_1 || R_ADC) + R_2)
 * 
 */
#define ADC_DIV_COMBO ((ADC_DIV_LEG * ADC_REF_RESISTANCE) / (ADC_DIV_LEG + ADC_REF_RESISTANCE))
#define ADC_VDD_DIV (ADC_DIV_COMBO / (22.0e3f + ADC_DIV_COMBO))
#define ADC_VBUS_DIV (ADC_DIV_COMBO / (120.0e3f + ADC_DIV_COMBO))

// NOTE: on ARM, the IRQ group.subgroup is configured via the AICR's PRIGROUP field. The default is a single bit for subgroup, rest group.
// Since the RP2350 only implements the top 4 bits, this should mean we have 16 priority levels with no subpriority selection.
#define SMOL_ADC_IRQ_PRIORITY  (0x10)
#define SMOL_PWM_IRQ_PRIORITY  (0x20)
#define SMOL_LOOP_IRQ_PRIORITY (0x80)

#define DEBUG_LOOP_IRQ_WITH_GPIO 8
#define DEBUG_ADC_TIMING_WITH_GPIO 9
#define DEBUG_PWM_IRQ_WITH_GPIO 13
// #define DEBUG_PWM_ADC_IRQ
// #define DEBUG_PWM_ADC_GPIO 28
// #define DEBUG_DMA_ADC
// #define DEBUG_ADC_FIFO_THRES
#define DEBUG_ADC_FIFO_THRES_WITH_GPIO 12
