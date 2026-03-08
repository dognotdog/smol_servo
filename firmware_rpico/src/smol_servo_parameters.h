#pragma once

/**
 * 3 phase servo loop sketch
 * 
 * LOOP ...  t=0                                             t=1...
 * 20k       |                                               |
 * PWM  ...  0               1               2               0  ...
 * 60k       |               |               |               |
 * MAG  ...  0           1           2           3           0  ...
 * 80k       |           |           |           |           |
 * ADC  ... 0.0 0.1 0.2 0.3 1.0 1.1 1.2 1.3 2.0 2.1 2.2 2.3 0.0 ... 
 * 240k     |   |   |   | : |   |   |   | : |   |   |   | : |
 * ADC-IRQ                x               x               x 
 * 
 * The LOOP IRQ is driven by signaling from the PWM callback, but ADC and MAG run independent PWM channels.
 * 
 * The startup phase is interesting to get the callbacks aligned:
 * PWM            >               0               1      ...
 * 60k            >               |               |
 * MAG            >   3   .   .   0           1          ...
 * 80k            >~~~|           |           |        
 * ADC (-1us)     >  2.1 2.2 2.3 0.0 0.1 0.2 0.3 1.0 1.1 ... 
 * 240k           >~~|   |   | : |   |   |   | : |   |
 * ADC-IRQ                     x               x            
 * 
 * [squiggly "~" line means shortened COUNT to align phases]
 * 
 * - ADC triggering has a -1us offset because sampling takes 2us, and we want to be centered on the PWM wrap as the primary tick. For starting out, to have the 1:4 ADC-IRQ align correctly with the PWM, the first "block" is only 3 ADC reads, then is switched to 4.
 *   - `-1us` offset for ADC read
 *   - 3 vs 4 reads in first block
 *   - start at N-1 phase for the IRQ
 * - MAG is a slightly shorter period than PWM, so we need to 
 *   - offset to align zero phases of PWM and MAG
 *   - start at N-1 phase of buffers
 * 
 * 
 */

#define SMOL_SERVO_PHASES                   (3)
#define SMOL_SERVO_PWM_PER_LOOP             (3)

#define SMOL_SERVO_LOOP_FREQ_HZ             (20000)
#define SMOL_SERVO_LOOP_PERIOD_CLOCKS       (7500)

#define SMOL_SERVO_BRIDGE_PWM_FREQ_HZ       (60000)
#define SMOL_SERVO_BRIDGE_PWM_PERIOD_CLOCKS (2500)

#define SMOL_SERVO_ADC_FREQ_HZ              (240000)
#define SMOL_SERVO_ADC_PERIOD_CLOCKS        (625)

#define SMOL_SERVO_ADC_ADVANCE_COUNT        (150)

#define SMOL_SERVO_MAG_PWM_PER_LOOP         (4)
#define SMOL_SERVO_MAG_FREQ_HZ              (80000)
#define SMOL_SERVO_MAG_PERIOD_CLOCKS        (1875)

#define SMOL_SERVO_PWM_SLICE_MASK ((1u << PWMA_SLICE) | (1u << PWMB_SLICE) | (1u << PWMC_SLICE) | (1u << PWMR_SLICE) | (1u << PWM_ADC_SLICE) | (1u << PWM_SERVO_LOOP_SLICE) | (1u << PWM_DOUBLETIME_SLICE) | (1u << PWM_MAG_SLICE))
// #define SMOL_SERVO_PWM_SLICE_MASK (0xFFFFFFFF)

#define BRIDGE_PWM_WRAP_IRQ PWM_IRQ_WRAP_0
#define ADC_IRQ ADC_IRQ_FIFO

// current sense resistors
#define ISENSE_R (5.0e-3f)
// TMC6200 amplifier settings
#define ISENSE_AMPLIFICATIONS {5.0f, 10.0f, 10.0f, 20.0f}
// THE TMC6200 is using its own 5V regulator, so measured ZERO is expected to vary because the ADC VDD is an independent rail.
#define VSENSE_NOMINAL_ZERO_V (5.0f/3.0f)
#define VSENSE_ZERO_V_INIT_VARIANCE (0.1f*0.1f)
// 10mV/sec
#define VSENSE_ZERO_V_UPDATE_VARIANCE (0.01f*0.01f)

#define ADC_REF_VOLTAGE (3.3f)
#define ADC_REF_RESISTANCE (100.0e6f)
#define ADC_DIV_LEG_R (5.1e3f)
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
#define ADC_DIV_COMBO ((ADC_DIV_LEG_R * ADC_REF_RESISTANCE) / (ADC_DIV_LEG_R + ADC_REF_RESISTANCE))
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
