# Servo Controller Firmware for RP2350

## Target Features

 - high frequency servo loop
   - LQG linear quadratic gaussian control
   - motor param auto-identification
   - on-axis AD5047 magnetic encoder
   - external A/B incremental encoder
 - 3-phase bridge control
 - 2/3-phase bridge control: 2ph stepper on 3ph bridge
 - multi-axis sync
 - 240W USB PD (48V @ 5A) or stand-alone 12V-48V supply

## Hardware Mapping

### 3ph Bridge + Power Resistor

GPIO22 | PWM[3A] | PWMR 
GPIO24 | PWM[4A] | PWMA
GPIO25 | PWM[4B] | ENA
GPIO26 | PWM[5A] | PWMB
GPIO27 | PWM[5B] | ENB
GPIO32 | PWM[8A] | PWMC
GPIO33 | PWM[8B] | ENC

### 3ph Driver Comms

GPIO28 | DRV_MISO
GPIO29 | DRV_NSS
GPIO30 | DRV_SLCK
GPIO31 | DRV_MOSI

GPIO34 | DRVFAULT
GPIO35 | DRVEN

### 3ph Current Sense

GPIO40_ADC0 | SOA
GPIO41_ADC1 | SOB
GPIO42_ADC2 | SOC

### Voltage Sensing 

GPIO43_ADC3 | VBUS
GPIO44_ADC4 | VDD
GPIO45_ADC5 | ADCA
GPIO46_ADC6 | ADCB
GPIO47_ADC7 | ADCC

### AD5047 Magnetic Encoder

GPIO36 | MAG_MISO
GPIO37 | MAG_NSS
GPIO38 | MAG_SLCK
GPIO39 | MAG_MOSI

### Encoder Input

GPIO3 | ENCI
GPIO4 | ENCA
GPIO5 | ENCB

### AP2020 RGB LED

GPIO10 | LED_SCK
GPIO11 | LED_MOSI

### FUSB302 USB PD Controller

GPIO0 | USB_PD_SDA
GPIO1 | USB_PD_SCL
GPIO2 | USB_PD_INT

### Serial Console UART

GPIO6 | CONSOLE_TX
GPIO7 | CONSOLE_RX

### Aux Flash

GPIO23 | FLASH_SS

Data on QSPI_* pins.

### Other GPIO

GPIO8
GPIO9
GPIO12
GPIO13
GPIO14 | SDA1_EXT
GPIO15 | SCL1_EXT
GPIO16 | SYNC
GPIO17-21 is not connected

## Servo Loop Timing

Assuming `sysclk = 150MHz`, and 20kHz control loop, we should run PWM in up/down mode at 60kHz (3 phases * 20kHz) and do ADC reads at both counter TOP and 0 values. `TOP+1 = 1250` for `2*60 = 120kHz` operation.


