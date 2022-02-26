# Project Layout

## firmware48

Contains files specific to 48pin STM32G431 on the stm32-nema17-servo board.

## mechaduino-firmware

Firmware for Mechaduino's SAMD21G18A Cortex M0+.

## hardware

Hardware design files for Schematic and PCB.

## tests

There is a basic harness in place to use [Unity](https://github.com/ThrowTheSwitch/Unity) for unit testing. Create `foo_test.c` modules for testing, and `foo_test.mk` makefiles for additional setup.

# Calculations

### Current Sense Shunt Resistor Value

Rated continous current 8A, max. Bus Voltage 50V, min. load 1R

 - 1206 can dissipate 1.5W usually, using a conservative 1W for the rated current: P/I^2 -> Rs < 15.6mOhm


With a 1R load on a 50V bus, up to 50A could flow at once, but hopefully motor inductance smoothes that out. Assuming 50kHz, 2mH winding inductance:
	V = L * di/dt -> di/dt = V/L = 25kA/s @ 50kHz = 0.5 A/cycle


### Gate Drive

The required charge pump current is
	I_VCP = 3 * Q_G * f_PWM.

DRV8323 charge pump VBUS limits:
6V -> 10mA
8V -> 15mA
10V -> 20mA
13V -> 25mA

#### NTTFS4937N

Q_G = 35.5 nC

I_VCP = 3.55 mA @ 100kHz
I_VCP = 1.78 mA @ 50kHz

Charge pump current capacity is well over required charge pump current.

#### FDMC86570L‎

Q_G < 88 nC

I_VCP = 8.8 mA @ 100kHz
I_VCP = 4.4 mA @ 50kHz

### Passive NSS OR circuit

3.3V, 5pF gate capacitance, 8mA sink/source per pin -> 412.5 Ohm minimum resistance

f_RC = 1 / 2pi R C

for R=470 that is 67.7MHz
for R=1k that is 31.8MHz
for R=2.2k that is 14.4MHz

max SPI frequency is 10MHz. 1k for both lines seems sufficiently fast and will not exceed max current on pin. Only one will be actively pulled high, the other will be in high impedance mode, but 1k is safe even if both are actively driven against the NSS pin.


# Electrical Motor Driving

## RL Circuit
The principal equation for an RL circuit, as is assumed for a motor winding, describing the relationship between voltage and current is:
`V = L * di/dt + R i`
or in differential form
```
	        V - R i
	di/dt = -------
	           L
```

### Discrete Time RL Equation

In discrete time, with n=0,1,2... indices going backward in time, the equation becomes
`V_0 = L * (i_0 - i_1) / T + R i_0`
solving for L
```
      V_0 - R i_0
L = T -----------
       i_0 - i_1
```
solving for R
```
    V_0 - L * (i_0 - i_1) / T
R = -------------------------
               i_0
```
assuming a measurement at n=0 for R and n=1 for L, and substituting
```
                V_0 - L * (i_0 - i_1) / T
      V_1 - i_1 -------------------------
                           i_0
L = T -----------------------------------
                 i_1 - i_2

      V_1 i_0 - i_1 V_0     i_1 (i_0 - i_1)
L = T ----------------- + L ---------------
       i_0 (i_1 - i_2)      i_0 (i_1 - i_2)        

               V_1 i_0 - i_1 V_0 
L = T ----------------------------------
       i_0 (i_1 - i_2) - i_1 (i_0 - i_1)        

      V_1 i_0 - i_1 V_0          i_0 - i_1
L = T ----------------- = T V ---------------
       i_1^2 - i_0 i_2        i_1^2 - i_0 i_2



          V_1 - R i_1
    V_0 - ----------- (i_0 - i_1)
           i_1 - i_2
R = -----------------------------
                i_0

    V_0 (i_1 - i_2) - V_1 (i_0 - i_1) + R i_1 (i_0 - i_1)
R = -----------------------------------------------------
                      i_0 (i_1 - i_2)

    V_0 (i_1 - i_2) - V_1 (i_0 - i_1)
R = ---------------------------------
    i_0 (i_1 - i_2) - i_1 (i_0 - i_1)

    V_0 (i_1 - i_2) - V_1 (i_0 - i_1)     2 i_1 - i_2 - i_0
R = --------------------------------- = V -----------------
             i_1^2 - i_0 i_2               i_1^2 - i_0 i_2

       L         i_0 - i_1
tau = --- = T -----------------
       R      2 i_1 - i_2 - i_0
```

# Stepper Control

A Bipolar hybrid stepper motor typically has 200 steps/rev on 2 phases.

An electrical revolution is four full steps: 
A+ -> B+ -> A- -> B-

NEMA 17 typically up to 3A current (eg. Nanotec ST4118D3004)""

winding current: 3A
winding inductance: 2.7mH 
winding resistance: 1.1R
Holding Torque: 0.8Nm
Rotor Inertia: 1.02e-5 Kg m^2
Torque Constant (mech): 0.267 Nm / A [or V / rad s]

At 48V, the motor should be able to spin (theoretically, discounting nonlinear effects) at 179.7 rad/s, about 1716 rpm or 28.6 rev/s.

at 30 rev/s with 200 steps/rev, we have to make 6000 full steps per second.

# BLDC Control

A BLDC has 3 phases, which are most simplistically driven with trapezoidal voltage curves instead of sinusoidal ones.
```
      _       _
A   /   \   /
  -       -
        _      
B \   /   \   /
    -       -
  _       _      
C   \   /   \   
      -       -
```

A ~ sin(alpha)
B ~ sin(alpha + 2pi/3)
C ~ sin(alpha + 4pi/3)

In a typical configuration, the three windings A,B,C are connected in a star pattern, with the endpoints named U,V,W.
```
V       U
B \   / A	    
    *       
    | C
    W
```
A = 

U-V 	= A
V-W 	= B-C = sin(a + 2pi/3) - sin(a + 4pi/3) 
	= sin(a) cos(2pi/3) + cos(a) sin(2pi/3) - sin(a) cos (4pi/3) - cos(a) sin(4pi/3)
	= sin(a) - sqrt(3) cos(a)
W-U = C-A



# STM32G431 Setup

## USB

Crystal-less USB Setup in MX
 - USB clock source should be `HSI48`1
 - set `CRS SYNC` to `USB_FS` in RCC Mode & Config
 - enable `Sof Enable` in USB device settings

Full-Speed USB (12Mbps) has a max. packet size of 64 bytes for regular packets, only isochronous might be up to 1023.

### CDC
 - increase `APP_RX_DATA_SIZE` and `APP_TX_DATA_SIZE` to at least 64
 - add at least 7 byte persistent buffer to store data from `CDC_SET_LINE_CODING` that can be output on `CDC_GET_LINE_CODING`

## Peripheral usage

ADC1: internal sensors, voltage reference
ADC2: CH3, CH13, CH17 inputs for current sense resistors, from V0.2 CH14 for VBUS sense

I2C3: I2C for communicating with the outside world
SPI2: connected to MOSFET driver and encoder ICs
USART1: external UART

TIM2: CH1 brake resistor, CH2,CH3,CH4 A,B,C PWM outputs for half bridges
TIM3: external quadrature encoder input
TIM4: CH1,CH2,CH3 LED RGB PWM output (CH4 connects to SYNC)
TIM6: Timer for performance counter with PSC=169 to get a 1us count resolution @ 170MHz clock

TIM1,TIM7,TIM8,TIM15,TIM16,TIM17: unused

# Blackmagic Probe debugging

Firmware update for probe:
`dfu-util -d 1d50:6018,:6017 -s 0x08002000:leave -D <blackmagic-native.bin>`

## GDB

```
~/Applications/ARM-Toolchain/gcc-arm-none-eabi-8-2019-q3-update/bin/arm-none-eabi-gdb
target extended-remote /dev/cu.usbmodem79A760B81
monitor swdp_scan
attach 1
```

# Board Revisions

## V0.4

### Changes
 - Change to 4-layer board for better power planes, and to de-clutter some of the signal paths.
 - Change USB-C socket footprint to be compatible two variants

## V0.3.1

### Known Bugs
 - (16.b) no zeners on VBUS/VDDP monitor taps means no chip protection

### Bugfixes
 - (15) move motor connector out by another 25mil
 - (16.a) 100n cap instead of zeners on VBUS/VDDP give much better accuracy
 - (17) add ground traces for current sense amps

### Observations
 - (17) it looks as though A and C bridges are measuring current much closer together with the dedicated ground traces



## V0.3

### Known Bugs
 - (15) motor connector does not have enough clearance for connection from back
 - (16) adding small buffer caps (100n) on voltage dividers for VBUS/VDDP would help with inaccuracy due to high-impedance source.
 - (17) current sense traces are not routed differentially, need ground lines directly to sense resistors instead of via ground plane to prevent induced effects

### Bugfixes
 - (3) change footprint of L2 from 0806 to 1005
 - (10) connects DRV8323 VREF to 3V3 plane
 - (11) connects STM32G4 VREF to 3V3 plane
 - (12) correct B1/B12 spacing on USB-C receptacle footprint
 - (13) Change D3 to ABRG and switch B/G channels on STM32G4
 - (14) move VDDDIV to PF0-ADC1_IN10
 - (8) with VDDDIV moved, re-connected hardware NSS, as STM32G4 should be able to pulse it between frames for proper DMA transfers with NSSP enabled, and add passive OR-ing resistors so that chip can be selected.

### Enhancements
 - Add 500mA polyfuse to USB 5V input to protect the input diode (and host) in case of VDD short to ground
 - change D2 to bigger footprint to be able to fit bigger diode with appropriate voltage rating

### Software Differences

Correction: Hardware NSS does NOT work because it requires opposite clock signal polarity :/

Enable hardware NSS control as master with pulsing between data words

- hspi2.Init.NSS = SPI_NSS_SOFT;
+ hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;

- hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE
+ hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE

This enables use of DMA transfers for reading data from sensors.

## V0.2

### Known Bugs
 - (10) VREF pin of DRV8323 not connected to 3V3
 - (11) VREF pin of STM32G4 not connected to 3V3
 - (12) USB-C receptacle footprint slightly wrong, spacing of B1/B12 pins too narrow for easy fit
 - (13) Wrong Pinout for D3, was RGBA should be ABRG
 - (14) sensing VDD connected to PA15 doesn't work as PA15 can only be a trigger input, not an analog channel, PF0 is the only available pin with an analog channel on ADC1

### Bugfixes
 - (1): adds bus voltage analog input to PB11, moves DRVSEL to PB12 (nixing NSS, see (8))
 - (2): connect B4/B9 pins on USB connectors
 - (5): Q8/9/10 changed be GSD in schematic, as they should be for the SSM3K35CTC, resulting in 180deg rotation
 - (6): moved C3 a nudge further away from DM via
 - (7): force thermals on J4 and J5
 - (8) Remove U2 and DRVSCS/ASCS lines and replace them directly with DRVSEL/ASEL
 - (9) Add R30 as 10K pullup on MISO


### Enhancements

#### Additional VBUS Cap

Adds two additional 10uF capacitor to VBUS, for a total of 60uF capacitance. For a 50V bus, and a 1R load, which is 50A max current, and 50kHz PWM, that means a single full PWM cycle can drain `50*20e-6 = 1000 uC`. At 50v, the cap charge is 3000uC, so the bus voltage could still drop by nearly 20V without an external buffer cap.

Same load with 10V, it's a 10A max and thus only a 100uC discharge, which is a 2V drop, or again 20% of bus voltage.

VBUS caps have been relocated so that each FET pair gets one next to it, plus one at the VBUS input.

#### VDD and VBUS sense dividers

Sensing VBUS was added for controlling the brake resistor when the voltage rises too much.

Sensing VDD is for ...?

#### Portruding power connectors

Power connectors have been moved to portrude from the 42x42mm footprint, so that the board can be mounted on the back surface or very close to the back surface of a motor without the through-hole power connectors being in danger of touching the motor casing.

## V0.1

### Known Bugs
 - (1) No feedback of bus voltage to measure it, to be able to drive brake resistor accordingly
 - (2) USB B4 and B9 not connected
 - (3) L2 probbaly has too small a footprint (BRL3225T2R2M is too large, though seems to solder ok)
 - (4) stencil cutouts too small for the high density IC pins (vreg especially), not enough solder is on pins to make good contact on reflow
 - (5) LED driver transistors Q8/9/10 are pinned wrong, should be GSD for SSM3K35CTC
 - (6) C3's 3V3 pad too close to DM via, though boards seemed to have come out fine
 - (7) VBUS pad on J4 and OR pad on J5 connector have no thermals for hand-soldering
 - (8) SPI hardware is incapable of doing word-wise NSS triggering, thus the whole logic dance with U2 may not be necessary, as we can just do software SS from GPIO
 - (9) 10K pullup might be required for DRV8323 on MISO line
 - (10) VREF pin of DRV8323 not connected to 3V3
 - (11) VREF pin of STM32G4 not connected to 3V3
 - (12) USB-C receptacle footprint slightly wrong, spacing of B1/B12 pins too narrow for easy fit
 - (13) Wrong Pinout for D3, was RGBA should be ABRG

### Status

 - USB CDC output works
 - SPI to both AD5047D and DRV8323 work after fixing bug (9)
 - all power transistors are switched under a no load condition, with DRV8323 in 3xPWM mode



