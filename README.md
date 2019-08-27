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

at 30 rev/s, we have to make 6000 steps per second.


# STM32G431 Setup

## USB

Crystal-less USB Setup in MX
 - USB clock source should be `HSI48`
 - set `CRS SYNC` to `USB_FS` in RCC Mode & Config
 - enable `Sof Enable` in USB device settings

Full-Speed USB (12Mbps) has a max. packet size of 64 bytes for regular packets, only isochronous might be up to 1023.

### CDC
 - increase `APP_RX_DATA_SIZE` and `APP_TX_DATA_SIZE` to at least 64
 - add at least 7 byte persistent buffer to store data from `CDC_SET_LINE_CODING` that can be output on `CDC_GET_LINE_CODING`


# Board Revisions

## V0.2

### Bugfixes
 - (1): adds bus voltage analog input to PB11, moves DRVSEL to PC6
 - (2): connect B4/B9 pins on USB connector
 - (5): Q8/9/10 changed be GSD in schematic, as they should be for the SSM3K35CTC, resulting in 180deg rotation

## V0.1

### Known Bugs
 - (1) No feedback of bus voltage to measure it, to be able to drive brake resistor accordingly
 - (2) USB B4 and B9 not connected
 - (3) L2 probbaly has too small a footprint (BRL3225T2R2M is too large, though seems to solder ok)
 - (4) stencil cutouts too small for the high density ICs (vreg especially), not enough solder is on pins to make contact on reflow
 - (5) LED driver transistors Q8/9/10 are pinned wrong, should be GSD for SSM3K35CTC
