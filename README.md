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

at ~30 rev/s, we have to make 6000 steps per second. 



# Board Revisions

## V0.2
 - adds bus voltage analog input to PB11, moves DRVSEL to PC6
 - connect B4/B9 pins on USB connector
 - fix Q8/9/10 to be GSD as they should be for the SSM3K35CTC

## V0.1

### Known Bugs
 - No feedback of bus voltage to measure it, to be able to drive brake resistor accordingly
 - USB B4 and B9 not connected
 - L2 probbaly has too small a footprint (BRL3225T2R2M is too large, though seems to solder ok)
 - stencil cutouts too small for the high density ICs (vreg especially), not enough solder is on pins to make contact on reflow
 - LED driver transistors are pinned wrong, should be GSD