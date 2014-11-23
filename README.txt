########################
# USB Motor Controller
########################

0. Description

PID Motor Position Conroller with Potentiometer.
USB connected to PC and receive command of target position and gain.
Send current position to PC if requested.


1. Circuit

pin0_13: CH1 to motor controller IC (ex., TA8428)
pin0_18: CH2 to motor controller IC (ex., TA8428)
pin0_11: AnalogIn to potentio meter

2. Install 

This program setup CDC driver. Install LPCXpresso VCOM driver.
In windows, usually the driver installed in C:\NXP\Drivers

3. Protocol

Send text-base commands to COM port.

3.1 "$ping#"
ping message. Controller sends back "$ping#" command.

3.2 "$get#"
Get current position. Position is represented as A/D converter output data [0, 4095]. Controller sends back "$****#" data. Zero padding, 4 digit number. Ex., $0123#, $1234#, and $0001#.

3.3 "$set****#"
Send target psotion. Position is represented as A/D converter output data [0, 4095]. Controller sends back "$set#". Zero padding, 4 digit number. Ex., $set0123#, $set1234#, and $set0001#.

3.4 "$kp****#"
Send Proportional gain. Controller sends back "$kp#". Zero padding, 4 digit number. Ex., $kp0123#, $kp1234#, and $kp0001#.

3.5 "$ti****#"
Send Integral Time. Controller sends back "$ti#". Zero padding, 4 digit number. Ex., $ti0123#, $ti1234#, and $ti0001#.

3.6 "$td****#"
Send Derivative time. Controller sends back "$td#". Zero padding, 4 digit number. Ex., $td0123#, $td1234#, and $td0001#.
