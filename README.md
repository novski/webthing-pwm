# webthing-pwm
Mozilla webthing, working with libmraa/sysfs GPIO, oneWire and PWM. Controlling a custom LED Strip.

## Webthing
Webthing is a w3 Standard in definition of iot things. This webthing follows the implementation of Mozillas iot sugestion.
https://iot.mozilla.org/schemas/
It is built on top of the webthing-python framework of mozilla:
https://github.com/mozilla-iot/webthing-python

## how to rebuild:
This depends on vaious libraries and is hard to reuse as is. 

Some facts of my system:
* Linkit Smart 7688 
* openWRT v19.02 custom build 
* libmraa
* w1

* MeanWell HLG185H-24B powersupply for a pwm controlled CC LED Strip
* DFrobot SEN0018 motion sensor
* DFrobot DFR0198 Temperature sensor (DS18b20)
* INA219 Powersensor

* a footswitch

## what it does:
It creates 3 webthings:
- LED Strip
- Motion Sensor
- Power & Temperature Sensor of the box

### LED Strip 
Has 3 functions:
- on/on
- fade brightness
- auto fade to point (action)

### Motion Sensor
Has 2 functions:
- show state of motion
- set the Delay until it auto power's off the LED Strip

### Power and Temperature
Has 4 functions:
- Power of the node in Wat
- Power of the node in Ampere
- Voltage over the node in Volt
- Temperature in the box where all is built in.

