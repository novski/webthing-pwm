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
* smb
* a custom pcb for manual power switching beside iot software api 

* MeanWell HLG185H-24B powersupply for a pwm controlled CC LED Strip
* Welleman VMA 314 motion sensor
* DFrobot DFR0198 Temperature sensor (DS18b20)
* INA219 Powersensor

* a footswitch 230V 10A

## what it does:
It creates 3 webthings:
- LED Strip
- Motion Sensor
- Power & Temperature Sensor of the box

### LED Strip 
Has 5 functions:
- on/off
- fade brightness
- auto fade to point (action)
- activate / deactivate motion handling
- set the delay until it auto power's off the LED Strip

### Motion Sensor
Has 2 functions:
- show state of motion
- directly trigger LED-Strip async


### Power and Temperature
Has 4 functions:
- Power of the node in Watt
- Power of the node in Ampere
- Voltage over the node in Volt
- Temperature in the box where all is built in.

