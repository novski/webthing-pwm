from __future__ import division, print_function
from webthing import (Action, Event, Property, MultipleThings, Thing, Value,
                      WebThingServer)
import ina219
import logging
import random
import time
import tornado.ioloop
import uuid
import os, sys
import mraa


pwm0 = mraa.Pwm(26)             # GPIO18 (P26) PWM0
#pwm0.period(0.001)             # as seconds in float
pwm0.period_us(20000)           # 20ms period ==> 50Hz
pwm0.enable(True)               # Start sending PWM signal

gpio13 = mraa.Gpio(18)          # gpio13 PIN P18 on Linkit Smart 7688 to Relay2
gpio12 = mraa.Gpio(19)          # gpio12 PIN P19 on Linkit Smart 7688 from Relay1
gpio13.dir(mraa.DIR_OUT)        # set as OUTPUT pin
gpio12.dir(mraa.DIR_IN)         # set as INPUT pin, its a pullup, so its High when the switch is open.


r1_previous = False
w1_device_id_list = []


def get_w1_devices():
    """
    Sensor initializeing, get the ammount of 
    devices connected to the w1-bus.
    """
    try:
        for x in os.listdir("/sys/bus/w1/devices"):
            if "master" in x:
                continue
            w1_device_id_list.append(x)
        logging.debug('list of devices found: %s ', w1_device_id_list)
    except:
        logging.debug('get_w1_devices: no devices found')
    

class FadeLedStrip(Action):

    def __init__(self, thing, input_):
        Action.__init__(self, uuid.uuid4().hex, thing, 'fade', input_=input_)

    def perform_action(self):
        brightness_after = self.input['brightness']
        brightness_now = self.thing.get_property('brightness')
        if  brightness_after > brightness_now :
            percentage = brightness_after - brightness_now
            delay_ms = self.input['duration'] / percentage
            logging.debug('FadeLedStrip.perform_action: start incrementing, brightness now %s, brightness afer %s, percentage %s, delay_ms %s, duration %s',
                         brightness_now,
                         brightness_after,
                         percentage,
                         delay_ms,
                         self.input['duration'])
            while brightness_after > self.thing.get_property('brightness'):
                self.thing.set_property('brightness', 
                                        self.thing.get_property('brightness')+1)
                time.sleep(delay_ms / 1000)
            logging.debug('finished incrementing')
        elif brightness_after < brightness_now:
            percentage = brightness_now - brightness_after
            delay_ms = self.input['duration'] / percentage
            logging.debug('start decrementing, brightness now %s, brightness afer %s, percentage %s, delay_ms %s, duration %s',
                        brightness_now,
                        brightness_after,
                        percentage,delay_ms,
                        self.input['duration'])
            while brightness_after < self.thing.get_property('brightness'):
                self.thing.set_property('brightness', 
                                        self.thing.get_property('brightness')-1)
                time.sleep(delay_ms / 1000)
            logging.debug('FadeLedStrip.perform_action: finished decrementing')
        return

class LedStrip(Thing):
    """A LED-Strip in the entry of my upper Level"""

    def __init__(self):
        self.id = 'urn:dev:ops:og-vorraum-vorraum-led'
        self.name = 'LED-Strip'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['OnOffSwitch', 'Light'],
            'A web connected LED-Strip'
        )

        self.state = Value(self.get_state(),self.toggle_digitalswitch)
        logging.debug('LedStrip:digitalswitch, get_property(self): %s',self.state.last_value)
        self.add_property(
            Property(self,
                     'digitalswitch',
                     self.state,
                     metadata={
                         '@type': 'OnOffProperty',
                         'title': 'LED On/Off',
                         'type': 'boolean',
                         'description': 'Whether the Strip is turned on',
                     }
            )
        )

        self.add_property(
            Property(self,
                     'brightness',
                     Value(self.get_brightness(),self.set_brightness),
                     metadata={
                         '@type': 'BrightnessProperty',
                         'title': 'Brightness',
                         'type': 'integer',
                         'description': 'The level of light from 0-100',
                         'minimum': 0,
                         'maximum': 100,
                         'unit': 'percent',
                     }
            )
        )

        self.add_available_action('fade',
        {
            'title': 'Fade',
            'description': 'Fade the lamp to a given level',
            'input': {
                'type': 'object',
                'required': [
                    'brightness',
                    'duration',
                ],
                'properties': {
                    'brightness': {
                        'type': 'integer',
                        'minimum': 0,
                        'maximum': 100,
                        'unit': 'percent',
                    },
                    'duration': {
                        'type': 'integer',
                        'minimum': 1,
                        'unit': 'milliseconds',
                    },
                },
            },
        },
        FadeLedStrip)

        self.timer = tornado.ioloop.PeriodicCallback(
                self.get_r1,
                1000
            )
        self.timer.start()

    def get_brightness(self):
        """get the level from mraa.pwm"""
        brightness = round(pwm0.read() * 100,2)
        return brightness

    def set_brightness(self, brightness):
        """set the level with mraa"""
        level = round((brightness/100),2)
        pwm0.write(level)
        on_off = self.get_property('digitalswitch')
        if brightness < 1 and on_off == True:
            self.set_property('digitalswitch', False)
        elif brightness> 1 and on_off == False: 
            self.set_property('digitalswitch', True)
        return brightness

    def gpio12_read(self):
        r1 = bool(gpio12.read())
        if r1 == True:
            r1 = False
        elif r1 == False:
            r1 = True
        return r1

    def get_r1(self):
        """ only if footswitch changes do more """
        global r1_previous
        r1 = self.gpio12_read()
        if r1 != r1_previous:
            logging.debug(' ')
            logging.debug('LedStrip.get_r1 changed: r1 %s, r1_previous %s', r1, r1_previous) 
            self.state.notify_of_external_update(self.get_state())
        r1_previous = r1

    def get_state(self):
        """
        The manual switch is connected to a multiway switch made of two Relays 
        with 3-ways. 
        https://en.wikipedia.org/wiki/Multiway_switching#Traveler_system
        """
        footswitch = self.gpio12_read()          # relay 1
        digitalswitch = bool(gpio13.read())       # relay 2
        orb = footswitch ^ digitalswitch
        logging.debug('LedStrip.get_state: footswitch %s, digitalswitch %s, orb %s', footswitch, digitalswitch,orb)
        return orb

    def toggle_digitalswitch(self,state):
        """
        gateway or other GUI representaton of switch to turn on/off LedStrip.
        """
        logging.debug('LedStrip.toggle_digitalswitch: state1 %s', state)
        gpio13.write(not gpio13.read())

    def cancel_update_state_task(self):
        logging.info('stopping the status update loop task')
        self.timer.stop()

class vorraum_node(Thing):
    """A node in the entry of my upper Level"""
    def __init__(self):
        self.id = 'urn:dev:ops:og-vorraum-vorraum-node'
        self.name = 'node-og-vorraum'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['EnergyMonitor'],
            'A web connected node'
        )
        self.ina219 = ina219.INA219(busnum=0,address=0x40)
        self.power = Value(0.0)
        self.add_property(
            Property(self,
                     'Power',
                     self.power,
                     metadata={
                         '@type': 'InstantaneousPowerProperty',
                         'title': 'Power',
                         'type': 'float',
                         'description': 'the power used by this node',
                     }
            )
        )
        self.volt = Value(0.0)
        self.add_property(
            Property(self,
                     'Voltage',
                     self.volt,
                     metadata={
                         '@type': 'VoltageProperty',
                         'title': 'Voltage',
                         'type': 'float',
                         'description': 'the voltage used by this node',
                     }
            )
        )
        self.current = Value(0.0)
        self.add_property(
            Property(self,
                     'Current',
                     self.current,
                     metadata={
                         '@type': 'CurrentProperty',
                         'title': 'Current',
                         'type': 'float',
                         'description': 'the current used by this node',
                     }
            )
        )
        self.temperature = Value(0.0)
        self.add_property(
            Property(self,
                     'Temperature',
                     self.temperature,
                     metadata={
                         '@type': 'TemperatureProperty',
                         'title': 'Temperature',
                         'type': 'float',
                         'multipleOf': 0.01,
                         'description': 'the temperature in the case',
                     }
            )
        )
        self.timer = tornado.ioloop.PeriodicCallback(
                3000000
                self.get_sensors,
            )
        self.timer.start()

    def get_sensors(self):
        logging.debug('get_sensors')
        self.get_power()
        self.get_temperature()

    def get_power(self):
        try:
            bus_voltage     = self.ina219.get_bus_voltage_mV()
            self.volt.notify_of_external_update(bus_voltage / 1000)
            curent          = self.ina219.get_current_mA()
            self.current.notify_of_external_update(curent / 1000)
            milliwats       = self.ina219.get_power_mW()
            self.power.notify_of_external_update(milliwats / 1000)
        except AssertionError as e:
            logging.error(repr(e))

    def get_temperature(self):
        if len(w1_device_id_list) > 1:
            logging.error("more than one w1 device detected")
            return
        if len(w1_device_id_list) < 1:
            logging.error("no w1 device detected")
            return
        for device in w1_device_id_list:
            temperature = self.get_temperature_read_one(device)
            self.temperature.notify_of_external_update(temperature)
        return

    def get_temperature_read_one(self, temp_sensor_id):
        try:
            """ read 1-wire slave file from a specific device """
            file_name = "/sys/bus/w1/devices/" + temp_sensor_id + "/w1_slave"
            file = open(file_name)
            filecontent = file.read()
            file.close()
            """ read temperature values and convert to readable float """
            stringvalue = filecontent.split("\n")[1].split(" ")[9]
            sensorvalue = float(stringvalue[2:]) / 1000
            temperature = '%6.2f' % sensorvalue
            return temperature
        except:
            logging.debug("get_temperature_read_one: not able to read from: %s",temp_sensor_id)

    def cancel_update_node_task(self):
        logging.info('stopping the node update loop task')
        self.timer.stop()

def run_server():
    vorraum_led = LedStrip()
    get_w1_devices()
    vorraum_functions = vorraum_node()

    vorraum_functions.get_sensors()
    # If adding more than one thing, use MultipleThings() with a name.
    # In the single thing case, the thing's name will be broadcast.
    server = WebThingServer(MultipleThings([vorraum_led,vorraum_functions],'LightAndTempDevice'), port=8888)
    try:
        logging.info('starting the server')
        server.start()
    except KeyboardInterrupt:
        vorraum_led.cancel_update_state_task()
        logging.info('stopping the server')
        server.stop()
        logging.info('done')


if __name__ == '__main__':
    logging.basicConfig(
        level=10,
        format="%(asctime)s %(filename)s:%(lineno)s %(levelname)s %(message)s"
    )
    run_server()

