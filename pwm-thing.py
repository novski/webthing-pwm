from __future__ import division, print_function
from webthing import (Action, Event, Property, MultipleThings, Thing, Value,
                      WebThingServer)
import ina219
import logging
import random
import time
import datetime
import uuid
import os, sys
import mraa
#from lotz for mraa interrupt traceback.
#import traceback
from tornado.ioloop import IOLoop, PeriodicCallback

location = 'og-vorraum'
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
            logging.info('FadeLedStrip.perform_action: start incrementing, brightness_now from %s to brightness_afer %s where percentage range is %s and per step of delay_ms: %s while total duration is %s',
                         brightness_now,
                         brightness_after,
                         percentage,
                         delay_ms,
                         self.input['duration'])
            while brightness_after > self.thing.get_property('brightness'):
                self.thing.set_property('brightness', 
                                        self.thing.get_property('brightness')+1)
                time.sleep(delay_ms / 1000)
        elif brightness_after < brightness_now:
            percentage = brightness_now - brightness_after
            delay_ms = self.input['duration'] / percentage
            while brightness_after < self.thing.get_property('brightness'):
                self.thing.set_property('brightness', 
                                        self.thing.get_property('brightness')-1)
                time.sleep(delay_ms / 1000)
        return

class LedStrip(Thing):
    """A LED-Strip in the entry of my upper Level"""
    
    def __init__(self, location):
        self.id = f'urn:dev:ops:{location}-vorraum-led'
        self.name = f'{location}-LED_Strip'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['OnOffSwitch', 'Light'],
            'A web connected LED-Strip'
        )
        self.pwm = mraa.Pwm(26)             # GPIO18 (P26) pwm on Linkit Smart 7688
        self.pwm.period_us(20000)           # 20ms period ==> 50Hz
        self.pwm.enable(True)               # Start sending PWM signal

        self.relay2 = mraa.Gpio(18)          # relay2 (P18) on Linkit Smart 7688
        self.relay1 = mraa.Gpio(19)          # relay1 (P19) on Linkit Smart 7688
        self.relay2.dir(mraa.DIR_OUT)        # set as OUTPUT pin
        self.relay1.dir(mraa.DIR_IN)         # set as INPUT pin, its a pullup, so its High when the switch is open.
        self.relay1_read()                   # get initial Value of Relay1
        self.r1_previous = False

        
        logging.debug('add property digitalswitch..')
        self.state = Value(self.get_state(),self.toggle_digitalswitch)
        self.add_property(
            Property(self,
                     f'{self.name}-digitalswitch',
                     self.state,
                     metadata={
                         '@type': 'OnOffProperty',
                         'title': f'{self.name}-On/Off',
                         'type': 'boolean',
                         'description': 'Whether the Strip is turned on',
                     }
            )
        )

        self.add_property(
            Property(self,
                     f'{self.name}-brightness',
                     Value(self.get_brightness(),self.set_brightness),
                     metadata={
                         '@type': 'BrightnessProperty',
                         'title': f'{self.name}-Brightness',
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
            'title': f'{self.name}-Fade_Action',
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

        self.timer = PeriodicCallback(
                self.get_r1,
                1000
            )
        self.timer.start()

    def get_brightness(self):
        """get the level from mraa.pwm"""
        brightness = round(self.pwm.read() * 100,2)
        return brightness

    def set_brightness(self, brightness):
        """set the level with mraa"""
        level = round((brightness/100),2)
        self.pwm.write(level)
        on_off = self.get_property('digitalswitch')
        if brightness < 1 and on_off == True:
            self.set_property('digitalswitch', False)
        elif brightness> 1 and on_off == False: 
            self.set_property('digitalswitch', True)
        return brightness

    def relay1_read(self):
        r1 = bool(self.relay1.read())
        if r1 == True:
            r1 = False
        elif r1 == False:
            r1 = True
        return r1

    def get_r1(self):
        """ only if footswitch changes do more """
        r1 = self.relay1_read()
        if r1 != self.r1_previous:
            self.state.notify_of_external_update(self.get_state())
        self.r1_previous = r1

    def get_state(self):
        """
        The manual switch is connected to a multiway switch made of two Relays 
        with 3-ways. 
        https://en.wikipedia.org/wiki/Multiway_switching#Traveler_system
        """
        footswitch = self.relay1_read()          # relay 1
        digitalswitch = bool(self.relay2.read())       # relay 2
        orb = footswitch ^ digitalswitch
        logging.debug('LedStrip.get_state: footswitch %s, digitalswitch %s, orb %s', footswitch, digitalswitch,orb)
        return orb

    def toggle_digitalswitch(self,state):
        """
        gateway or other GUI representaton of switch to turn on/off LedStrip.
        """
        logging.debug('LedStrip.toggle_digitalswitch:%s',state)
        self.relay2.write(not self.relay2.read())

    def cancel_update_state_task(self):
        logging.info('stopping the status update loop task')
        self.timer.stop()

class PirSensor(Thing):
    """ a PIR sensor implementation """

    def __init__(self,location):
        self.id = f'urn:dev:ops:{location}-pir_sensor'
        self.name = f'{location}-pir_sensor'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['MotionSensor'],
            'A PIR Motion Sensor'
        )
        self.pir_sensor = mraa.Gpio(13)      # GPIO 1 (P13) on Linkit Smart 7688
        self.pir_sensor.dir(mraa.DIR_IN)     # set as INPUT pin
        self.pir_sensor.isr(mraa.EDGE_BOTH, PirSensor.pir_motion_detected, self)
        """
        figured out that mt7688an has onls 8 interrupts and GPIO13 and 18 get
        triggered same time because of a electic pinmux that is not handled
        in kernel. There might be more pin's triggering same time.
        todo: [] check for pin's triggering simultaneously with others. 
              [] previous, build a test breadboard.
        """
        self.turn_off_timeout = None
        self.pir_timeout = 15

        self.add_property(
            Property(self,
                     f'{self.name}-pir motion sensor',
                     Value(self.get_motion(),self.pir_motion_detected),
                     metadata={
                         '@type': 'MotionProperty',
                         'title': f'{self.name}-PIR_Motion_Sensor',
                         'type': 'boolean',
                         'description': 'motion=true, nomotion=false',
                     }
            )
        )
    def get_motion(self):
        pass

    def pir_motion_detected(self):
        if not self.pir_sensor.read():
            return
        logging.debug(f'pir_motion_detected: value1: {self.pir_sensor.read()} at {datetime.datetime.now()}')
        io_loop = IOLoop.current()
        logging.debug(f'pir_motion_detected: value2: {self.pir_sensor.read()} at {datetime.datetime.now()}')
        io_loop.add_callback(PirSensor.pir_event_waiter,self)
        logging.debug(f'pir_motion_detected: value3: {self.pir_sensor.read()} at {datetime.datetime.now()}')

    def pir_event_waiter(self):
        logging.debug(f'pir_event_waiter:entered')
        io_loop = IOLoop.current()
        if self.turn_off_timeout:
            io_loop.remove_timeout(self.turn_off_timeout)
        self.turn_off_timeout = io_loop.call_later(self.pir_timeout, PirSensor.pir_turn_off, self)
        logging.debug('turn light ON')

    def pir_turn_off(self):
        self.pir_timeout = None
        logging.debug('turn light OFF')

    def cancel_pir_interrupt(self):
        self.pir_sensor.isrExit()


class VorraumNode(Thing):
    """A node in the entry of my upper Level"""
    def __init__(self,location):
        self.id = f'urn:dev:ops:{location}-vorraum-node'
        self.name = f'{location}-node_functions'
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
                     f'{self.name}-power',
                     self.power,
                     metadata={
                         '@type': 'InstantaneousPowerProperty',
                         'title': f'{self.name}-Power',
                         'type': 'float',
                         'multipleOf': 0.01,
                         'description': 'the power used by this node',
                     }
            )
        )
        self.volt = Value(0.0)
        self.add_property(
            Property(self,
                     f'{self.name}-voltage',
                     self.volt,
                     metadata={
                         '@type': 'VoltageProperty',
                         'title': f'{self.name}-Voltage',
                         'type': 'float',
                         'multipleOf': 0.01,
                         'description': 'the voltage used by this node',
                     }
            )
        )
        self.current = Value(0.0)
        self.add_property(
            Property(self,
                     f'{self.name}-current',
                     self.current,
                     metadata={
                         '@type': 'CurrentProperty',
                         'title': f'{self.name}-Current',
                         'type': 'float',
                         'multipleOf': 0.01,
                         'description': 'the current used by this node',
                     }
            )
        )
        self.temperature = Value(0.0)
        self.add_property(
            Property(self,
                     f'{self.name}-Temperature',
                     self.temperature,
                     metadata={
                         '@type': 'TemperatureProperty',
                         'title': f'{self.name}-Temperature',
                         'type': 'float',
                         'multipleOf': 0.01,
                         'description': 'the temperature in the case',
                     }
            )
        )
        self.timer = PeriodicCallback(
                self.get_sensors,
                300000
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
            logging.debug('get_power:milliwats %s',milliwats)
        except AssertionError as e:
            logging.error(repr(e))

    def get_temperature(self):
        if len(w1_device_id_list) > 1:
            logging.error("get_temperature:more than one w1 device detected")
            return
        if len(w1_device_id_list) < 1:
            logging.error("get_temperature:no w1 device detected")
            return 
        for device in w1_device_id_list:
            logging.info("get_temperature:reading w1 device")
            temperature = self.get_temperature_read_one(device)
            self.temperature.notify_of_external_update(temperature)

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
    led_strip = LedStrip(location)
    pir_sensor = PirSensor(location)
    get_w1_devices()
    node_functions = VorraumNode(location)
    node_functions.get_sensors()
    # If adding more than one thing, use MultipleThings() with a name.
    # In the single thing case, the thing's name will be broadcast.
    server = WebThingServer(MultipleThings([led_strip,
                                            pir_sensor,
                                            node_functions],
                                           'LightAndTempDevice'), port=8888)
    try:
        logging.info('starting the server')
        server.start()
    except KeyboardInterrupt:
        led_strip.cancel_update_state_task()
        pir_sensor.cancel_pir_interrupt()
        logging.info('stopping the server')
        server.stop()
        logging.info('done')


if __name__ == '__main__':
    logging.basicConfig(
        level=10,
        format="%(asctime)s %(filename)s:%(lineno)s %(levelname)s %(message)s"
    )
    run_server()

