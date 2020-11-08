from __future__ import division, print_function
from webthing import (Action, Event, Property, MultipleThings, Thing, Value,
                      WebThingServer)
import logging
import random
import time
import tornado.ioloop
import uuid
import os
import sys
from datetime import datetime, timedelta
import mraa
import ina219
import pdb
import threading


class FadeLedStrip(Action):

    def __init__(self, thing, input_):
        Action.__init__(self, uuid.uuid4().hex, thing, 'fade', input_=input_)
        self.thing.get_property('on')  # TODO: check if needed.

    def perform_action(self):
        self.brightness_after = self.input['brightness']
        self.duration = self.input['duration']
        self.brightness_now = self.thing.get_property('brightness')
        if self.brightness_after > self.brightness_now:
            self.percentage = self.brightness_after - self.brightness_now
            self.delay_ms = self.duration / self.percentage
            logging.info(f'FadeLedStrip.perform_action: start incrementing,'
                         'brightness_now from {self.brightness_now} '
                         'to brightness_afer {self.brightness_after} '
                         'where percentage range is {self.percentage} '
                         'and per step of delay_ms: {self.delay_ms} '
                         'while total duration is {self.duration} ms'
                         )
            while self.brightness_after > self.thing.get_property('brightness'):
                self.thing.set_property('brightness',
                                        self.thing.get_property('brightness')+1)
                time.sleep(self.delay_ms / 1000)
        elif self.brightness_after < self.brightness_now:
            self.percentage = self.brightness_now - self.brightness_after
            self.delay_ms = self.duration / self.percentage
            while self.brightness_after < self.thing.get_property('brightness'):
                self.thing.set_property('brightness',
                                        self.thing.get_property('brightness')-1)
                time.sleep(self.delay_ms / 1000)
        return


class LedStrip(Thing):
    """A LED-Strip in the entry of my upper Level"""

    def __init__(self, _location, _main_loop):
        self.location = _location
        self.main_loop = _main_loop
        self.id = f'urn:dev:ops:{self.location}-vorraum-led'
        self.name = f'{self.location}-LED_Strip'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['OnOffSwitch', 'Light'],
            'A web connected LED-Strip'
        )
        # GPIO18 (P26) pwm on Linkit Smart 7688
        self.pwm = mraa.Pwm(26)
        self.pwm.period_us(20000)           # 20ms period ==> 50Hz
        self.pwm.enable(True)               # Start sending PWM signal

        # relay2 (P18) on Linkit Smart 7688
        self.relay2 = mraa.Gpio(18)
        # relay1 (P19) on Linkit Smart 7688
        self.relay1 = mraa.Gpio(19)
        self.relay2.dir(mraa.DIR_OUT)        # set as OUTPUT pin
        # set as INPUT pin, its a pullup, so its High when the switch is open.
        self.relay1.dir(mraa.DIR_IN)
        self.relay1_read()                   # get initial Value of Relay1
        self.r1_previous = False

        self.main_loop_time = None
        self.async_timeout = None

        self.day_time_start = datetime(2020, 6, 20, 8, 00, 00, 000000).time()
        self.day_time_stop = datetime(2020, 6, 20, 11, 00, 00, 000000).time()

        self.state = Value(self.get_state(), self.toggle_digitalswitch)
        self.add_property(
            Property(self,
                     'digitalswitch',
                     self.state,
                     metadata={
                         '@type': 'OnOffProperty',
                         'title': f'{self.name}-digitalswitch',
                         'type': 'boolean',
                         'description': 'Whether the Strip is turned on',
                     }
                     )
        )

        self.add_property(
            Property(self,
                     'brightness',
                     Value(self.get_brightness(), self.set_brightness),
                     metadata={
                         '@type': 'BrightnessProperty',
                         'title': 'Helligkeit',
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
                                      'title': 'Helligkeitswert über eine Zeitdauer einstellen',
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

        self.delay_minutes = Value(3)
        self.add_property(
            Property(self,
                     'motion_sensor_delay',
                     self.delay_minutes,
                     metadata={
                         '@type': 'LevelProperty',
                         'title': 'Verzögertes Aus bei Bewegung (in Minuten)',
                         'type': 'integer',
                         'description': 'The delay in minutes',
                         'minimum': 0,
                         'maximum': 20,
                         'unit': 'minutes',
                     }
                     )
        )

        self.motion_on_off = Value(True)
        self.add_property(
            Property(self,
                     'Motiondetection-Active',
                     self.motion_on_off,
                     metadata={
                         '@type': 'BooleanProperty',
                         'title': 'Auf Bewegung reagieren?',
                         'type': 'boolean',
                         'description': 'Whether the motion sensor is active or not',
                     }
                     )
        )

        self.add_property(
            Property(self,
                     'motion_detection_follower',
                     Value(self.get_motion(), self.set_motion),
                     metadata={
                         '@type': 'MotionProperty',
                         'title': 'Bewegungs Sensor',
                         'type': 'boolean',
                         'description': 'motion=true, nomotion=false',
                     }
                     )
        )

        self.timer = tornado.ioloop.PeriodicCallback(
            self.get_r1,
            1000
        )
        self.timer.start()

    def get_brightness(self):
        """get the level from mraa.pwm"""
        self.brightness = round(self.pwm.read() * 100, 2)
        return self.brightness

    def set_brightness(self, brightness):
        """set the level with mraa"""
        self.level = round((brightness/100), 2)
        self.pwm.write(self.level)
        self.on_off = self.get_property('digitalswitch')
        if self.brightness < 1 and self.on_off == True:
            self.set_property('digitalswitch', False)
        elif self.brightness >= 1 and self.on_off == False:
            self.set_property('digitalswitch', True)
        logging.info(f'Set Brightness to {self.brightness}')
        return self.brightness

    def relay1_read(self):
        self.relay1_value = bool(self.relay1.read())
        if self.relay1_value == True:
            self.relay1_value = False
        elif self.relay1_value == False:
            self.relay1_value = True
        return self.relay1_value

    def get_r1(self):
        """ only if footswitch changes """
        self.r1 = self.relay1_read()
        if self.r1 != self.r1_previous:
            self.state.notify_of_external_update(self.get_state())
        self.r1_previous = self.r1

    def get_state(self):
        """
        The manual switch is connected to a multiway switch made of two Relays 
        with 3-ways. 
        https://en.wikipedia.org/wiki/Multiway_switching#Traveler_system
        """
        self.footswitch = bool(self.relay1_read())                 # relay 1
        self.digitalswitch = bool(self.relay2.read())        # relay 2
        return self.footswitch ^ self.digitalswitch

    def toggle_digitalswitch(self, state):
        """
        switch to turn on/off LedStrip.
        """
        self.relay2.write(not self.relay2.read())

    def motion(self):
        """ MOTION SENSOR calls this when ever it triggers """
        logging.debug('Motion called')
        self.date_time_now = datetime.now().time()
        logging.debug(f'Motion called self.date_time_now:{self.date_time_now}')
        self.start = self.day_time_start
        logging.debug(f'Motion called self.start:{self.start}')
        self.end = self.day_time_stop
        logging.debug(f'Motion called self.end:{self.end}')
        if self.date_time_now > self.start and self.date_time_now < self.end:
            self.day_time = True
            logging.debug('Motion called during the Day')
        else:
            self.day_time = False
            logging.debug('Motion called in the Night')
        if not self.day_time:
            logging.debug('Motion called on not day_time')
            if self.motion_on_off:
                logging.debug(
                    'Motion called on not day_time and Motion was activated, add_callback fires now')
                self.main_loop.add_callback(self.interrupt_call_back)

    def interrupt_call_back(self):
        logging.info(
            f'interrupt_call_back called, async_timeout:{self.async_timeout}')
        if self.async_timeout:
            logging.debug(
                f'timeout was present, async_timeout:{self.async_timeout}')
            self.main_loop.remove_timeout(self.async_timeout)
            logging.debug(
                f'timeout was removed, async_timeout:{self.async_timeout}')
        self.main_loop_time = self.main_loop.time()
        self.delay_time = self.get_delay_time()
        self.sw = self.get_property('digitalswitch')
        if not self.sw:
            logging.debug('digitalswitch was OFF light now turns ON')
            self.set_property('digitalswitch', True)
        self.timeout_delta = self.main_loop_time + self.delay_time
        self.timeout_delta_human = time.strftime(
            '%H:%M:%S', time.gmtime(self.timeout_delta))
        logging.debug(
            f'Light will turn OFF at: {self.timeout_delta}, or readable: {self.timeout_delta_human}')
        self.async_timeout = self.main_loop.add_timeout(
            self.timeout_delta, self.interrupt_call_timeout)

    def interrupt_call_timeout(self):
        logging.info('interrupt_call_timeout called')
        self.swi = self.get_property('digitalswitch')
        self.foot_swi = self.relay1_read()
        if (self.swi) and (not self.foot_swi):
            self.set_property('digitalswitch', False)
            self.swi = self.get_property('digitalswitch')
        logging.info(
            f'interrupt_call_timeout async_timeout: {self.async_timeout}')
        self.main_loop.remove_timeout(self.async_timeout)
        logging.info(
            f'interrupt_call_timeout async_timeout removed: {self.async_timeout}')
        self.async_timeout = False

    def get_delay_time(self):
        self.delay_minutes = self.get_property('motion_sensor_delay')
        self.delay_seconds = self.delay_minutes * 60
        return self.delay_seconds

    def get_motion(self):
        logging.debug("get_motion_follower")
        if self.get_property('motion_detection_follower') is None:
            logging.debug("get_motion_follower is None")
            self.set_property('motion_detection_follower', False)
        return

    def set_motion(self, value):
        logging.debug(f'set_motion_follower (LedStrip): {value}')
        self.motion()
        logging.debug(f'set_motion_follower (LedStrip): {value}')
        return

    """ END OF LED STRIP THING """

    def cancel_led_strip_async_tasks(self):
        logging.info('stopping the status update loop task')
        self.timer.stop()


class MotionSensor(Thing):
    """A PIR Sensor in the entry of my upper Level"""

    def __init__(self, _location,
                 _main_loop,
                 _led_strip
                 ):
        self.location = _location
        self.main_loop = _main_loop
        self.led_strip = _led_strip
        self.id = f'urn:dev:ops:{self.location}-motion-sensor'
        self.name = f'{self.location}-motion-sensor'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['MotionSensor'],
            'A web connected motion sensor'
        )
        """ MOTION SENSOR PIN DEFINITION """
        self.motion_sensor = mraa.Gpio(13)  # GPIO 1 (P13) on Linkit Smart 7688
        self.motion_sensor.dir(mraa.DIR_IN) # set as INPUT pin
        self.motion_sensor.isr(
            mraa.EDGE_RISING, MotionSensor.interrupt_call, self)
        self.motion_sensor.isr(mraa.EDGE_FALLING, MotionSensor.timeout, self)
        """
        I figured out that mt7688an has onls 8 interrupts and GPIO13 and 18 get
        triggered same time because of a electic pinmux that is not handled
        in kernel. There might be more pin's triggering same time.
        It can be avoided by checking the pin that is interrupted with the pin
        that is whanted to be HIGH. In this case 'interrupt_call' first checks 
        if pin13 is High whenever a interrupt acures.
        And falling edge interrupt does not trigger. 
        Have to workaround a motion reset with async timeout.
        """

        # self.motion =
        self.add_property(
            Property(self,
                     'motion_detection',
                     Value(self.get_motion(), self.set_motion),
                     metadata={
                         '@type': 'MotionProperty',
                         'title': 'Bewegungs Sensor',
                         'type': 'boolean',
                         'description': 'motion=true, nomotion=false',
                     }
                     )
        )

    def get_motion(self):
        logging.debug('get_motion')
        if self.get_property('motion_detection') is None:
            logging.debug("get_motion is None")
            self.set_property('motion_detection', False)
        logging.debug(f'currently {threading.enumerate()} '
            f'are active while {threading.activeCount() } are active '
            f'and we are in {threading.currentThread()}')

    def set_motion(self, value):
        logging.debug(f'set_motion: {value}')

    def interrupt_call(self):
        logging.debug(f'currently {threading.enumerate()} '
            f'are active while {threading.activeCount() } are active '
            f'and we are in {threading.currentThread()}')
        sys.exit()
        sys.exit()
        logging.debug(f'currently {threading.enumerate()} '
            f'are active while {threading.activeCount() } are active '
            f'and we are in {threading.currentThread()}')
        logging.debug('interrupt_call')
        if self.motion_sensor.read():
            logging.debug('interrupt_call read True')
            self.set_property('motion_detection', True)
            self.led_strip.set_property('motion_detection_follower', True)
            logging.debug('interrupt_call back')
            self.main_loop_time = self.main_loop.time()
            self.n = time.strftime(
                '%H:%M:%S', time.gmtime(self.main_loop_time))
            logging.debug(
                f'main_loop_time at {self.main_loop_time} or {self.n}')
            self.delay_in_sec = 4
            logging.debug(f'delay_in_sec: {self.delay_in_sec}')
            self.time_when_off = self.main_loop_time + self.delay_in_sec
            logging.debug(f'time_when_off at {self.time_when_off}')
            self.t = time.strftime('%H:%M:%S', time.gmtime(self.time_when_off))
            logging.debug(f'timeout at {self.t}')
            self.main_loop.add_timeout(self.time_when_off, self.timeout)

    def timeout(self):
        logging.debug('motion sensor interrupt Falling edge or timeout')
        self.set_property('motion_detection', False)
        self.led_strip.set_property('motion_detection_follower', False)

    """ END OF MOTION SENSOR THING """

    def cancel_motion_sensor_async_tasks(self):
        logging.info('stopping the interrupt')
        self.motion_sensor.isrExit()


class VorraumNode(Thing):
    """A node in the entry of my upper Level"""

    def __init__(self, _location, _w1_device_id_list):
        self.location = _location
        self.id = f'urn:dev:ops:{self.location}-vorraum-node'
        self.name = f'{self.location}-node_functions'
        self.w1_device_id_list = _w1_device_id_list
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['EnergyMonitor'],
            'A web connected node'
        )
        self.ina219 = ina219.INA219(busnum=0, address=0x40)
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
        self.timer = tornado.ioloop.PeriodicCallback(
            self.get_sensors,
            300000
        )
        self.timer.start()

    def get_sensors(self):
        self.get_power()
        self.get_temperature()

    def get_power(self):
        logging.info("get_power")
        try:
            bus_voltage = self.ina219.get_bus_voltage_mV()
            self.volt.notify_of_external_update(bus_voltage / 1000)
            curent = self.ina219.get_current_mA()
            self.current.notify_of_external_update(curent / 1000)
            milliwats = self.ina219.get_power_mW()
            self.power.notify_of_external_update(milliwats / 1000)
        except AssertionError as e:
            logging.error(repr(e))

    def get_temperature(self):
        logging.info('get_temperature')
        number_of_device_ids = len(self.w1_device_id_list)
        if number_of_device_ids < 1:
            logging.error('get_temperature:no w1 device detected')
            return
        for device in self.w1_device_id_list:
            logging.info(
                f'get_temperature:reading w1 device {device}. {number_of_device_ids} to been read')
            temperature = self.get_temperature_read_one(device)
            self.temperature.notify_of_external_update(temperature)

    def get_temperature_read_one(self, temp_sensor_id):
        try:
            logging.info(f'get_temperature_read_one of {temp_sensor_id}')
            # read 1-wire slave file from a specific device
            file_name = "/sys/bus/w1/devices/" + temp_sensor_id + "/w1_slave"
            file = open(file_name)
            filecontent = file.read()
            file.close()
            # read temperature values and convert to readable float
            stringvalue = filecontent.split("\n")[1].split(" ")[9]
            sensorvalue = float(stringvalue[2:]) / 1000
            temperature = '%6.2f' % sensorvalue
            return temperature
        except:
            logging.warn(
                f'get_temperature_read_one: not able to read from: {temp_sensor_id}')

    def cancel_node_async_tasks(self):
        logging.info('stopping the node update loop task')
        self.timer.stop()


def get_w1_devices():
    """
    Sensor initializeing, get the ammount of 
    devices connected to the w1-bus.
    """
    w1_device_id_list = []
    try:
        for x in os.listdir('/sys/bus/w1/devices'):
            if 'master' in x:
                continue
            w1_device_id_list.append(x)
        return w1_device_id_list
    except:
        logging.warn('get_w1_devices: no devices found')

def run_server():
    location = 'og-vorraum'
    w1_device_id_list = get_w1_devices()
    main_loop = tornado.ioloop.IOLoop.current()
    node_functions = VorraumNode(location,w1_device_id_list)
    node_functions.get_sensors()
    led_strip = LedStrip(location,
                         main_loop,
                         )
    motion_sensor = MotionSensor(location,
                                 main_loop,
                                 led_strip,
                                 )

    server = WebThingServer(MultipleThings([node_functions,
                                            led_strip,
                                            motion_sensor,
                                            ],
                                           'LightAndTempDevice'), port=8888)
    try:
        logging.info('starting the server')
        print('running')
        server.start()
    except KeyboardInterrupt:
        logging.info('stopping async tasks')
        print('interrupted')
        node_functions.cancel_node_async_tasks()
        led_strip.cancel_led_strip_async_tasks()
        motion_sensor.cancel_motion_sensor_async_tasks()
        logging.info('stopping the server')
        server.stop()
        logging.info('done \n')


if __name__ == '__main__':
    BASE_DIR = os.getcwd()
    FILE = os.path.join(BASE_DIR,"pwm-thing.log")
    logging.basicConfig(
        filename=FILE,
        level=10,
        format='%(asctime)s '
        '%(filename)s:'
        '%(lineno)s '
        '%(processName)s '
        '%(process)d, '
        '%(threadName)s '
        '%(thread)d, '
        '%(module)s, '
        '%(funcName)s, '
        '[%(levelname)s] '
        '%(message)s'
    )
    run_server()
