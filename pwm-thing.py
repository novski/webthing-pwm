from __future__ import division, print_function

import logging
import os
import random
import sys
import time
import uuid
from datetime import datetime, timedelta

import mraa
import tornado.ioloop
from astral import LocationInfo, sun
from webthing import (Action, Event, MultipleThings, Property, Thing, Value,
                      WebThingServer)

import ina219


class FadeLedStrip(Action):

    def __init__(self, thing, input_):
        Action.__init__(self, uuid.uuid4().hex, thing, 'fade', input_=input_)

    def perform_action(self):
        brightness_after = self.input['brightness']
        duration = self.input['duration']
        brightness_now = self.thing.get_property('brightness')
        if brightness_after > brightness_now:
            steps = brightness_after - brightness_now
            delay_ms = duration / steps
            while brightness_after > self.thing.get_property('brightness'):
                self.thing.set_property('brightness',
                                        self.thing.get_property('brightness')+1)
                time.sleep(delay_ms / 1000)
        elif brightness_after < brightness_now:
            steps = brightness_now - brightness_after
            delay_ms = duration / steps
            while brightness_after < self.thing.get_property('brightness'):
                self.thing.set_property('brightness',
                                        self.thing.get_property('brightness')-1)
                time.sleep(delay_ms / 1000)


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

        self.locality = LocationInfo(
            'Zürich', 'Switzerland', 'Europe/Zurich', 47.39, 8.07)
        logging.info(
            f'Information for {self.locality.name}/{self.locality.region}, '
            f'Timezone: {self.locality.timezone}, '
            f'Latitude: {self.locality.latitude:.02f}; '
            f'Longitude: {self.locality.longitude:.02f}'
        )
        self.day_time_start = sun.sunrise(
            self.locality.observer,
            date=datetime.now(),
            tzinfo=self.locality.timezone) - timedelta(minutes=30)
        logging.debug(f'LedStrip: day_time_start:{self.day_time_start}')
        self.day_time_stop = sun.sunset(
            self.locality.observer,
            date=datetime.now(),
            tzinfo=self.locality.timezone) + timedelta(minutes=30)
        logging.debug(f'LedStrip: day_time_stop:{self.day_time_stop}')

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

        self.brightness = Value(self.get_brightness(), self.set_brightness)
        self.add_property(
            Property(self,
                     'brightness',
                     self.brightness,
                     metadata={
                         '@type': 'LevelProperty',
                         'title': 'Helligkeit',
                         'type': 'integer',
                         'description': 'The level of light from 0-100%',
                         'minimum': 0,
                         'maximum': 100,
                         'unit': 'percent',
                     }
                     )
        )

        self.add_available_action('fade', {
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

        self.motion_detection_delay = Value(
            self.get_motion_detection_delay(), self.set_motion_detection_delay)
        self.add_property(
            Property(self,
                     'motion_detection_delay',
                     self.motion_detection_delay,
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

        self.motion_detection_active = Value(
            self.get_motion_detection_active(), self.set_motion_detection_active)
        self.add_property(
            Property(self,
                     'motion_detection_active',
                     self.motion_detection_active,
                     metadata={
                         '@type': 'BooleanProperty',
                         'title': 'Auf Bewegung reagieren?',
                         'type': 'boolean',
                         'description': 'Set Motion detection active or not',
                     }
                     )
        )

        self.motion_detection_follower = Value(
            self.get_motion(), self.set_motion)
        self.add_property(
            Property(self,
                     'motion_detection_follower',
                     self.motion_detection_follower,
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
        brightness = round(self.pwm.read() * 100)
        logging.debug(f'LedStrip: get_brightness is set to: {brightness}')
        return brightness

    def set_brightness(self, brightness):
        """set the level with mraa"""
        self.pwm.write(round((brightness/100), 2))
        led_on = self.get_property('digitalswitch')
        if brightness < 1 and led_on is True:
            self.set_property('digitalswitch', False)
        elif brightness >= 1 and led_on is False:
            self.set_property('digitalswitch', True)
        logging.info(f'LedStrip: Set Brightness to {brightness}')
        return brightness

    def relay1_read(self):
        relay1 = bool(self.relay1.read())
        if relay1 is True:
            relay1 = False
        elif relay1 is False:
            relay1 = True
        return relay1

    def get_r1(self):
        """ only if footswitch changes """
        r1 = self.relay1_read()
        if r1 != self.r1_previous:
            state = self.get_state()
            # if state:
            #     self.fade_brightness_up(self.get_property('brightness'), 4000, 80)
            # else:
            #     self.fade_brightness_down(self.get_property('brightness'), 4000, 0)
            self.state.notify_of_external_update(state)
        self.r1_previous = r1

    def get_state(self):
        """
        The manual switch is connected to a multiway switch made of two Relays
        with 3-ways.
        https://en.wikipedia.org/wiki/Multiway_switching#Traveler_system
        """
        footswitch = bool(self.relay1_read())
        digitalswitch = bool(self.relay2.read())
        state = footswitch ^ digitalswitch
        logging.info(f'LedStrip: get_state: is set to {state}')
        return state

    def toggle_digitalswitch(self, state):
        """
        switch to turn on/off LedStrip.
        """
        s = self.relay2.read()
        self.relay2.write(not s)
        logging.info('LedStrip: toggle_digitalswitch called '
                     f'with state:{state} set to {s}')

    def motion(self):
        """ MOTION SENSOR calls this when ever it triggers """
        logging.debug('LedStrip: motion: starts')
        date_time_now = datetime.now().time()
        logging.debug('LedStrip: motion: called self.date_time_now:'
                      f'{date_time_now}')
        day_start = self.day_time_start.time()
        logging.debug(f'LedStrip: motion: called day_start:{day_start}')
        day_end = self.day_time_stop.time()
        logging.debug(f'LedStrip: motion: called day_end:{day_end}')
        if date_time_now > day_start and date_time_now < day_end:
            self.day_time = True
            logging.debug('LedStrip: motion: called during the Day')
        else:
            self.day_time = False
            logging.debug('LedStrip: motion: called in the Night')
        if not self.day_time:
            motion_active = self.get_property('motion_detection_active')
            logging.debug('LedStrip: motion: called while motion_active:'
                          f'{motion_active}')
            if motion_active:
                logging.debug('LedStrip: motion: called on not day_time '
                              'and Motion was activated, future add_callback '
                              'awaits interrupt_call_back now')
                self.main_loop.add_callback(self.interrupt_call_back)
            else:
                logging.debug('LedStrip: motion: Motion is not activated')
        logging.debug('LedStrip: motion: end')

    def interrupt_call_back(self):
        logging.info('LedStrip: interrupt_call_back: starts, '
                     f'async_timeout:{self.async_timeout}')
        if self.async_timeout:
            self.main_loop.remove_timeout(self.async_timeout)
            logging.debug('LedStrip: interrupt_call_back: timeout was removed,'
                          f' async_timeout:{self.async_timeout}')
        digitalswitch = self.get_property('digitalswitch')
        if digitalswitch:
            logging.debug(f'LedStrip: interrupt_call_back: '
                          f'digitalswitch is {digitalswitch} '
                          'light is already ON')
        else:
            logging.debug(f'LedStrip: interrupt_call_back: '
                          f'digitalswitch is {digitalswitch} '
                          'light now turns ON')
            #self.set_brightness(self, 1) 
            self.set_property('digitalswitch', True)
        delay_seconds = self.get_delay_seconds()
        timeout_delta = self.main_loop.time() + delay_seconds
        timeout_delta_human = time.strftime('%H:%M:%S',
                                            time.gmtime(timeout_delta))
        logging.debug(f'LedStrip: interrupt_call_back:'
                      f'Light will stay on for {delay_seconds}s '
                      f'and turns OFF at:{timeout_delta_human}')
        self.async_timeout = self.main_loop.add_timeout(
            timeout_delta, self.interrupt_call_timeout)
        logging.debug('LedStrip: interrupt_call_back: end')

    def interrupt_call_timeout(self):
        logging.info('LedStrip: interrupt_call_timeout: called')
        digitalswitch = self.get_property('digitalswitch')
        footswitch = self.relay1_read()
        if (digitalswitch) and (not footswitch):
            self.set_property('digitalswitch', False)
        logging.debug('LedStrip: interrupt_call_timeout: '
                      f'async_timeout: {self.async_timeout}')
        self.main_loop.remove_timeout(self.async_timeout)
        logging.debug('LedStrip: interrupt_call_timeout '
                      f'async_timeout removed: {self.async_timeout}')
        self.async_timeout = False

    def fade_brightness_up(self, brightness_now, duration, brightness_after):
        steps = brightness_after - brightness_now
        if steps < 1:
            steps = 1
        if steps > 100:
            steps = 100
        delay_ms = duration / steps
        while brightness_after > self.get_property('brightness'):
            self.set_property('brightness', self.get_property('brightness')+1)
            time.sleep(delay_ms / 1000)

    def fade_brightness_down(self, brightness_now, duration, brightness_after):
        steps = brightness_now - brightness_after
        if steps < 1:
            steps = 1
        if steps > 100:
            steps = 100
        delay_ms = duration / steps
        while brightness_after < self.get_property('brightness'):
            self.set_property('brightness', self.get_property('brightness')-1)
            time.sleep(delay_ms / 1000)

    def get_delay_seconds(self):
        delay_minutes = self.get_property('motion_detection_delay')
        delay_seconds = delay_minutes * 60
        logging.debug(f'LedStrip: get_delay_seconds is {delay_seconds}')
        return delay_seconds

    def get_motion(self):
        logging.debug('LedStrip: get_motion: get_motion_follower')
        value = self.get_property('motion_detection_follower')
        if value is None:
            logging.debug(
                f'LedStrip: get_motion: '
                f'get_motion_detection_follower is {value}')
            self.set_property('motion_detection_follower', False)
        value = self.get_property('motion_detection_follower')
        logging.debug(
            f'LedStrip: get_motion: get_motion_detection_follower is {value}')
        return value

    def set_motion(self, value):
        if value:
            logging.debug('LedStrip: set_motion before set_motion_follower '
                          f'(LedStrip): {value}')
            self.motion()
            logging.debug('LedStrip: set_motion after set_motion_follower '
                          f'(LedStrip): {value}')
        else:
            logging.debug('LedStrip: set_motion is False we do nothing more '
                          f'(LedStrip) set_motion: {value}')

    def get_motion_detection_active(self):
        value = self.get_property('motion_detection_active')
        if value is None:
            logging.info(
                f'LedStrip: get_motion_detection_active: '
                f'setting: True because {value}')
            return True
        return value

    def set_motion_detection_active(self, value):
        logging.info(f'LedStrip: set_motion_detection_active to {value}')

    def get_motion_detection_delay(self):
        value = self.get_property('motion_detection_delay')
        if value is None:
            logging.info(
                f'get_motion_detection_delay setting to 2 because {value}')
            return 2
        return value

    def set_motion_detection_delay(self, value):
        logging.info(f'LedStrip: set_motion_detection_delay to {value}')

    """ END OF LED STRIP THING """

    def cancel_led_strip_async_tasks(self):
        logging.info('LedStrip: stopping the status update loop task')
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
        self.motion_sensor.dir(mraa.DIR_IN)  # set as INPUT pin
        # self.motion_sensor.isr(
        #     mraa.EDGE_RISING, MotionSensor.interrupt_call, self)
        # self.motion_sensor.isr(mraa.EDGE_FALLING, MotionSensor.timeout, self)
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
        self.timer = tornado.ioloop.PeriodicCallback(
            self.interrupt_call,
            500
        )
        self.timer.start()

    def get_motion(self):
        logging.debug('MotionSensor: get_motion: starts')
        if self.get_property('motion_detection') is None:
            logging.debug("MotionSensor: get_motion: is None")
            self.set_property('motion_detection', False)

    def set_motion(self, value):
        logging.debug(f'MotionSensor: set_motion: {value}')

    def interrupt_call(self):
        if (self.motion_sensor.read() and
                not self.get_property('motion_detection')):
            logging.debug('MotionSensor: interrupt_call: starts, INPUT is '
                          'triggerd and Motion is False now setting Motion '
                          'True')
            self.set_property('motion_detection', True)
            logging.debug('MotionSensor: interrupt_call: INPUT is triggerd '
                          'and Motion is False now setting Motion True in '
                          'LEDStrip')
            self.led_strip.set_property('motion_detection_follower', True)
            logging.debug(
                'MotionSensor: interrupt_call: LED Strip set Motion done.')
            self.main_loop_time = self.main_loop.time()
            self.main_loop_time_human = time.strftime('%H:%M:%S',
                                                      time.gmtime(
                                                          self.main_loop_time))
            logging.debug(f'MotionSensor: interrupt_call: main_loop_time at '
                          f'{self.main_loop_time} or '
                          f'{self.main_loop_time_human}')
            self.delay_in_sec = 4
            logging.debug(f'MotionSensor: interrupt_call: delay_in_sec: '
                          f'{self.delay_in_sec}')
            self.time_when_off = self.main_loop_time + self.delay_in_sec
            logging.debug(f'MotionSensor: interrupt_call: time_when_off at '
                          f'{self.time_when_off}')
            self.t = time.strftime(
                '%H:%M:%S', time.gmtime(self.time_when_off))
            logging.debug(f'MotionSensor: interrupt_call: timeout at {self.t}')
            self.main_loop.add_timeout(self.time_when_off, self.timeout)
            logging.debug('MotionSensor: interrupt_call: end')

    def timeout(self):
        logging.debug(
            'MotionSensor: timeout: setting motion False')
        self.set_property('motion_detection', False)
        logging.debug(
            'MotionSensor: timeout: setting motion False on LED Strip')
        self.led_strip.set_property('motion_detection_follower', False)
        logging.debug('MotionSensor: timeout: end')

    """ END OF MOTION SENSOR THING """

    def cancel_motion_sensor_async_tasks(self):
        logging.info('MotionSensor: stopping the interrupt')
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
            logging.info(f'get_temperature:reading w1 device {device}. '
                         f'{number_of_device_ids} to been read')
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
        except Exception:
            logging.warn('get_temperature_read_one: '
                         f'not able to read from: {temp_sensor_id}')

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
    except Exception:
        logging.warn('get_w1_devices: no devices found')


def run_server():
    location = 'og-vorraum'
    w1_device_id_list = get_w1_devices()
    main_loop = tornado.ioloop.IOLoop.current()
    node_functions = VorraumNode(location, w1_device_id_list)
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
    FILE = os.path.join(BASE_DIR, "pwm-thing.log")
    logging.basicConfig(
        # filename=FILE,
        level=10,
        format='%(asctime)s '
        '[%(levelname)s] '
        '%(processName)s:'
        '{%(process)d}, '
        '%(threadName)s:'
        '{%(thread)d}, '
        '%(module)s, '
        '%(filename)s:'
        '%(lineno)s '
        '%(message)s'
    )
    run_server()
