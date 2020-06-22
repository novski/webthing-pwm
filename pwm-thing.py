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
from astral import LocationInfo
from astral.sun import sun
from tornado import ioloop
city = LocationInfo("Zürich", "Switzerland", "Europe/Zurich", 47.4, 8.5)
currDate = datetime.datetime.now()
s = sun(city.observer, date=datetime.date(2020, 6, 6))
print((
    f'now:     {currDate}\n'
    f'Dawn:    {s["dawn"]}\n'
    f'Sunrise: {s["sunrise"]}\n'
    f'Noon:    {s["noon"]}\n'
    f'Sunset:  {s["sunset"]}\n'
    f'Dusk:    {s["dusk"]}\n'
))
location = 'og-vorraum'
w1_device_id_list = []

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
    except:
        logging.warn('get_w1_devices: no devices found')

def get_main_loop():
    return ioloop.IOLoop.current()

class FadeLedStrip(Action):

    def __init__(self, thing, input_):
        Action.__init__(self, uuid.uuid4().hex, thing, 'fade', input_=input_)
        self.thing.get_property('on')

    def perform_action(self):
        brightness_after = self.input['brightness']
        brightness_now = self.thing.get_property('brightness')
        if  brightness_after > brightness_now :
            percentage = brightness_after - brightness_now
            delay_ms = self.input['duration'] / percentage
            logging.info('FadeLedStrip.perform_action: start incrementing, brightness_now from %s to brightness_afer %s where percentage range is %s and per step of delay_ms: %s while total duration is %s ms',
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

    def __init__(self, location, _main_loop):
        self.main_loop = _main_loop
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
        
        self.main_loop_time = None
        self.timeout = None

        self.state = Value(self.get_state(),self.toggle_digitalswitch)
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
                     Value(self.get_brightness(),self.set_brightness),
                     metadata={
                         '@type': 'BrightnessProperty',
                         'title': f'{self.name}-brightness',
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
            'title': f'{self.name}-fade_action',
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

        self.add_property(
            Property(self,
                'motion_sensor_delay',
                Value(5),
                metadata={
                    '@type': 'LevelProperty',
                    'title': f'Motion_Sensor_Delay',
                    'type': 'float',
                    'description': 'The delay in minutes',
                    'minimum': 0,
                    'maximum': 60,
                    'unit': 'minutes',
                }
            )
        )

        self.motion_on_off = Value(True)
        self.add_property(
            Property(self,
                     'motion_on_off',
                     self.motion_on_off,
                     metadata={
                         '@type': 'OnOffProperty',
                         'title': f'{self.name}-motion_on_off',
                         'type': 'boolean',
                         'description': 'Whether the motion sensor is active',
                     }
            )
        )

        self.timer = ioloop.PeriodicCallback(
                self.get_r1,
                1000
            )
        self.timer.start()
    
    """ PWM """
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
        """ only if footswitch changes """
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
        footswitch = self.relay1_read()                 # relay 1
        digitalswitch = bool(self.relay2.read())        # relay 2
        return footswitch ^ digitalswitch

    def toggle_digitalswitch(self,state):
        """
        gateway or other GUI representaton of switch to turn on/off LedStrip.
        """
        self.relay2.write(not self.relay2.read())

    """ MOTION SENSOR 
        Interrupt calls this when ever it triggers """

    def motion(self):
        if self.motion_on_off:
            self.main_loop.add_callback(self.interrupt_call_back)

    def interrupt_call_back(self):
        print("21")
        if self.timeout:
            self.main_loop.remove_timeout(self.timeout)
        self.main_loop_time = self.main_loop.time()
        self.get_delay_time()
        sw = self.get_property('digitalswitch')
        if not sw:
            self.set_property('digitalswitch', True)
        timeout = self.main_loop_time + self.delay_seconds
        self.timeout = self.main_loop.add_timeout(timeout, self.interrupt_call_timeout)

    def interrupt_call_timeout(self):
        print("22")
        sw = self.get_property('digitalswitch')
        foot_sw = self.relay1_read()
        if (sw) and (not foot_sw):
            self.set_property('digitalswitch', False)
            sw = self.get_property('digitalswitch')
        self.timeout = None

    def get_delay_time(self):
        self.delay_minutes = self.get_property('motion_sensor_delay')
        self.delay_seconds = self.delay_minutes * 60

    """ END OF LED STRIP THING """
    def cancel_led_strip_async_tasks(self):
        logging.info('stopping the status update loop task')
        self.timer.stop()


class MotionSensor(Thing):
    """A PIR Sensor in the entry of my upper Level"""

    def __init__(self,location, _main_loop, _led_strip):
        self.main_loop = _main_loop
        self.led_strip = _led_strip
        self.id = f'urn:dev:ops:{location}-motion-sensor'
        self.name = f'{location}-motion-sensor'
        Thing.__init__(
            self,
            self.id,
            self.name,
            ['MotionSensor'],
            'A web connected motion sensor'
        )
        """ MOTION SENSOR PIN DEFINITION """
        self.motion_sensor = mraa.Gpio(13)      # GPIO 1 (P13) on Linkit Smart 7688
        self.motion_sensor.dir(mraa.DIR_IN)     # set as INPUT pin
        self.motion_sensor.isr(mraa.EDGE_BOTH, MotionSensor.interrupt_call, self)
        """
        figured out that mt7688an has onls 8 interrupts and GPIO13 and 18 get
        triggered same time because of a electic pinmux that is not handled
        in kernel. There might be more pin's triggering same time.
        Can be avoided by checking the pin that is interrupted with the pin
        that is whanted to be HIGH. In this case i check if pin13 is High
        whenever a interrupt acures.
        """

        self.interrupt_blocker = None


        self.motion = Value(self.get_motion(),self.set_motion)
        self.add_property(
            Property(self,
                'motion_detection',
                self.motion,
                metadata={
                    '@type': 'MotionProperty',
                    'title': f'{self.name}-Motion_Status',
                    'type': 'boolean',
                    'description': 'motion=true, nomotion=false',
                }
            )
        )

    def get_motion(self):
        logging.debug("get_motion")
    
    def set_motion(self, value):
        logging.debug(f"set_motion{value}")

    def interrupt_call(self):
        if self.motion_sensor.read():
            logging.debug("motion_sensor detected motion")
            print("1")
            if not self.interrupt_blocker:
                self.interrupt_blocker = True
                self.set_property('motion_detection', True)
                self.led_strip.motion()
                self.set_property('motion_detection', False)
                print("21")
                pass
            print("11")
            self.main_loop.add_callback(self.interrupt_block_call_back)

            
    def interrupt_block_call_back(self):
        print("12")
        self.blocker_seconds = 2
        self.main_loop.call_later(self.blocker_seconds,
                                  self.interrupt_block_call_deblock)

    def interrupt_block_call_deblock(self):
        self.interrupt_blocker = False
        print("13")

    """ END OF MOTION SENSOE THING """
    def cancel_motion_sensor_async_tasks(self):
        logging.info('stopping the interrupt')
        self.motion_sensor.isrExit()   

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
        self.timer = ioloop.PeriodicCallback(
                self.get_sensors,
                300000
            )
        self.timer.start()

    def get_sensors(self):
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
            logging.warn("get_temperature_read_one: not able to read from: %s",temp_sensor_id)

    def cancel_node_async_tasks(self):
        logging.info('stopping the node update loop task')
        self.timer.stop()

def run_server():
    get_w1_devices()
    main_loop = get_main_loop()
    node_functions = VorraumNode(location)
    node_functions.get_sensors()
    led_strip = LedStrip(location, main_loop)
    motion_sensor = MotionSensor(location,main_loop, led_strip)

    server = WebThingServer(MultipleThings([node_functions,
                                            led_strip,
                                            motion_sensor],
                                           'LightAndTempDevice'), port=8888)
    try:
        logging.info('starting the server')
        server.start()
    except KeyboardInterrupt:
        logging.info('stopping async tasks')
        led_strip.cancel_led_strip_async_tasks()
        node_functions.cancel_node_async_tasks()
        motion_sensor.cancel_motion_sensor_async_tasks()
        logging.info('stopping the server')
        server.stop()
        logging.info('done')


if __name__ == '__main__':
    logging.basicConfig(
        level=10,
        format="%(asctime)s %(filename)s:%(lineno)s %(levelname)s %(message)s"
    )
    run_server()

