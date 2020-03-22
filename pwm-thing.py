from __future__ import division, print_function
from webthing import (Action, Event, Property, MultipleThings, Thing, Value,
                      WebThingServer)
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
print("period max: "+str(pwm0.max_period())+" min: "+str(pwm0.min_period()))
pwm0.enable(True)               # Start sending PWM signal

gpio13 = mraa.Gpio(18)          # gpio13 PIN P18 on Linkit Smart 7688
gpio12 = mraa.Gpio(19)          # gpio12 PIN P19 on Linkit Smart 7688
gpio13.dir(mraa.DIR_OUT)        # set as OUTPUT pin
gpio12.dir(mraa.DIR_IN)         # set as INPUT pin
manual_switch_state_previous = gpio12.read()

class FadeAction(Action):

    def __init__(self, thing, input_):
        Action.__init__(self, uuid.uuid4().hex, thing, 'fade', input_=input_)

    def perform_action(self):
        brigthness_after = self.input['brightness']
        brightness_now = self.thing.get_property('brightness')
        if  brigthness_after > brightness_now :
            percentage = brigthness_after - brightness_now
            delay_ms = self.input['duration'] / percentage
            logging.info('start incrementing, brightness now %s, brightness afer %s, percentage %s, delay_ms %s, duration %s',
                         brightness_now,
                         brigthness_after,
                         percentage,
                         delay_ms,
                         self.input['duration'])
            while brigthness_after > self.thing.get_property('brightness'):
                self.thing.set_property('brightness', 
                                        self.thing.get_property('brightness')+1)
                time.sleep(delay_ms / 1000)
            logging.info('finished incrementing')
            return
        if brigthness_after < brightness_now:
            percentage = brightness_now - brigthness_after
            delay_ms = self.input['duration'] / percentage
            logging.info('start decrementing, brightness now %s, brightness afer %s, percentage %s, delay_ms %s, duration %s',
                        brightness_now,
                        brigthness_after,
                        percentage,delay_ms,
                        self.input['duration'])
            while brigthness_after < self.thing.get_property('brightness'):
                self.thing.set_property('brightness', 
                                        self.thing.get_property('brightness')-1)
                time.sleep(delay_ms / 1000)
            logging.info('finished decrementing')
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

        self.add_property(
            Property(self,
                     'state',
                     Value(self.get_state(),self.set_state),
                     metadata={
                         '@type': 'OnOffProperty',
                         'title': 'On/Off',
                         'type': 'boolean',
                         'description': 'Whether the Strip is turned on',
                     }
            )
        )
        self.timer = tornado.ioloop.PeriodicCallback(self.get_state,1000)
        self.timer.start()

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
        FadeAction)

    def get_brightness(self):
        """get the level from mraa.pwm"""
        brightness = round(pwm0.read() * 100,2)
        return brightness

    def set_brightness(self, brightness):
        """set the level with mraa"""
        level = round((brightness/100),2)
        pwm0.write(level)
        return brightness

    def get_state(self):
        """
        The manual switch is connected to a multiway switch made of two Relays 
        with 3-ways. 
        https://en.wikipedia.org/wiki/Multiway_switching#Traveler_system
        """
        state = bool(gpio13.read())
        global manual_switch_state_previous
        manual_switch_state = gpio12.read()
        if manual_switch_state_previous != manual_switch_state:
            if state == True:
                state = False
                pass     
            elif state == False:
                state = True
                pass
        manual_switch_state_previous = manual_switch_state
        self.set_property('state',state)

    def set_state(self, state):
        """set the state with mraa"""
        gpio13.write(state)
        return state

    def cancel_update_status_task(self):
        self.timer.stop()

def run_server():
    vorraum_led = LedStrip()

    # If adding more than one thing, use MultipleThings() with a name.
    # In the single thing case, the thing's name will be broadcast.
    server = WebThingServer(MultipleThings([vorraum_led],'LightAndTempDevice'), port=8888)
    try:
        logging.info('starting the server')
        server.start()
    except KeyboardInterrupt:
        logging.info('stopping the status update loop task')
        vorraum_led.cancel_update_status_task()
        logging.info('stopping the server')
        server.stop()
        logging.info('done')


if __name__ == '__main__':
    logging.basicConfig(
        level=10,
        format="%(asctime)s %(filename)s:%(lineno)s %(levelname)s %(message)s"
    )
    run_server()
