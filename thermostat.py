# pip3 install paho-mqtt

import RPi.GPIO as GPIO
import time
import os
import time
from time import sleep, strftime
from lib_sh1106 import sh1106
from PIL import ImageFont, ImageDraw, Image
from smbus import SMBus
import datetime
import paho.mqtt.client as mqtt
import logging

#######################################################

#room_temp_topic = "sensor_2/loft/temperature"
living_room_temp_topic = "sensor/433_gateway/temperature"

outdoor_temp_topic = "sensor/temperature_pm_sensor_0/Temperature"
supply_water_temp_topic = "central_heating/status/supply_water_temp"
mode_control_topic = "central_heating/control/mode"
destination_temp_topic = "central_heating/control/destination_temp"
relay_control_topic = "central_heating/control/relay"

#######################################################

i2cbus = SMBus(1)
oled = sh1106(i2cbus)
draw = oled.canvas

default_font = ImageFont.load_default()
font = ImageFont.truetype('OpenSans-Light.ttf', 12)

def draw_text(text, x, y):
    try:
        (font_width, font_height) = font.getsize(text)
        draw.rectangle((x, y, font_width, font_height + y), outline=0, fill=0)
        draw.text((x, y), text, font=font, fill=1)
        #oled.display()
        #oled.cls()
        #draw.rectangle((0, 5, font_width, font_height + 5), outline=1, fill=0)
        oled.display()
    except Exception as e:
        self.logger.error(e)

#####################################################

class Logger:
    def __init__(self):
        logging.basicConfig(filename = "thermostat.log",
                            filemode = 'a',
                            format = '%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                            datefmt = '%H:%M:%S',
                            level = logging.DEBUG)
        self.logger = logging.getLogger('thermostat')

#####################################################

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(9, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class StepperMotor(Logger):

    def __init__(self, A1, A2, B1, B2):
        Logger.__init__(self)
        self.PIN_A1 = A1
        self.PIN_A2 = A2
        self.PIN_B1 = B1
        self.PIN_B2 = B2
        GPIO.setup(A1, GPIO.OUT)
        GPIO.setup(A2, GPIO.OUT)
        GPIO.setup(B1, GPIO.OUT)
        GPIO.setup(B2, GPIO.OUT)
        self.calibrated_low_counter = None
        self.calibrated_high = None
        self.min_temperature = 27
        self.max_temperature = 74
        self.one_degree_counter = None
        self.speed = 5 / 1000
        self.count = 0
        self.motor_forward = [
            (1, 0, 0, 0),
            (1, 1, 0, 0),
            (0, 1, 0, 0),
            (0, 1, 1, 0),
            (0, 0, 1, 0),
            (0, 0, 1, 1),
            (0, 0, 0, 1),
            (1, 0, 0, 1)
        ]
        self.motor_backward = [
            (0, 0, 0, 1),
            (0, 0, 1, 1),
            (0, 0, 1, 0),
            (0, 1, 1, 0),
            (0, 1, 0, 0),
            (1, 1, 0, 0),
            (1, 0, 0, 0),
            (1, 0, 0, 1)
        ]
        self.motor_neutral = (0, 0, 0, 0)
        self.status = None
        self.is_calibrated = False

    def backward(self, delay, steps):
        #GPIO.output(17, GPIO.HIGH)
        self.counter = 0
        for i in range(0, steps):
            if GPIO.input(9) == True:
                break
            for entry in self.motor_forward:
                self.setStep(entry[0], entry[1], entry[2], entry[3])
                time.sleep(delay)
            self.counter += 1
            self.count += 1
        #GPIO.output(17, GPIO.LOW)
        return self.counter

    def forward(self, delay, steps):
        self.counter = 0
        for i in range(0, steps):
            if GPIO.input(10) == True:
                break
                
            for entry in self.motor_backward:
                self.setStep(entry[0], entry[1], entry[2], entry[3])
                time.sleep(delay)
                
            self.counter += 1
            self.count -= 1
        return self.counter

    def setStep(self, w1, w2, w3, w4):
        GPIO.output(self.PIN_A1, GPIO.HIGH if w1 == 1 else GPIO.LOW)
        GPIO.output(self.PIN_A2, GPIO.HIGH if w2 == 1 else GPIO.LOW)
        GPIO.output(self.PIN_B1, GPIO.HIGH if w3 == 1 else GPIO.LOW)
        GPIO.output(self.PIN_B2, GPIO.HIGH if w4 == 1 else GPIO.LOW)

    def calibrate(self):
        self.logger.debug("calibrating")
        self.is_calibrated = False
        draw_text("calibrating...", 0, 0)
        
        self.logger.debug("calibrating min temperature steps value")
        back_counter = self.forward(self.speed, 1024)
        #self.logger.debug(back_counter)
        #time.sleep(1)        
        self.logger.debug("calibrating max temperature steps value")
        forward_counter = self.backward(self.speed, 1024)
        #self.logger.debug(forward_counter)
        #time.sleep(1)
        
        max_counter = max(forward_counter, back_counter)
        #self.logger.debug("max_counter", max_counter)
        self.one_degree_counter = max_counter / (self.max_temperature - self.min_temperature)
        self.logger.debug("one degree %.2f steps" % self.one_degree_counter)
        
        self.forward(self.speed, max_counter)
        self.count = 0
        #self.logger.debug("min temp")
        self.setStep(0, 0, 0, 0)
        self.is_calibrated = True
        oled.display()
        oled.cls()
        self.logger.debug("thermostat has been calibrated")

    def set_water_supply_temperature(self, temp):
        if self.is_calibrated:
            draw_text("temperature: %i" % temp, 0, 0)
            x = int((temp - self.min_temperature) * self.one_degree_counter - self.count)
            if x > 0:
                self.backward(self.speed, abs(x))
            else:
                self.forward(self.speed, abs(x))
            self.setStep(0, 0, 0, 0)

class HeatingCurve(Logger):
    def __init__(self):
        Logger.__init__(self)
        self.minimum_temp = 25
        self.maximum_temp = 90
        self.slope = 1.4
        self.factor = 50
    
    def calculate_water_supply_temp(self, outdoor_temp):
        temp =  self.minimum_temp + (self.minimum_temp - outdoor_temp) * (self.maximum_temp - self.minimum_temp) / (self.minimum_temp + self.factor / self.slope)
        self.logger.debug("calculated water supply temp: %.2f for outdoor temperature: %.2f" % (temp, outdoor_temp))
        if temp > 60:
            temp = 60
        elif temp < 47:
            temp = 47
        return temp

class RelayControl(Logger):
    def __init__(self, relay_1_pin, relay_2_pin):
        Logger.__init__(self)
        self.relay_1_pin = relay_1_pin
        self.relay_2_pin = relay_2_pin
        self.init_gpio()
    
    def init_gpio(self):
        GPIO.setup(self.relay_1_pin, GPIO.OUT)
        GPIO.setup(self.relay_2_pin, GPIO.OUT)
        GPIO.output(self.relay_1_pin, GPIO.HIGH)
        GPIO.output(self.relay_2_pin, GPIO.HIGH)
    
    def relay_1_on(self):
        self.logger.debug("relay_1_on")
        GPIO.output(self.relay_1_pin, GPIO.LOW)

    def relay_1_off(self):
        self.logger.debug("relay_1_off")
        GPIO.output(self.relay_1_pin, GPIO.HIGH)

    def relay_2_on(self):
        GPIO.output(self.relay_2_pin, GPIO.LOW)

    def relay_2_off(self):
        GPIO.output(self.relay_2_pin, GPIO.HIGH)


class Thermostat(Logger):
    def __init__(self):
        Logger.__init__(self)
        self.motor1 = StepperMotor(6, 13, 19, 26)
        self.init_mqtt()
        self.relay = RelayControl(17, 27)
        self.mode = 'auto'
        self.heating_curve = HeatingCurve()
        self.base_water_supply_temperature = 50
        self.destination_temp_up_margin = 0.1
        self.destination_temp_down_margin = 0
        self.destination_temps = [
            # 0   1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23   
            20.3, 20.3, 20.3, 20.3, 20.3, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.7, 20.3
        ]
        self.outdoor_temperature_offset = -1.0
    
    def init_mqtt(self):
        self.mqtt_client = mqtt.Client("thermostat")
        self.mqtt_client.connect("mqtt_server", 1883)
        self.mqtt_client.subscribe(living_room_temp_topic)
        self.mqtt_client.subscribe(outdoor_temp_topic)
        self.mqtt_client.subscribe(supply_water_temp_topic)
        self.mqtt_client.subscribe(mode_control_topic)
        self.mqtt_client.subscribe(destination_temp_topic)
        self.mqtt_client.subscribe(relay_control_topic)
        self.mqtt_client.on_message = self.mqtt_callback
        self.mqtt_client.loop_start()

    def mqtt_callback (self, client, userdata, message):
        topic = str(message.topic)
        message = str(message.payload.decode("utf-8"))
        self.logger.debug(topic + ": " + message)
        if topic == mode_control_topic:
            if message == 'calibrate':
                self.calibrate()
                self.set_water_supply_temperature(self.base_water_supply_temperature)
            if message == 'auto':
                self.set_mode('auto')
            if message == 'manual':
                self.set_mode('manual')
            if message == 'test':
                self.test()
        elif topic == relay_control_topic:
            if message == 'relay_1_on':
                self.mqtt_client.publish("central_heating/status/heating", payload="1", qos=0, retain=False)
                self.relay.relay_1_on()
            if message == 'relay_1_off':
                self.mqtt_client.publish("central_heating/status/heating", payload="0", qos=0, retain=False)
                self.relay.relay_1_off()
            if message == 'relay_2_on':
                self.relay.relay_2_on()
            if message == 'relay_2_off':
                self.relay.relay_2_off()
        #elif topic == supply_water_temp_topic:
        #    self.set_water_supply_temperature(float(message))
        #elif topic == destination_temp_topic:
            #self.set_destination_temp(float(message))
        elif topic == outdoor_temp_topic:
            draw.text((0, 10), "outdoor temp: %s" % message, font=font, fill=1)
            outdoor_temp = float(message) + self.outdoor_temperature_offset
            water_supply_temp = self.heating_curve.calculate_water_supply_temp(outdoor_temp)
            self.set_water_supply_temperature(water_supply_temp)
        #elif topic == room_temp_topic or topic == living_room_temp_topic:
        elif topic == living_room_temp_topic:
            draw.text((0, 20), "room temp: %s" % message, font=font, fill=1)
            temp = float(message)
            current_hour = datetime.datetime.now().hour
            target_temperature_for_current_hour = self.destination_temps[current_hour]
            self.logger.debug("target temperature for current hour: %.1f" % target_temperature_for_current_hour)
            
            if temp > target_temperature_for_current_hour + self.destination_temp_up_margin:
                self.relay.relay_1_off()
                self.mqtt_client.publish("central_heating/status/heating", payload="0", qos=0, retain=False)
            elif temp < target_temperature_for_current_hour - self.destination_temp_down_margin:
                self.relay.relay_1_on()
                self.mqtt_client.publish("central_heating/status/heating", payload="1", qos=0, retain=False)
        
        #elif topic == living_room_temp_topic:
            #draw.text((0, 20), "living room temp: %s" % message, font=font, fill=1)
            #temp = float(message)
            #current_hour = datetime.datetime.now().hour
            #target_temperature_for_current_hour = self.destination_temps[current_hour] - 0.2
            #self.logger.debug("living room's target temperature for current hour: %.1f" % target_temperature_for_current_hour)
            #if temp > target_temperature_for_current_hour + self.destination_temp_up_margin:
                #self.relay.relay_1_off()
                #self.mqtt_client.publish("central_heating/status/heating", payload="0", qos=0, retain=False)
            #elif temp < target_temperature_for_current_hour - self.destination_temp_down_margin:
                #self.relay.relay_1_on()
                #self.mqtt_client.publish("central_heating/status/heating", payload="1", qos=0, retain=False)
        
        oled.display()
        
    #def set_destination_temp(self, temp):
        #self.destination_temp = temp
        #draw.text((0, 30), "destination temp: %.2f" % self.destination_temp, font=font, fill=1)
        #oled.display()
    
    def calibrate(self):
        self.mqtt_client.publish("central_heating/status/status", payload="calibrating", qos=0, retain=False)
        self.motor1.calibrate()
        self.mqtt_client.publish("central_heating/status/status", payload="calibrated", qos=0, retain=False)
        #draw.text((0, 30), "destination temp: %.2f" % self.destination_temp, font=font, fill=1)
        oled.display()
    
    def set_mode(mode):
        self.mode = mode
    
    def set_water_supply_temperature(self, temp):
        self.logger.debug("setting water supply temperature: %i" % temp)
        self.mqtt_client.publish(supply_water_temp_topic, payload=round(temp, 2), qos=0, retain=False)
        self.motor1.set_water_supply_temperature(temp)
        
    def test(self):
        for temp in (50, 70, 30, 58, 47, 50, 51, 52, 53, 54, 50):
            self.logger.debug("test temp: %i" % temp)
            self.motor1.set_water_supply_temperature(temp)
            time.sleep(3)


#######################################################

if __name__=='__main__':
    thermostat = Thermostat()
    thermostat.calibrate()
    thermostat.relay.relay_1_on()
    #thermostat.test()
    
    while True:
        time.sleep(1)
