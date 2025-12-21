import esp32
import machine
from machine import Pin, SoftI2C, ADC
from machine import deepsleep, wake_reason, reset_cause

import ujson
import logging

from time import sleep
#import json

import mpyaes
import ubinascii
#import uos
#from ucryptolib import aes

import dht

# store in flash
rtc = machine.RTC()

buttonPIN = Pin(4, mode = Pin.IN)
mosfetPIN = Pin(14, mode = Pin.OUT, value=0)
ledPIN = Pin(25, mode = Pin.OUT, value=0)




log = logging.getLogger(__name__)
debug = Pin(39, mode = Pin.IN)
log.setLevel(logging.INFO)
if debug.value(): 
    log.setLevel(logging.DEBUG)




# VBAT Analog Readings
pot = ADC(Pin(35))
pot.atten(ADC.ATTN_11DB) 
"""The atten() method can take the following arguments:
    ADC.ATTN_0DB — the full range voltage: 1.2V
    ADC.ATTN_2_5DB — the full range voltage: 1.5V
    ADC.ATTN_6DB — the full range voltage: 2.0V
    ADC.ATTN_11DB — the full range voltage: 3.3V
"""

# Sleep Pins
""" Matrix
   off  off  = no sleep
   on   off  = 1min
   off  on   = 5min
   on   of   = 15min
"""
sleep1PIN = Pin(13, mode = Pin.IN)
sleep2PIN = Pin(36, mode = Pin.IN)

# DHT22
dhtPIN=Pin(15, mode= Pin.IN)
d = dht.DHT22(dhtPIN)



def measure():
    """DHT22 Sensor"""
    try:
        d.measure()
        temp = d.temperature()
        hum = d.humidity()
        log.info("Temperature %s - Humidity %s", temp, hum)
        return (temp, hum, "true")
    except OSError as e:
        log.error('Failed to read DHT22 sensor.')
        return (0, 0, "false")


def vbat():
    """Internal Power"""
    vbat_value = pot.read()
    vbat_volts = vbat_value / 4095*2*3.3*1.1
    log.info("VBAT raw %s - volts %s", vbat_value, vbat_volts)
    return (float)(vbat_volts)


def sensor():
    """Sensor Digital high/low"""
    sensor_value = buttonPIN.value()
    sensor_state = "false"
    if sensor_value == 1:
        sensor_state = "true"
    log.info("Sensor value %s - state %s", sensor_value, sensor_state)
    return (sensor_value, sensor_state)


def sleepMode():
    """"""
    sleep1_value = sleep1PIN.value()
    sleep2_value = sleep2PIN.value()
    if sleep1_value == 1 and sleep2_value == 0:
        log.debug("Sleep Mode 60 secs")
        return 60
    elif sleep1_value == 0 and sleep2_value == 1:
        log.debug("Sleep Mode 300 secs")
        return 300
    elif sleep1_value == 1 and sleep2_value == 1:
        log.debug("Sleep Mode 900 secs")
        return 900
    else:
        log.debug("No Sleep")
        return 0





def encrypt(plain):
    """"""
    key = '1234567890123456'
    iv = '1234567890123456'

    log.debug("Using AES%s-CBC cipher", (len(key * 8)))

    cipher = mpyaes.new(key,mpyaes.MODE_CBC,iv)
    ct_bytes = cipher.encrypt(plain)

    #print(type(ct_bytes))
    log.debug("AES-CBC encrypted: %s", ct_bytes)
    log.debug("AES-CBC encrypted HEX: %s", ct_bytes.hex())

    # Encode data in base64 format
    return ubinascii.b2a_base64(ct_bytes, newline=False)



def process(display, lora):
    """"""
    counter = 0
    log.info("This is LoRa sender")

    # non volatil, restore from RTC RAM, as storage for message counter
    log.debug("Read from RTC memory")
    rtc = machine.RTC()
    try:
        data = ujson.loads(rtc.memory()) 
        log.debug("  {}".format(data))
        counter = data['1']
    except:
        log.warning("RTC seems to be empty")
        counter = 0
        pass
    
    counter += 1
    
    d = {1:counter, 2:counter}
    
    # Save in RTC RAM
    rtc.memory(ujson.dumps(d))  


    #while True:

    if 1 == 1:

        ledPIN(1)

        temperature_value, humidity_value, state = measure()

        sensor_value, sensor_state = sensor()

        dictionary = { 
                "node": "0xA9",
                "msg_num": counter,
                "temperature": temperature_value,
                "humidity": humidity_value,
                "sensor_value": sensor_value,
                "sensor_state": sensor_state,
                "vbattery": vbat(),
                "wakeup_reason": wake_reason()
            }

        # for testing     
        xdictionary = {
                      "vbattery": 4.396777, 
                      "temperature": 23.6, 
                      "wakeup_reason": 4, 
                      "sensor_state": "false", 
                      "humidity": 32.5, 
                      "node": "0xA9", 
                      "msg_num": 1, 
                      "sensor_value": 0}
 
        display.text('Sensor:  {}'.format(sensor_value), 0, 20) # Text, X, Y
        display.text('Tempera: {}'.format(temperature_value), 0, 30) # Text, X, Y
        display.text('Luftfkt: {}'.format(humidity_value), 0, 40) # Text, X, Y
        display.show()

        # Convert dictionary to JSON string
        json_string = ujson.dumps(dictionary)
        
        # send as plain
        #log.debug("send as plain")

        # encrypt / encode base64
        enc = encrypt(json_string)
        log.debug('BASE64: %s', enc)
        enc = enc.strip()
        log.debug("send as encrypted---")
        lora.println(enc)

        ledPIN(0)

        ### END
 

def gotoSleep(display):
    """"""
    # get sleep mode
    sleep_ms = sleepMode() * 1000

    log.info("send, sleep for {}sec, see you later".format(int(sleep_ms/1000)))
        
    display.fill(0)
    display.show()
    display.text('send...', 0, 0) # Text, X, Y
    display.text('sleep for {}s'.format(int(sleep_ms/1000)), 0, 20)
    display.text('  see you later', 0, 40) # Text, X, Y

    display.show()

    sleep(1)
    
    if sleep_ms > 0:
        display.poweroff()
        deepsleep(sleep_ms)
    else:
        sleep(10)




def new(display,lora):

    """
    debug = Pin(39, mode = Pin.IN)
    logging.basicConfig(level=logging.INFO)
    if debug.value(): 
        logging.basicConfig(level=logging.DEBUG)
    """

    #log.info("info - test")
    #log.debug("debug - test")

    display.text('Lora Sender', 0, 0) # Text, X, Y
    display.show()
    
    ledPIN.off()
    mosfetPIN.on()

    sleep(2)

    process(display,lora)

    sleep(2)

    ledPIN.off()
    mosfetPIN.off()

    gotoSleep(display)

