import esp32
import machine
from machine import Pin, SoftI2C, ADC
from machine import deepsleep, wake_reason, reset_cause

import ujson
from time import sleep

import myaes as myaes
import ubinascii

import dht

from lib import logging
import config.config as config
import config.secrets as secrets

# store in flash
rtc = machine.RTC()

log = logging.getLogger(__name__)


# PINs
sensorPIN = Pin(4, mode=Pin.IN)                   # sensor
greenLedPIN = Pin(25, mode=Pin.OUT, value=0)      # green onboard led, lora send indicator 
outDriverPIN = Pin(14, mode=Pin.OUT, value=0)

# Sleep Pins
sleep1PIN = Pin(2, mode=Pin.IN, pull=Pin.PULL_DOWN)
sleep2PIN = Pin(39, mode=Pin.IN)

# DHT22
dhtPIN=Pin(15, mode=Pin.IN)
d = dht.DHT22(dhtPIN)

# VBAT Analog Readings
pot = ADC(Pin(35))
pot.atten(ADC.ATTN_11DB) 
"""The atten() method can take the following arguments:
    ADC.ATTN_0DB — the full range voltage: 1.2V
    ADC.ATTN_2_5DB — the full range voltage: 1.5V
    ADC.ATTN_6DB — the full range voltage: 2.0V
    ADC.ATTN_11DB — the full range voltage: 3.3V
"""

### end 



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
    sensor_value = sensorPIN.value()
    sensor_state = "false"
    if sensor_value == 1:
        sensor_state = "true"
    log.info("Sensor Pin: %s=%s, state %s", sensorPIN, sensor_value, sensor_state)
    return (sensor_value, sensor_state)



def sleepMode():
    """ Matrix
     off  off  = 5 min
     on   off  = 10 sec
     off  on   = 60 sec
     on   on   = 15min
    """
    sleep1_value = sleep1PIN.value()
    sleep2_value = sleep2PIN.value()
    log.debug("Sleep Pin: %s=%s, %s=%s", sleep1PIN, sleep1PIN.value(), sleep2PIN, sleep2PIN.value())
    if sleep1_value == 0 and sleep2_value == 0:
        return 900
    elif sleep1_value == 1 and sleep2_value == 0:
        return 300
    elif sleep1_value == 0 and sleep2_value == 1:
        return 60
    elif sleep1_value == 1 and sleep2_value == 1:
        return 10
    else:
        return 10



def encrypt(plain):
    """"""
    key = secrets.aes_key
    iv = secrets.aes_iv

    log.debug("Using AES%s-CBC cipher", (len(key * 8)))

    cipher = myaes.new(key,myaes.MODE_CBC,iv)
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


    temperature_value, humidity_value, state = measure()

    sensor_value, sensor_state = sensor()

    dictionary = { 
                "node": config.node_id,
                "msg_num": counter,
                "temperature": temperature_value,
                "humidity": humidity_value,
                "sensor_value": sensor_value,
                "sensor_state": sensor_state,
                "vbattery": vbat(),
                "wakeup_reason": wake_reason()
            }

    display.text('Sensor:  {}'.format(sensor_value), 0, 20) # Text, X, Y
    display.text('Tempera: {}'.format(temperature_value), 0, 30) # Text, X, Y
    display.text('Luftfkt: {}'.format(humidity_value), 0, 40) # Text, X, Y
    display.show()

    # Convert dictionary to JSON string
    json_string = ujson.dumps(dictionary)
    log.debug('PLAIN: %s', json_string)
        
    # encrypt / encode base64
    enc = encrypt(json_string)
    log.debug('BASE64: %s', enc)
    enc = enc.strip()
        
    log.info("send as encrypted---")
    greenLedPIN.on()
    lora.println(enc)
    greenLedPIN.off()
 


def gotoSleep(display):
    """"""
    # get sleep mode
    sleep_ms = sleepMode() * 1000

    log.info("sleep for {} sec, see you later".format(int(sleep_ms/1000)))
        
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



def send(display,lora):
    while True:
        display.text('Lora Sender', 0, 0) # Text, X, Y
        display.show()
    
        outDriverPIN.on()
        sleep(2)
        process(display,lora)
        sleep(2)
        outDriverPIN.off()

        gotoSleep(display)

