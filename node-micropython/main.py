import esp32
import machine
from machine import Pin, SoftSPI, SoftI2C
from machine import deepsleep, wake_reason, reset_cause
from ssd1306 import SSD1306_I2C
from sx127x import SX127x

from examples import LoRaSender
from examples import _LoRaReceiver
from examples import _LoRaPing
from examples import _LoRaReceiverCallback

import logging


"""
LILYGO_TTGO_LORA32 T3 V1.6.1
"""

i2c_pins = {
    'sda': 21,
    'scl': 22
}

spi_pins = {
    'sck':5,
    'miso':19,
    'mosi':27
}

lora_pins = {
    'dio_0':26,
    'ss':18,
    'reset':16,   # oder 23
}

lora_default = {
    'frequency': 866000000,
    'frequency_offset':0,
    'tx_power_level': 14,
    'signal_bandwidth': 125e3,
    'spreading_factor': 7,
    'coding_rate': 5,
    'preamble_length': 8,
    'implicitHeader': False,
    'sync_word': 0x14,
    'enable_CRC': True,
    'invert_IQ': False,
    'debug': False,
}


lora_spi = SoftSPI(
    baudrate=10000000, polarity=0, phase=0,
    bits=8, firstbit=SoftSPI.MSB,
    sck=Pin(spi_pins['sck'], Pin.OUT, Pin.PULL_DOWN),
    mosi=Pin(spi_pins['mosi'], Pin.OUT, Pin.PULL_UP),
    miso=Pin(spi_pins['miso'], Pin.IN, Pin.PULL_UP),
)





type = 'sender'
# type = 'receiver'
# type = 'ping_master'
# type = 'ping_slave'
# type = 'receiver_callback'

log = logging.getLogger(__name__)

if __name__ == '__main__':

    wakeupPIN = Pin(34, mode = Pin.IN)
    esp32.wake_on_ext0(pin = wakeupPIN, level = esp32.WAKEUP_ANY_HIGH)

    debug = Pin(39, mode = Pin.IN)
    #logging.basicConfig(level=logging.INFO)
    log.setLevel(logging.INFO)
    if debug.value(): 
        log.setLevel(logging.DEBUG)
        #logging.basicConfig(level=logging.DEBUG)
        log.debug("DEBUG mode!!!")

    # check if the device woke from a deep sleep
    if reset_cause() == machine.DEEPSLEEP_RESET:
        log.info('***Wake from deep sleep, reason {}'.format(wake_reason()))

    log.debug("I2C initialisieren")
    i2c = SoftI2C(sda=Pin(i2c_pins['sda']), scl=Pin(i2c_pins['scl']))

    log.debug("OLED-Display initialisieren")
    oled_width = 128 # Breite: Pixel
    oled_height = 64 # Höhe: Pixel
    display = SSD1306_I2C(oled_width, oled_height, i2c)

    log.debug("SX127x initialisieren")
    lora = SX127x(lora_spi, pins=lora_pins, parameters=lora_default)

    if type == 'sender':
        LoRaSender.new(display, lora)
    """
    if type == 'receiver':
        LoRaReceiver.receive(lora)
    if type == 'ping_master':
        LoRaPing.ping(lora, master=True)
    if type == 'ping_slave':
        LoRaPing.ping(lora, master=False)
    if type == 'receiver_callback':
        LoRaReceiverCallback.receiveCallback(lora)
    """


    # Following a normal Exception or main() exiting, reset the board.
    # Following a non-Exception error such as KeyboardInterrupt (Ctrl-C),
    # this code will drop to a REPL. Place machine.reset() in a finally
    # block to always reset, instead.
    machine.reset()    