import struct
import smbus
import time
import RPi.GPIO as GPIO

INTERRUPT_PIN = 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(INTERRUPT_PIN,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def get_data():
    return bus.read_i2c_block_data(address, 0);

def get_float(data, index):
    bytes = data[4*index:(index+1)*4]
    return struct.unpack('f', "".join(map(chr, bytes)))[0]

while True:
    GPIO.wait_for_edge(INTERRUPT_PIN, GPIO.RISING)

    try:
        data = get_data()
        latitude = get_float(data, 0)/100
        longitude = get_float(data, 1)/100
        buf = get_float(data, 5)
        print('Trigger detected!')
        if int(buf) == 78:
            lat = 'N'
        else: 
            lat = 'S'
            latitude = latitude * -1
        lon = get_float(data, 6)
        if int(lon) == 87: 
            lon = 'W'
            longitude = longitude * -1
        else: 
            lon = 'E'
        hour = int(get_float(data, 2))
        minute = int(get_float(data, 3))
        second = int(get_float(data, 4))
        print"Latitude: ",latitude,lat
        print"Longitude: ",longitude,lon
        print"Time: ",hour,":",minute,":",second
    except:
        continue
    time.sleep(1);
