import time
import VL53L0X
import RPi.GPIO as GPIO
from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep

GPIO.setmode(GPIO.BCM)

Act = 26
Actt = 19
Fan = 13
Fann = 6
GPIO.setup(Act,GPIO.OUT)
GPIO.setup(Actt,GPIO.OUT)
GPIO.setup(Fan,GPIO.OUT)
GPIO.setup(Fann,GPIO.OUT)

# Create a VL53L0X object
tof = VL53L0X.VL53L0X()
sensor = Adafruit_AMG88xx()

# Start ranging
#wait for it to boot
tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
sleep(.1)

timing = tof.get_timing()
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))

a, b = input('a,c time: ').split()
b = int(b)

for count in range(1,1010):
    distance = tof.get_distance()
    Rdistance = distance*0.6931 + 16.375
    temp = max(sensor.readPixels())
    if (distance > 0):
        print ("%d mm, %d cm, %d 'c, %d" % (Rdistance, (distance/10), temp, count))
    time.sleep(timing/100000.00)

tof.stop_ranging()
