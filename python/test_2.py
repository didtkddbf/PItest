import time
import VL53L0X
import RPi.GPIO as GPIO
from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep

f = raw_input('file name : ')
filename = f + '.txt'
tdata = open(filename, 'a+')

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

tdata.write("Displacement mm Displacement cm Temperature 'c Time s")

a = 0
while a!=3:
    a= int(input('act=1,cool=2,stop=3 '))
    if a==1:
        c = int(input('time : '))
        b = c*2+1
        GPIO.output(Act, GPIO.LOW)
        for count in range(1,b):
            distance = tof.get_distance()
            Rdistance = distance*0.6931 + 16.375
            temp = max(sensor.readPixels())
            if (distance > 0):
                print("%d mm, %d cm, %d 'c, %f s" % (Rdistance, (distance/10), temp, count*0.5))
                tdata.write("%d mm, %d cm, %d 'c, %f s" % (Rdistance, (distance/10), temp, count*0.5))
            time.sleep(timing/200000.00)
        GPIO.output(Act, GPIO.HIGH)
    elif a==2:
        c = int(input('time : '))
        b = c*2+1
        GPIO.output(Fan, GPIO.LOW)
        for count in range(1,b):
            distance = tof.get_distance()
            Rdistance = distance*0.6931 + 16.375
            temp = max(sensor.readPixels())
            if (distance > 0):
                print("%d mm, %d cm, %d 'c, %f s" % (Rdistance, (distance/10), temp, count*0.5))
                tdata.write("%d mm, %d cm, %d 'c, %f s" % (Rdistance, (distance/10), temp, count*0.5))
            time.sleep(timing/200000.00)
        GPIO.output(Fan, GPIO.HIGH)

tof.stop_ranging()

tdata.close()