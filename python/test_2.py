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

tdata.write("Cal_Disp, mm, Sen_Disp, mm, Temperature, 'c, Time, s \n")

a = 0
while a!=3:
    a= int(input('act=1,cool=2,stop=3 '))
    if a==1:
        c = int(input('time : '))
        b = c*5+1
        GPIO.output(Act, GPIO.LOW)
        tdata.write("Cal_Disp, mm, Temperature, 'c, Time, s \n")
        for count in range(1,b):
            distance = tof.get_distance()
            Rdistance = distance*0.6931 + 16.375
            temp = max(sensor.readPixels())
            if (distance > 0):
                print("%d mm, %d 'c, %f s" % (Rdistance, temp, count*0.2))
                tdata.write("%d mm, %d 'c, %f s \n" % (Rdistance, temp, count*0.2))
            time.sleep(timing/500000.00)
        GPIO.output(Act, GPIO.HIGH)
        for count in range(b,b+51):
            print("%d mm, %d 'c, %f s" % (Rdistance, temp, count*0.2))
            tdata.write("%d mm, %d 'c, %f s \n" % (Rdistance, temp, count*0.2))
            time.sleep(timing/500000.00)
    elif a==2:
        c = int(input('time : '))
        b = c*5+1
        GPIO.output(Fan, GPIO.LOW)
        tdata.write("Cal_Disp, mm, Sen_Disp, mm, Temperature, 'c, Time, s \n")
        for count in range(1,b):
            distance = tof.get_distance()
            Rdistance = distance*0.6931 + 16.375
            temp = max(sensor.readPixels())
            if (distance > 0):
                print("%d mm, %d 'c, %f s" % (Rdistance, (distance/10), temp, count*0.2))
                tdata.write("%d mm, %d 'c, %f s \n" % (Rdistance, (distance/10), temp, count*0.2))
            time.sleep(0.5)
        GPIO.output(Fan, GPIO.HIGH)

tof.stop_ranging()

tdata.close()
