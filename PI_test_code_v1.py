import RPi.GPIO as GPIO
import time
from Adafruit_AMG88xx import Adafruit_AMG88xx
import serial


sensor = Adafruit_AMG88xx(address=0x69, busnum=2)

#-----------Force gauage Serial setting---------------
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=38400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0)

#------------------------------------------------------

Act = 13
Act_dirA = 14
Act_dirB = 15
Fan = 16
encPinA = 23
encPinB = 24


#----------------ENCODER-------------------------
GPIO.setup(Act_dirA, GPIO.OUT)
GPIO.setup(Act_dirB, GPIO.OUT)
GPIO.setup(Fan, GPIO.OUT)

GPIO.setmode(IO.BCM)
GPIO.setwarnings(False)
GPIO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
GPIO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
#-----------------------------------------------
encoderPos = 0
def encoderA(channel):
    global encoderPos
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPos += 1
    else:
        encoderPos -= 1
    print('PinA : %d, encoder : %d' % (channel, encoderPos))


def encoderB(channel):
    global encoderPos
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1
    print('PinB : %d, encoder : %d' % (channel, encoderPos))

GPIO.add_event_detect(encPinA, GPIO.BOTH, callback=encoderA)
GPIO.add_event_detect(encPinB, GPIO.BOTH, callback=encoderB)
#--------------ENCODER--------------------------

PWM.start(Act, 0, 1000)

GPIO.output(Act_dirA, GPIO.HIGH)
GPIO.output(Act, GPIO.LOW)
GPIO.output(Act_dirB, GPIO.LOW)
GPIO.output(Fan, GPIO.HIGH)
GPIO.output(Fann, GPIO.HIGH)


f = raw_input('file name : ')
filename = f + '.txt'
tdata = open(filename, 'a+')

tdata.write("Test start,start,start,start,start\n")
a=0
R = 0
pwm = 20

#----------Force sensing-------------
def Force_sensing():
    force = 0.000
    ser.write(b'RDF0\r')
    serialData = ser.read(ser.inWaiting())
    serialString = serialData.decode()
    serialList = re.findall("\d+", serialString)
    try:
        force = int(serialList[0]) + int(serialList[1]) / 1000
    except:
        continue
    if "-" in serialString:
        force = -force
    return force
#----------------------------------------------

#main function
try:
    while True :
        a = int(input('act=1, cool=2, cycle=3, stop=4, zero=5, PWMset=6 '))

        if a==1:
            c = int(input('heating time(s) : '))
            b = c*10+1
            tdata.write("Cal_Disp(mm),Temperature('c),Resistance(ohm),Time(s) \n")
            for count in range(1,b):
                PWM.set_duty_cycle(Act, pwm)
                distance = encoderPos * 0.04
                temp = max(sensor.readPixels())
                force = Force_sensing()
                print("%.2f,%d,%.3f,%.1f\n" % (distance, temp, force, count * 0.1))
                tdata.write("%.2f,%d,%.3f,%.1f\n" % (distance, temp, force, count * 0.1))
                time.sleep(0.1)
            PWM.set_duty_cycle(Act, 0)
            for count in range(b,b+1):
                distance = encoderPos * 0.04
                temp = max(sensor.readPixels())
                force = Force_sensing()
                print("%.2f,%d,%.3f,%.1f\n" % (distance, temp, force, count * 0.1))
                tdata.write("%.2f,%d,%.3f,%.1f\n" % (distance, temp, force, count * 0.1))
                time.sleep(0.1)

        elif a==2:
            c = int(input('cooling time(s) : '))
            b = c*5+1
            GPIO.output(Fan, GPIO.LOW)
            tdata.write("Cal_Disp, mm, Sen_Disp, mm, Temperature, 'c, Time, s \n")
            for count in range(1,b):
                distance = encoderPos * 0.04
                temp = max(sensor.readPixels())
                GPIO.output(Fan, GPIO.LOW)
                force = Force_sensing()
                print("%.2f,%d,%.3f,%.1f\n" % (distance, temp, force, count * 0.1))
                tdata.write("%.2f,%d,%.3f,%.1f\n" % (distance, temp, force, count * 0.1))
                time.sleep(0.1)
            GPIO.output(Fan, GPIO.HIGH)

        elif a==3:
            ht = int(input('High temp : '))
            ct = int(input('Low temp : '))
            cy = int(input('cycle number : '))
            tdata.write("Cal_Disp(mm),Temperature('c),Resistance(Ohm),Time(s),Cycle(n) \n")
            distance = 0.00
            count = 0.00
            temp = 0.00
            R = 0.00
            for cycle in range(1, cy):
                while temp < ht:
                    PWM.set_duty_cycle(Act, pwm)
                    distance = -encoderPos * 0.04
                    temp = max(sensor.readPixels())
                    count = count+1
                    force = Force_sensing()
                    print("%.2f,%.2f,%.3f,%.1f,%d" % (distance, temp, force, count * 1, cycle))
                    tdata.write("%.2f,%.2f,%.3f,%.1f,%d\n" % (distance, temp, force, count * 1, cycle))
                    time.sleep(0.1)
                PWM.set_duty_cycle(Act, 0)
                while temp > ct:
                    GPIO.output(Fan, GPIO.LOW)
                    distance = -encoderPos * 0.04
                    temp = max(sensor.readPixels())
                    count = count+1
                    force = Force_sensing()
                    print("%.2f,%.2f,%.3f,%.1f,%d" % (distance, temp, force, count * 1, cycle))
                    tdata.write("%.2f,%.3f,%.2f,%.1f,%d\n" % (distance, temp, force, count * 1, cycle))
                    time.sleep(0.1)
                GPIO.output(Fan, GPIO.HIGH)
                PWM.set_duty_cycle(Act, 0)
        elif a==4:
            break
        elif a==5:
            encoderPos = 0
        elif a==6:
            pwm = int(input('PWM value(1~100) : '))
        elif a==7:
            print(pwm)
except KeyboardInterrupt:
    break
    print('Emergency Stop')

PWM.stop(act)
PWM.cleanup()
GPIO.cleanup()
tdata.close()

