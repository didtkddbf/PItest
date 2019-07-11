import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

GPIO.setup(6, GPIO.OUT) # output rf
GPIO.setup(13, GPIO.OUT) # output rf
GPIO.setup(19, GPIO.OUT) # output rf
GPIO.setup(26, GPIO.OUT) # output rf

# Initial state for LEDs:
print("Testing RF out, Press CTRL+C to exit")

try:
     print("set GIOP high")
     GPIO.output(6, GPIO.HIGH)
     GPIO.output(13, GPIO.HIGH)
     GPIO.output(19, GPIO.HIGH)
     GPIO.output(26, GPIO.HIGH)
     time.sleep(5)
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
   print("Keyboard interrupt")

except:
   print("some error")

finally:
   print("clean up")
   GPIO.cleanup() # cleanup all GPIOnano