import RPi.GPIO as gpio
import time

GpioPins = [[6 ,13], # FL
            [16,12], # FR
            [26,19], # BL
            [20,21]] # BR

gpio.setmode(gpio.BCM)

for ind, (in1, in2) in enumerate(GpioPins):
    # init
    gpio.setup(in1, gpio.OUT)
    gpio.setup(in2, gpio.OUT)
    
    # forward
    print(f"motor {ind} forward")
    gpio.output(in1, False)
    gpio.output(in2, True)
    time.sleep(1)
    
    # stop
    gpio.output(in1, False)
    gpio.output(in2, False)
    time.sleep(0.5)
    
    # backward
    print(f"motor {ind} backward\n")
    gpio.output(in1, True)
    gpio.output(in2, False)
    time.sleep(1)
    
    # stop
    gpio.output(in1, False)
    gpio.output(in2, False)
    time.sleep(0.5)

gpio.cleanup()