import RPi.GPIO as gpio
import time

GpioPins = [[6 ,13,27], # FL
            [16,12,23], # FR
            [26,19,22], # BL
            [20,21,24]] # BR

gpio.setmode(gpio.BCM)

for ind, (in1, in2, pwm) in enumerate(GpioPins):
    # init
    gpio.setup(in1, gpio.OUT)
    gpio.setup(in2, gpio.OUT)
    gpio.setup(pwm, gpio.OUT)
    
    p = gpio.PWM(pwm, 10)
    p.start(0)
    
    for speed in (0, 10, 25, 50, 75, 100):
        p.ChangeDutyCycle(speed)
        
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
    
    p.stop()

gpio.cleanup()
