import RPi.GPIO as gpio

GpioPins = [[6 ,13,27], # FL
            [16,12,23], # FR
            [26,19,22], # BL
            [20,21,24]] # BR

PwmSpeeds = [0.6/0.60,
             0.3/0.60,
             0.6/0.60,
             0.3/0.60]

p = []

gpio.setmode(gpio.BCM)

try:
    for ind, (in1, in2, pwm) in enumerate(GpioPins):
        # init
        gpio.setup(in1, gpio.OUT)
        gpio.setup(in2, gpio.OUT)
        gpio.setup(pwm, gpio.OUT)
        
        p.append(gpio.PWM(pwm, 50))
        p[-1].start(PwmSpeeds[ind]*100)

        gpio.output(in1, False)
        gpio.output(in2, True)

    while True:
        pass
except Exception as e:
    print(e)
finally:
    for ind, (in1, in2, pwm) in enumerate(GpioPins):
        gpio.output(in1, False)
        gpio.output(in2, False)
    
    for i in p:
        i.stop()
    
    gpio.cleanup()

"""
600 mm in 2.35 sec
240 mm/s
wheel radius = 65 mm
48/13 rad/s

FL 0.60
BL 0.53
FR 0.50
BR 0.50
"""
