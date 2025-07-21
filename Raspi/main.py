import utils

GpioPins = [[6 ,13,27], # FL
            [16,12,23], # FR
            [26,19,22], # BL
            [20,21,24]] # BR

MaxRpm = 48/13*60*0.45

PwmSpeeds = [0.60/0.60,
             0.27/0.60,
             0.50/0.60,
             0.27/0.60]

CarController = utils.CarController(GpioPins, MaxRpm, FilterOutBackwards=True, MotorCalibrationSpeeds=PwmSpeeds)

try:
    while True:
        CarController.main()

finally:
    CarController.clean()