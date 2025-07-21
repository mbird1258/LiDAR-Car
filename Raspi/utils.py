from rplidar_sdk.output.Linux.Release import rplidar
import RPi.GPIO as gpio
import time
import numpy as np
from client import get, send

gpio.setmode(gpio.BCM)

class CarController:
    def __init__(self, GpioPins, MaxRpm, FilterOutBackwards=False, MinQuality=3, MotorCalibrationSpeeds = (1,1,1,1), FailsafeTimeLimit = 3, LidarPort = '/dev/ttyUSB0'):
        for ind, (in1, in2, pwm) in enumerate(GpioPins):
            gpio.setup(in1, gpio.OUT)
            gpio.setup(in2, gpio.OUT)
            gpio.setup(pwm, gpio.OUT)
            
            GpioPins[ind].append(gpio.PWM(pwm, 50))
            GpioPins[ind][-1].start(0)
        
        self.m1gpio, self.m2gpio, self.m3gpio, self.m4gpio = GpioPins
        self.MotorCalibrationSpeeds = MotorCalibrationSpeeds
        
        self.TimeLimit = FailsafeTimeLimit
        
        self.lidar = rplidar.RPLidar()
        self.lidar.connect(LidarPort)
        self.lidar.start_scan()
        
        self.waypoints = []
        self.MaxRpm = MaxRpm
        self.NextMotorUpdateTime = -1
        
        self.SendLidarFlag = False
        self.LastLidarUpdateTime = None
        
        self.FilterOutBackwards = FilterOutBackwards
        self.MinQuality = MinQuality

    def GetLidar(self):
        scans = self.lidar.get_scan()
        
        # Preprocessing
        if not scans:
            return False
        scans = [i for j in scans for i in j]
        
        # Get time of each scan before culling some to avoid inaccuracy
        if self.LastLidarUpdateTime:
            t = time.time()
            times = np.linspace(self.LastLidarUpdateTime, t, len(scans))
        
        # Cull bad scans
        temp1 = []
        temp2 = []
        for ind, (quality, angle, distance) in enumerate(scans):
            if quality > self.MinQuality and (angle < 90 or angle > 270 or not self.FilterOutBackwards):
                temp1.append([angle, distance])
                if self.LastLidarUpdateTime: temp2.append(times[ind])
        scans = temp1
        times = temp2
        
        # Add time data to scan
        if not self.LastLidarUpdateTime:
            # Edge case of first scan
            self.LastLidarUpdateTime = time.time()
            scans = [i + [self.LastLidarUpdateTime] for i in scans]
        else:
            scans = [i + [j] for i, j in zip(scans, times)]
            self.LastLidarUpdateTime = t
        
        # Cull if more than one revolution is returned in one scan
        scans = np.array(scans)
        RelAngles = scans[:, -1] - scans[0, -1] % 360 # angles relative to first data entry in scans
        mask = RelAngles[1:] < RelAngles[:-1]
        if np.any(mask):
            ind = np.nonzero(mask)[0][0]+1 # first index where angle is back to or after the first angle in scans
            scans = scans[:ind]
        
        return scans.tolist()

    def ChangeDriving(self, m1AngVel, m2AngVel, m3AngVel, m4AngVel, reverse=True):
        MotorGpios = [self.m1gpio, self.m2gpio, self.m3gpio, self.m4gpio]
        MotorAngVels = [m1AngVel, m2AngVel, m3AngVel, m4AngVel]
        
        for MotorInd in range(4):
            in1, in2, _, pwm = MotorGpios[MotorInd]
            AngVel = MotorAngVels[MotorInd]
            rpm = 30/np.pi * AngVel * (-1 if reverse else 1)
            
            if rpm > 0:
                gpio.output(in1, True)
                gpio.output(in2, False)
            else:
                gpio.output(in1, False)
                gpio.output(in2, True)
            
            pwm.ChangeDutyCycle(np.abs(rpm)/self.MaxRpm*100*self.MotorCalibrationSpeeds[MotorInd])

    def UpdateDriving(self):
        if len(self.waypoints) == 0 or time.time() < self.NextMotorUpdateTime:
            return
        
        m1, m2, m3, m4, dt = self.waypoints[0]
        self.waypoints = self.waypoints[1:]
        
        self.ChangeDriving(m1, m2, m3, m4)
        self.NextMotorUpdateTime = time.time()+dt

    def UpdateSendLidar(self):
        if self.SendLidarFlag:
            LidarOut = self.GetLidar()
            if LidarOut:
                send(LidarOut)
                self.SendLidarFlag = False

    def main(self):
        out = get(blocking=False)
        if not out:
            self.UpdateDriving()
            self.UpdateSendLidar()
            return
        
        flag = out[0]
        
        if flag == "Get Lidar":
            self.SendLidarFlag = True
            
            out = get(blocking=False)
            if not out:
                self.UpdateDriving()
                self.UpdateSendLidar()
                return
            
            flag = out[0]
        
        
        if flag == "Drive":
            self.waypoints = out[1]
        
        self.UpdateDriving()
        self.UpdateSendLidar()

    def clean(self):
        self.lidar.stop_scan()
        
        for in1, in2, _, pwm in [self.m1gpio, self.m2gpio, self.m3gpio, self.m4gpio]:
            gpio.output(in1, False)
            gpio.output(in2, False)
            pwm.stop()
        
        gpio.cleanup()