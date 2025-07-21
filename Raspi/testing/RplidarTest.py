from rplidar_sdk.output.Linux.Release import rplidar
import time
lidar = rplidar.RPLidar()
lidar.connect("/dev/ttyUSB0")
lidar.start_scan()

while True:
    scan = lidar.get_scan()
    if scan:
        print(scan)
        print("\n\n\n\n")
        print([i for j in scan for i in j])
        print("\n\n\n\n")
        break

for _ in range(10):
    scan = lidar.get_scan()
    scan = [i for j in scan for i in j]
    print(f"Got {len(scan)} measurements")
    time.sleep(0.1)

lidar.stop_scan()
