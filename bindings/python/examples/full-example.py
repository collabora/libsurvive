import sys
import pysurvive

ctx = pysurvive.init(sys.argv)

if ctx is None: # implies -help or similiar
    exit(-1)

def imu_func(ctx, mode, accelgyro, timecode, id):
    print(accelgyro)

keepRunning = True

pysurvive.install_imu_fn(ctx, imu_func)
while keepRunning and pysurvive.poll(ctx) == 0:
    pass

pysurvive.close(ctx)
