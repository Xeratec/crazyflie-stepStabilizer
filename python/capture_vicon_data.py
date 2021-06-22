import sys
sys.path.append("../extern/pyvicon/")

from datetime import datetime
from vicon_commander import *
import json
import keyboard
from numpy import arctan2, arcsin
import simpleaudio as sa
import time

file = sys.argv[1]
t0 = datetime.now()

to_log_vicon = ["Test"]
# Variables to log from CF
to_log_cf = ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimate.roll",
          "stateEstimate.pitch", "stateEstimate.yaw"]

vicon0 = ViconWrapper(ip="192.168.10.1", period=20, subjects=to_log_vicon, time0=t0, filename=file)
# cflib.crtp.init_drivers(enable_debug_driver=False)
# drones = cflib.crtp.scan_interfaces()
# cf0 = CrazyFlieCommander(drones[0][0], log_list=to_log_cf, sampling_period=50, time0=t0, filename=file)
# # sampling_rate: Not les than 30-40ms / variable
# while(cf0.is_connected != True):
#     wait = 1

time.sleep(1)
print("Logging starting...")
vicon0.logging_enabled(1)
time.sleep(1)
cf0.logging_enabled(1)
vicon0.logging_enabled(1)

# time.sleep(50)
while 1:
    time.sleep(1)
    if keyboard.is_pressed('s'):
        break

# cf0.logging_enabled(0)     
vicon0.logging_enabled(0)
vicon0.save_log()
# cf0.save_log()
print("Logging finished!")


filename = 'beep.wav'
wave_obj = sa.WaveObject.from_wave_file(filename)
play_obj = wave_obj.play()
play_obj.wait_done()  # Wait until sound has finished playing