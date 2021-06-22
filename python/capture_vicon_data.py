import sys
sys.path.append("../extern/pyvicon/")

import json
import keyboard
import time
import simpleaudio as sa

from datetime import datetime
from threading import Thread
from numpy import arctan2, arcsin

from vicon_commander import *
from cflib.positioning.motion_commander import MotionCommander

class Logger(Thread):
    def __init__(self, filename, log_config_vicon, log_config_crazyflie, log_period_vicon=20, log_period_crazyflie=50):
        Thread.__init__(self)
        self.zero_time = datetime.now()
        self.log_config_vicon = log_config_vicon
        self.log_config_crazyflie = log_config_crazyflie
        self.abort = False
        self.cf = None
        self.vicon = None
       
        cflib.crtp.init_drivers(enable_debug_driver=False)
        drones = cflib.crtp.scan_interfaces()
        print("[Logger] Found ", drones)

        if not drones:
            print("[Logger] No drone found!")
            return

        if log_config_vicon is not None:
            self.vicon = ViconWrapper(ip="192.168.10.1", period=log_period_vicon, subjects=log_config_vicon, time0=self.zero_time, filename=filename)
        if log_config_crazyflie is not None:
            self.cf = CrazyFlieCommander(drones[0][0], log_list=log_config_crazyflie, sampling_period=log_period_crazyflie, time0=self.zero_time, filename=filename)

        self.start()

    def run(self):
        while(self.cf.is_connected != True):
            pass

        time.sleep(1)
        print("Logging starting...")
        time.sleep(1)
        if self.log_config_vicon is not None:
            self.vicon.logging_enabled(1)

        if self.log_config_crazyflie is not None:
            self.cf.logging_enabled(1)

            self.mc = MotionCommander(self.cf.scf)
            with self.mc as mc:

                print('Taking off!')
                mc.up(0.7)
                time.sleep(1)

                print('Moving up 0.2m')
                mc.up(0.7)
                # Wait a bit
                time.sleep(1)

                # There is a set of functions that move a specific distance
                # We can move in all directions
                print('Moving forward 0.5m')
                
                mc.forward(0.5)
                # Wait a bit
                time.sleep(10)

                # We land when the MotionCommander goes out of scope
                print('Landing!')

        while self.abort == False:
            pass

        self.stop()

        filename = 'beep.wav'
        wave_obj = sa.WaveObject.from_wave_file(filename)
        play_obj = wave_obj.play()
        play_obj.wait_done()  # Wa

    def stop(self):
        self.abort = True
        if self.log_config_vicon is not None:
            self.vicon.logging_enabled(0)
            self.vicon.save_log()
        if self.log_config_crazyflie is not None and self.cf is not None:
            self.cf.logging_enabled(0)     
            self.cf.save_log()
            print('Landing!')
            self.mc.land()


if __name__ == '__main__':
    log_config_vicon = None
    # Variables to log from CF
    log_config_crazyflie = ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimate.roll",
                    "stateEstimate.pitch", "stateEstimate.yaw"]

     ## run function in the background
    logger = Logger(filename=sys.argv[1], log_config_vicon=log_config_vicon, log_config_crazyflie=log_config_crazyflie)
    
    ## will not exit if function finishes, only when
    ## "q" is entered, but this is just a simple example
    while True:
        if keyboard.is_pressed('q'):
            break
   
    print("Aborting...")
    logger.stop()




   

   