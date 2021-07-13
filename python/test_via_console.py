"""
This script shows a simple scripted flight path using the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
import sys
import os

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.crtp.crtpstack import CRTPPort
from cflib.utils.callbacks import Caller

URI = 'radio://0/81/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class Console:
    def __init__(self, crazyflie):
        """
        Initialize the console and register it to receive data from the copter.
        """

        self.receivedChar = Caller()
        self.text = ""

        self.cf = crazyflie
        self.cf.add_port_callback(CRTPPort.CONSOLE, self.incoming)

    def incoming(self, packet):
        """
        Callback for data received from the copter.
        """
        console_text = packet.data.decode('UTF-8')

        self.text = self.text + console_text

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:

        console = Console(scf.cf)

        # We take off when the commander is created
        with MotionCommander(scf, default_height = 0.40) as mc:

            print('Taking off!')
            time.sleep(0.5)

            # move forwards
            print('Moving forward 1m')
            mc.forward(1, velocity=0.3)

            # Wait a bit and then land
            time.sleep(0.5)
            print('Landing!')

            # Save the log. Don't save filter anotations
            console_log = console.text.replace("ESTKALMAN: State out of bounds, resetting", "")
            log_name = "" if len(sys.argv) <= 1 else sys.argv[1]
            script_dir = os.path.dirname(os.path.abspath(__file__))
            target = os.path.join(script_dir, os.path.join("logs", "console_logs", "test" + log_name + ".csv"))
            f = open(target, "w")
            f.write(console.text)
            f.close()