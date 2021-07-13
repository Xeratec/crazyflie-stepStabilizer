import math
import time
import logging


from datetime import datetime
from queue import Empty
from queue import Queue
from threading import Thread

from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

logger = logging.getLogger(__name__)

class StableMotionCommander(MotionCommander):
    """The stable motion commander"""
    VELOCITY = 0.2
    RATE = 360.0 / 5

    def __init__(self, crazyflie, default_height=0.5, time0 = 0):
        """
        Construct an instance of a MotionCommander

        :param crazyflie: a Crazyflie or SyncCrazyflie instance
        :param default_height: the default height to fly at
        """
        self.name = "StableMotionCommander"
        MotionCommander.__init__(self, crazyflie, default_height)
        self.t0 = time0

    def take_off(self, height=None, velocity=VELOCITY):
        """
        Takes off, that is starts the motors, goes straight up and hovers.
        Do not call this function if you use the with keyword. Take off is
        done automatically when the context is created.

        :param height: the height (meters) to hover at. None uses the default
                       height set when constructed.
        :param velocity: the velocity (meters/second) when taking off
        :return:
        """
        if self._is_flying:
            raise Exception('[SMC] Already flying')

        if not self._cf.is_connected():
            raise Exception('[SMC] Crazyflie is not connected')

        self._is_flying = True
        self._reset_position_estimator()

        self._thread = _StableSetPointThread(self._cf, time0 = self.t0)
        self._thread.start()

        if height is None:
            height = self.default_height

        self.up(height, velocity)

    def set_z_offset(self, offset):
        if self._thread is not None:
            self._thread.set_z_offset(offset)

class _StableSetPointThread(Thread):
    TERMINATE_EVENT = 'terminate'
    UPDATE_PERIOD = 0.2
    ABS_Z_INDEX = 3

    def __init__(self, cf, update_period=UPDATE_PERIOD, time0 = 0):
        Thread.__init__(self)
        self.update_period = update_period

        self._queue = Queue()
        self._cf = cf

        self._hover_setpoint = [0.0, 0.0, 0.0, 0.0]

        self._z_base = 0.0
        self._z_velocity = 0.0
        self._z_base_time = 0.0

        self._z_offset = 0
        self.t0 = time0

    def stop(self):
        """
        Stop the thread and wait for it to terminate

        :return:
        """
        self._queue.put(self.TERMINATE_EVENT)
        self.join()

    def set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        """Set the velocity setpoint to use for the future motion"""
        self._queue.put((velocity_x, velocity_y, velocity_z, rate_yaw))

    def get_height(self):
        """
        Get the current height of the Crazyflie.

        :return: The height (meters)
        """
        return self._hover_setpoint[self.ABS_Z_INDEX]

    def set_z_offset(self, offset_z):
        self._z_offset = offset_z
        self._update_z_in_setpoint()
        self._cf.commander.send_hover_setpoint(*self._hover_setpoint)

    def run(self):
        while True:
            try:
                event = self._queue.get(block=True, timeout=self.update_period)
                if event == self.TERMINATE_EVENT:
                    return

                self._new_setpoint(*event)
            except Empty:
                pass

            self._update_z_in_setpoint()
            self._cf.commander.send_hover_setpoint(*self._hover_setpoint)

    def _new_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        self._z_base = self._current_z()
        self._z_velocity = velocity_z
        self._z_base_time = time.time()

        self._hover_setpoint = [velocity_x, velocity_y, rate_yaw, self._z_base]
    
    def _update_z_in_setpoint(self):
        z_setpoint = max(0, self._current_z() + self._z_offset)
            
        self._hover_setpoint[self.ABS_Z_INDEX] = z_setpoint
        
    def _current_z(self):
        now = time.time()
        return self._z_base + self._z_velocity * (now - self._z_base_time)




