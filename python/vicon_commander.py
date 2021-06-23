from pyvicon.pyvicon import *
from threading import Thread
import math
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
from datetime import datetime
import sys
import time


class ViconWrapper(Thread):
    def __init__(self, ip, period, subjects, time0, filename):
        Thread.__init__(self)
        self.ip = ip
        self.period = period
        self.vicon = []
        self.subjects = subjects
        self.send_data = None
        self.data_log = np.array([])
        self.data_logging_en = False
        self.text = " "
        self.start()
        self.position = dict([])
        self.quaternions = dict([])
        self.t0 = time0
        self.is_running = False
        self.filename = filename

    def run(self):
        self.connect()
        self.loop()

    def connect(self):
        self.vicon = PyVicon()
        print("SDK version : {}".format(self.vicon.__version__))
        print(self.vicon.connect(self.ip))
        print("Vicon connection status : {}".format(self.vicon.is_connected()))
        self.vicon.set_stream_mode(StreamMode.ServerPush)
        self.vicon.enable_segment_data()
        self.vicon.enable_marker_data()
        self.vicon.enable_unlabeled_marker_data()
        self.vicon.enable_device_data()
        self.vicon.set_axis_mapping(Direction.Forward, Direction.Left, Direction.Up)

    def loop(self):
        self.is_running = True
        while self.is_running:
            while self.vicon.get_frame() != Result.Success:
                time.sleep(self.period)
            subj_count = self.vicon.get_subject_count()
            for i in range(0, subj_count):
                name = self.vicon.get_subject_name(i)
                if name in self.subjects:
                    timestamp = round(1000 * (datetime.now() - self.t0).total_seconds(), 3)
                    pos = self.vicon.get_segment_global_translation(name, name)
                    quat = self.vicon.get_segment_global_quaternion(name, name)
                    if quat is not None:
                        pos = pos / 1000.0
                        self.position[name] = pos
                        self.quaternions[name] = quat
                        if self.data_logging_en:
                            self.log(timestamp, name + "_" + "posx", pos[0])
                            self.log(timestamp, name + "_" + "posy", pos[1])
                            self.log(timestamp, name + "_" + "posz", pos[2])
                            self.log(timestamp, name + "_" + "qw", quat[0])
                            self.log(timestamp, name + "_" + "qx", quat[1])
                            self.log(timestamp, name + "_" + "qy", quat[2])
                            self.log(timestamp, name + "_" + "qz", quat[3])
                        # if (self.send_data):
                        #     if name == "CFVLAD":
                        #         self.send_data([pos, quat])

            time.sleep(self.period / 1000.0)

    def log(self, timestamp, id_var, value):
        data_row = np.array([timestamp, id_var, value]).reshape(1, -1)
        if self.data_log.shape[0] == 0:
            self.data_log = data_row
        else:
            self.data_log = np.append(self.data_log, data_row, axis=0)

    def save_log(self):
        self.is_running = False
        np.savetxt(self.filename + "vicon.csv", self.data_log, fmt='%s', delimiter=',')
        print("Log Saved!")
        # if self.is_running: 
        #     time.sleep(0.01)
        #     self.join()

    def logging_enabled(self, val):
        if val == 0:
            self.data_logging_en = False
        else:
            self.data_logging_en = True



class CrazyFlieCommander(Thread):
    def __init__(self, uri, log_list, sampling_period, time0, filename):
        Thread.__init__(self)
        self.uri = uri
        self.period = sampling_period
        self.data_log = np.array([])
        self.data_logging_en = False
        self.log_list = log_list
        self.text = " "
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
        self.is_connected = False
        self.start()
        self.is_running = False
        self.t0 = time0
        self.filename = filename
        self.current_log = dict([])

    def run(self):
        self.is_running = True
        self.connect_tocf()
        while self.is_connected == False:
            time.sleep(0.01)

    def connect_tocf(self):
        # Connect some callbacks from the Crazyflie API
        self.scf.cf.connected.add_callback(self._connected)
        self.scf.cf.disconnected.add_callback(self._disconnected)
        self.scf.cf.connection_failed.add_callback(self._connection_failed)
        self.scf.cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % self.uri)

        # Try to connect to the Crazyflie
        self.scf.cf.open_link(self.uri)


    def _connected(self, link_uri):
        print('Connected to %s' % link_uri)
        self.is_connected = True
        if len(self.log_list) > 0:
            self.logging()

    def logging(self):
        N = len(self.log_list)
        print(self.log_list)
        logs_nr = math.ceil(N / 6.0)

        logs = []
        for i in range(logs_nr):
            logs.append(LogConfig(name="log" + str(i), period_in_ms=self.period))
        print("Logs added: ", logs_nr)

        for i in range(N):
            logs[i//6].add_variable(self.log_list[i], 'float')

        try:
            for i in range(logs_nr):
                current_log = logs[i]
                self.scf.cf.log.add_config(current_log)
                # This callback will receive the data
                current_log.data_received_cb.add_callback(self._stab_log_data)
                # Start the logging
                current_log.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def config(self):
        time.sleep(0.2)
        self.scf.cf.param.set_value('stabilizer.estimator', '2')
        # self.scf.cfparam.set_value('locSrv.extQuatStdDev', 0.05)
        # self.scf.cfparam.set_value('stabilizer.controller', '2')
        self.scf.cf.param.set_value('commander.enHighLevel', '1')
        print("CF configured!")
        print("Reset estimator")
        self.scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.scf.cf.param.set_value('kalman.resetEstimation', '0')

    def _stab_log_data(self, timestamp, data, logconf):
        t1 = datetime.now()
        out = np.fromiter(data.values(), dtype=float).reshape(1, -1)
        names = list(data.keys())
        if self.data_logging_en:    
            timestamp = round(1000 * (datetime.now() - self.t0).total_seconds(), 3)
            for i in range(len(names)):
                self.log(timestamp, names[i], out[0, i])
                self.current_log[names[i]] = out[0, i]
                # if names[i] == "MULTILATERATION.ESTX" or names[i] == "MULTILATERATION.ESTY" or names[i] == "MULTILATERATION.ESTZ" or names[i] == "DWM1000.dist_vicon" or \
                #     names[i] == "DWM1000.wx" or names[i] == "DWM1000.wy" or names[i] == "DWM1000.wz":
                #     print(names[i], out[0, i])
                #

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def send_extpose(self, cf, pos, quat):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]

        # scf.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
        scf.cf.extpos.send_extpos(x, y, z)

    def log(self, timestamp, id_var, value):
        data_row = np.array([timestamp, id_var, value]).reshape(1, -1)
        if self.data_log.shape[0] == 0:
            self.data_log = data_row
        else:
            self.data_log = np.append(self.data_log, data_row, axis=0)

    def save_log(self):
        np.savetxt(self.filename + "cf.csv", self.data_log, fmt='%s', delimiter=',')
        self.scf.cf.close_link()
        # if self.is_running:
        #     time.sleep(0.1)
        #     self.join()

    def logging_enabled(self, val):
        if val == 0:
            self.data_logging_en = False
        else:
            self.data_logging_en = True



