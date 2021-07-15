import logging
import sys
import os
import argparse
import coloredlogs
import signal
import time
import textwrap

# Custom user libraries
from stepStabilizer import MainCommunication

# Crazyflie libraries
import cflib
import cfclient
from cfclient.utils.config import Config
from cfclient.utils.config_manager import ConfigManager

logger = logging.getLogger(__name__)
            
def flight_script(cf_commander, state):
    try:
        logger.info('Taking off!')
        state = "TAKEOFF"
        cf_commander.take_off(0.4)
        
        time.sleep(1)

        logger.info('Moving forward')
        state = "FLY"
        cf_commander.forward(0.8, velocity=0.4)
        time.sleep(1)

        logger.info('Landing!')
        state = "LAND"
        cf_commander.land()
    except:
        pass

def main():
    # Console argument parser
    parser = argparse.ArgumentParser(
        prog="flight_script", 
        description='Logging and control scirpt for crazyflie with automatic step detection and correction. Authors: Philip Wiese, Luca Rufer',
        formatter_class=argparse.RawTextHelpFormatter
    )

    # Save log data from drone and crazyflie
    parser_log = parser.add_argument_group("Data Logging")
    parser_log.add_argument("-l", action="store", nargs='?', type=str,
                        default = "",
                        dest = "log_path",
                        help="Filename for log files. \r\ndefault: ''")
    parser_log.add_argument("--sv", action="store", nargs="+", type=str, dest="log_config_vicon",
                        default=['None'],
                        help="Vicon log entry to save. \r\ndefault: None")
    parser_log.add_argument("--exposeViconPos", type=str,
                        dest="exposeVicon",
                        default='None',
                        help="Send vicon object position to drone."
                        "default: None")
    parser_log.add_argument("--sc", action="store", nargs="+", type=str, dest="log_config_crazyflie",
                        default=["stateEstimate.z", "stateEstimate.vz", "posCtl.targetZ", "acc.z", "range.zrange"],
                        help="Crazyflie log entry to save. \r\n"
                             "default: 'stateEstimate.z' 'stateEstimate.vz' 'posCtl.targetZ' 'acc.z' 'range.zrange'")

    # Drone configuration
    parser_drone = parser.add_argument_group("Crazyflie Configuration")
    parser_drone.add_argument("-u", "--uri", action="store", dest="uri", type=str,
                        default="radio://0/80/2M",
                        help="URI to use for connection to the Crazyradio dongle. \r\n"
                             "default: radio://0/80/2M")
    parser_drone.add_argument("-i", action="store", dest="input",
                        type=str, default="xbox360_mode1",
                        help="Input mapping to use for the controller. \r\n"
                             "default: xbox360_mode1")
    parser_drone.add_argument("-c", action="store", type=int,
                        dest="controller", default=-1,
                        help="Use controller with specified id.\r\n"
                            "default: Use flight script")
    parser_drone.add_argument("--controllers", action="store_true",
                        dest="list_controllers",
                        help="Only display available controllers and exit")

    # Algorithm configuration 
    parser_algorithm = parser.add_argument_group("Algorithm Configuration")                   
    parser_algorithm.add_argument('--stepStabilizer',
                    type=int,
                    choices=range(0, 4),
                    default= 0,
                    help=textwrap.dedent('''\
                        Select step stabilizer algorithm.
                        \t0: None
                        \t1: Proof of Concept Python Algorithm
                        \t2: Filter Online Algorithm 
                        \t3: Machine Learning Online Algorithm
                        default: 0''')
    )
    # Misc configuration
    parser.add_argument('--verbose', '-v', action='count', default=0,
                    help="Verbosity (-vv for higher verbosity)")


    args, _ = parser.parse_known_args()

    if args.verbose > 1:
        coloredlogs.install(level='DEBUG')
    elif args.verbose > 0:
        coloredlogs.install(level='INFO')
        logger.debug(args)
    else:
        coloredlogs.install(level='WARN')

    logger.debug(args)

    ## Create main communication thread
    data_logger = MainCommunication(
        filename=args.log_path, 
        algorithm=args.stepStabilizer,
        log_config_vicon=args.log_config_vicon, 
        log_config_crazyflie=args.log_config_crazyflie,
        exposeVicon=args.exposeVicon,
        uri=args.uri)
    
    if (args.list_controllers):
        data_logger.cf.list_controllers()
        return

    if args.controller != -1:
        if not data_logger.cf.controller_connected():
            print("No input-device connected!")
            return

        data_logger.cf.setup_controller(input_config=args.input, input_device=args.controller)
        data_logger.start()

        while data_logger.is_alive():
            try:
                data_logger.join(0.1)
                time.sleep(0.1)
            except KeyboardInterrupt:
                # Ctrl-C handling and send kill to threads
                logging.warning("Exiting...")
                data_logger.is_running = False

                # Wait for all logs to be saved
                while data_logger.cf.is_running:
                    time.sleep(0.1)

                if (data_logger.vicon is not None):
                    while data_logger.vicon.is_running:
                        time.sleep(0.1)

                # Save landing
                while data_logger.cf.mc._is_flying:
                    data_logger.cf.mc.land()
                    time.sleep(1)
                os.kill(os.getpid(), signal.SIGKILL)
        return
    
    # Use flight script
    data_logger.set_flight_script(flight_script)
    data_logger.start()
    while data_logger.is_alive():
        try:
            data_logger.join(0.1)
            time.sleep(0.1)
        except KeyboardInterrupt:
            # Ctrl-C handling and send kill to threads
            logging.warning("Exiting...")
            data_logger.is_running = False

            # Save landing
            current_time = time.time()
            while (time.time()-current_time < 5):
                if (not data_logger.cf.mc._is_flying):
                    break
                try:
                    data_logger.cf.mc.land()
                except:
                    break
                time.sleep(1)
            
            data_logger.cf.save_log()
            if (data_logger.vicon is not None):
                data_logger.vicon.save_log()
                
            os.kill(os.getpid(), signal.SIGKILL)


if __name__ == '__main__':
    main()
