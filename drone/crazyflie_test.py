import logging
import time

from simple_pid import PID

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Drone movement parameters
HOVER_HEIGHT = 0.5
MIN_THRUST = 10001 # Absolute min supported by crazyflie is 10001, may need to use a higher value to fly?
MAX_THRUST = 40000 # NOTE: max supported is 60000, starting with this for safety
DELTA_TIME = 0.02 # In seconds
LOG_INTERVAL_MS = 100 # In milliseconds
HOVER_DURATION = 5 # In seconds
LANDING_SPEED = 0.2 # In m/s
LANDING_DECREMENT = 0.1 # In meters

# Globals
_drone_z_position = 0.0

logging.basicConfig(level=logging.INFO)

# Unlock startup thrust protection
# Must be done before sending other movement setpoint commands
def unlock_setpoint_commands(cf):
    cf.commander.send_setpoint(0, 0, 0, 0)

# Logger for position data
def set_drone_z_position(timestamp, log_data, log_config_name):
    global _drone_z_position
    PARAMETER_NAME = 'stateEstimate.z'

    _drone_z_position = log_data[PARAMETER_NAME] # Update z position global variable
    # print(log_data)
    # print('[%d][%s]: %s = %.3f' % (timestamp, log_config_name, PARAMETER_NAME, log_data[PARAMETER_NAME]))

def drone_z_pos_logging_error(log_config_name, msg):
    print('Error in logging [%s]: %s', (log_config_name, msg))

# Hover to start position
# cf - crazyflie object
def run_hover_sequence(cf):
    hover_start_time = time.time()
    current_height = _drone_z_position
    print("Starting at z=%.3f" % current_height)
    while (time.time() - hover_start_time < HOVER_DURATION): # Run hover loop for specified number of seconds
        cf.commander.send_hover_setpoint(0, 0, 0, HOVER_HEIGHT)
        time.sleep(DELTA_TIME)
    current_height = _drone_z_position
    print("Hovering at z=%.3f" % current_height)

# Landing sequence
# Immediately sends a hover setpoint to stop the drone's flight, then descends slowly
def run_landing_sequence(cf):
    LAND_HEIGHT = 0.05 # Height in meters after which the drone will drop
    while (_drone_z_position > (LAND_HEIGHT + 0.05)):
        print('in land loop')
        new_height = max(_drone_z_position - LANDING_DECREMENT, LAND_HEIGHT)
        print(new_height) # TODO: remove
        cf.commander.send_hover_setpoint(0, 0, 0, new_height)
        time.sleep(0.5)
        cf.commander.send_hover_setpoint(0, 0, 0, new_height)
        time.sleep(0.5)

# Run flight sequence
# cf - crazyflie object
# mc - MotionController object
def run_flight_sequence(cf):
    ROT_SPEED = 1 # Rotation speed in rad/s # TODO: match to paper's assumptions?

    print("Starting flight sequence")

    # Flight sequence parameters
    t_r = 1 # Roll switch time
    t_t = 1 # Thrust switch time
    t_tot = 5 # Total flight time

    current_time = 0
    roll = 0 # Start with no roll
    thrust = 0

    while (current_time < t_tot):
        # Set thrust and roll for next time segment
        thrust = MIN_THRUST if (current_time < t_t) else MAX_THRUST
        roll = roll + ROT_SPEED * DELTA_TIME if (current_time < t_r) else roll

        # Move
        cf.commander.send_setpoint(roll, 0, 0, thrust)

        # Update time tracker and sleep
        current_time += DELTA_TIME
        time.sleep(DELTA_TIME)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Set up logging config
        log_config = LogConfig(name='Position', period_in_ms=LOG_INTERVAL_MS)
        log_config.add_variable('stateEstimate.z', 'float')
        cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(set_drone_z_position)
        log_config.error_cb.add_callback(drone_z_pos_logging_error)
        log_config.start()

        print("~~~~~~ Crazyflie connected ~~~~~~")

        time.sleep(1)
        print('~~~~~~ Running hover sequence ~~~~~~')
        unlock_setpoint_commands(cf)
        run_hover_sequence(cf)

        print('~~~~~~ Running flight sequence ~~~~~~')
        # run_flight_sequence(cf)

        print('~~~~~~ Running landing sequence ~~~~~~')
        run_landing_sequence(cf)

        # Old hover method using MotionCommander - doesn't seem to allow flight sequence to run :(
        # with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
        #     time.sleep(10) # Wait for automatic hover sequence to complete
        #     # Movement commands here


        time.sleep(1)
        print("~~~~~~ Crazyflie disconnected ~~~~~~")

