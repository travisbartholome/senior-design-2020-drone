import logging
import time

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

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        print("~~~~~~ Crazyflie connected ~~~~~~")

        time.sleep(1)

        
        with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
            time.sleep(10) # Wait for automatic hover sequence to complete
            # Movement commands here


        time.sleep(3)
        print("~~~~~~ Crazyflie disconnected ~~~~~~")

