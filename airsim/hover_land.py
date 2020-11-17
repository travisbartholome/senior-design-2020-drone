import airsim
from simple_pid import PID

import time

# Connect to simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Flight sequence
# client.takeoffAsync().join()

Z_TARGET = -1 # Target hover height

# test sensor reading
# print(client.getDistanceSensorData("Distance1", "Drone1"))

# fly up and hover
MIN_THRUST = 0.48 # Min thrust to overcome gravity, minus a little # TODO: determine via a starting/calibration sequence
# Note: min thrust to overcome gravity: 0.58 in simulation
MAX_THRUST = 1.0 # TODO: adjust?
# Note: max thrust supported in airsim: 1
DELTA_TIME = 0.01 # Crazyflie docs suggest tick rate of 100Hz
# thrust = 0 # TODO: remove
lastHeight = 0
currentHeight = client.getMultirotorState().kinematics_estimated.position.z_val
startTime = time.time()
hoverPid = PID(
    Kp=0.4,
    Ki=1,
    Kd=1,
    setpoint=-Z_TARGET, # Negative so that output will have the right sign
    sample_time=DELTA_TIME,
    output_limits=(MIN_THRUST, MAX_THRUST))
thrust = hoverPid(-currentHeight) # Negative so that output will have the right sign
while (time.time() - startTime < 20): # run control loop for 20 seconds

    # TODO: use a more nuanced thrust calculation
    # if (currentHeight > Z_TARGET): # If below target height (> implies lower)
    #     thrust = 0.4 * (min(currentHeight - Z_TARGET, 1) * (MAX_THRUST - MIN_THRUST)) + MIN_THRUST
    # else:
    #     thrust = MIN_THRUST
    client.moveByRollPitchYawThrottleAsync(0, 0, 0, thrust, DELTA_TIME).join()
    lastHeight = currentHeight
    currentHeight = client.getMultirotorState().kinematics_estimated.position.z_val
    thrust = hoverPid(-currentHeight) # Negative so that output will have the right sign

    # time.sleep(DELTA_TIME)

print(client.getMultirotorState().kinematics_estimated.position.z_val)

# Try using moveByRollPitchYawrateThrottleAsync
# This seems to be what the Crazyflie API exposes?
# client.moveByRollPitchYawThrottleAsync(1, 0, 0, 1, 1).join()
# client.moveByRollPitchYawThrottleAsync(0, 1, 0, 1, 1).join()

# Clean up
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
