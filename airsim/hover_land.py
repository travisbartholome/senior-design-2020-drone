# Dependency imports
import airsim
from simple_pid import PID

# Standard imports
import time

def getDroneZPosition(multirotorClient):
    """
    Method to get the height of the drone.
    In simulation, we can just get this from built-in kinematics estimations.

    On the real drone, we'd want to read this from the distance/altitude sensor,
    and there won't be an airsim client object anyway, so the body and signature
    of this method should be changed.

    Args:
        multirotorClient (airsim.MultirotorClient): The airsim client object

    Returns:
        zPosition (float): The z position of the drone associated with this client
    """
    return multirotorClient.getMultirotorState().kinematics_estimated.position.z_val

# ~~~~~~~~~~~~~~~~~~~~

# Connect to simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Define system parameters
GROUND_Z_VAL = getDroneZPosition(client) # Starting height is considered the "ground"
Z_TARGET = -1 # Target hover height

MIN_THRUST = 0.53 # Min thrust to overcome gravity, minus a little
# TODO: determine min thrust via a starting/calibration sequence?
MAX_THRUST = 1.0
# Note: min thrust to overcome gravity: 0.58 in simulation
# Note: max thrust supported in airsim: 1
DELTA_TIME = 0.01 # Crazyflie docs suggest tick rate of 100Hz
startTime = time.time()

# Create movement controller
# Note: airsim uses a coordinate system where -z is up and +z is down
# So to get the PID controller to apply thrust to go in a negative direction,
#   all of our controller parameters must be negative
# On the real drone, this might not apply - PID parameters will probably
#   have to be re-tuned for the real drone
hoverPid = PID(
    Kp=-0.4,
    Ki=-1,
    Kd=-1,
    setpoint=Z_TARGET,
    sample_time=DELTA_TIME,
    output_limits=(MIN_THRUST, MAX_THRUST))
currentHeight = getDroneZPosition(client)
thrust = hoverPid(currentHeight) # Set initial thrust value

# Fly up and hover
print("Starting at z=%.3f" % currentHeight)
while (time.time() - startTime < 20): # Run control loop for 20 seconds
    # Apply thrust
    client.moveByRollPitchYawThrottleAsync(0, 0, 0, thrust, DELTA_TIME).join()

    # Calculate next thrust value from the PID based on new position
    currentHeight = getDroneZPosition(client)
    thrust = hoverPid(currentHeight)

    # No need to sleep here in simulation, since the move command takes time
    #  to execute, but on the drone I think we'd want to sleep here
    # time.sleep(DELTA_TIME)

currentHeight = getDroneZPosition(client)
print("Hovering at z=%.3f" % currentHeight)

# Landing sequence, when starting from hover
# Just apply low thrust (not quite enough to overcome gravity)
#   until the drone reaches the ground
while (currentHeight < GROUND_Z_VAL):
    # TODO: For the real drone, loop condition would probably be currentHeight > GROUND_Z_VAL
    client.moveByRollPitchYawThrottleAsync(0, 0, 0, MIN_THRUST, DELTA_TIME).join()
    currentHeight = getDroneZPosition(client)
    # time.sleep(DELTA_TIME)

print("Landed at z=%.3f" % currentHeight)

# Wait for a short time, then clean up simulator
time.sleep(5)
print("Cleaning up and resetting simulator...")
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
