# Dependency imports
import airsim
from simple_pid import PID
import matplotlib.pyplot as plt
import numpy as np

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

def getDroneYPosition(multirotorClient):
    """
    Get the horizontal position of the drone.
    This is similar to getDroneZPosition in simulation.

    Args:
        multirotorClient (airsim.MultirotorClient): The airsim client object

    Returns:
        yPosition (float): The y position of the drone associated with this client
    """
    return multirotorClient.getMultirotorState().kinematics_estimated.position.y_val

# ~~~~~~~~~~~~~~~~~~~~

# Connect to simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Define system parameters
GROUND_Z_VAL = getDroneZPosition(client) # Starting height is considered the "ground"
Z_HOVER = -5 # Target hover height
MIN_THRUST = 0.53 # Min thrust to overcome gravity, minus a little # TODO: calibrate?
MAX_THRUST = 1.0
# Note: min thrust to overcome gravity: 0.58 in simulation
# Note: max thrust supported in airsim: 1
DELTA_TIME = 0.01 # Crazyflie docs suggest tick rate of 100Hz

def runSimulation(t_t, t_r, t_tot):
    """
    Reusable method to run a single flight trajectory

    Args:
        t_t (float): Thrust switching time
        t_r (float): Rotation switching time
        t_tot (float): Total flight time
    """
    client.enableApiControl(True)
    client.armDisarm(True)

    startTime = time.time()

    # Lists for data collection
    tData = []
    zData = []

    # Hover to start position - see hover_land.py
    hoverPid = PID(
        Kp=-0.4,
        Ki=-1,
        Kd=-1,
        setpoint=Z_HOVER,
        sample_time=DELTA_TIME,
        output_limits=(MIN_THRUST, MAX_THRUST))
    currentHeight = getDroneZPosition(client)
    thrust = hoverPid(currentHeight) # Set initial thrust value
    print("Starting at z=%.3f" % currentHeight)
    while (time.time() - startTime < 20): # Run control loop for 20 seconds
        tData.append(time.time() - startTime)
        zData.append(currentHeight)
        
        client.moveByRollPitchYawThrottleAsync(0, 0, 0, thrust, DELTA_TIME).join()
        currentHeight = getDroneZPosition(client)
        thrust = hoverPid(currentHeight)
    currentHeight = getDroneZPosition(client)
    print("Hovering at z=%.3f" % currentHeight)

    # ~~~~~~~~~~~~~~~~~~~~

    # Run flight sequence
    ROT_SPEED = 1 # Rotation speed in rad/s # TODO: match to paper's assumptions?
    tData = []
    zData = []
    yData = []
    startTime = time.time()
    print("Starting flight sequence")
    currentTime = time.time() - startTime
    roll = 0 # Start with no roll
    while (currentTime < t_tot):
        # Data capture
        tData.append(currentTime)
        # Use -z so our final results use +z as the up direction
        # Also adjust z to be relative to the starting position
        zData.append(-(getDroneZPosition(client) - Z_HOVER))
        yData.append(getDroneYPosition(client))

        # Set thrust and roll for next time segment
        thrust = MIN_THRUST if (currentTime < t_t) else MAX_THRUST
        roll = roll + ROT_SPEED * DELTA_TIME if (currentTime < t_r) else roll

        # Move
        # Use roll for rotation
        client.moveByRollPitchYawThrottleAsync(roll, 0, 0, thrust, DELTA_TIME).join()

        # Update time
        currentTime = time.time() - startTime
    plt.plot(yData, zData)

    # Reset simulator
    print("Resetting simulator...")
    client.reset()
    time.sleep(2)

# Run a one-shot simulation
plt.figure()
t_t = 0
t_r = 1
t_tot = 5
runSimulation(t_t, t_r, t_tot)
plt.title(f"Flight path with $t_T={t_t}$, $t_R={t_r}$, and $t_{{tot}}={t_tot}$")
plt.xlabel("Horizontal position")
plt.ylabel("Vertical position")
plt.savefig("./local-figures/flight-data.png")

# Run a series of simulations, shown on the same plot
print("Running multiple-simulation series...")
plt.figure()
t_tot = 5
# Tuples (t_t, t_r)
parameterPairs = [
    (0, 1),
    (2, 1), # Note: not an optimal path
    (0, 0),
    (3, 0),
    (0.4, 2),
]
legendStrings = []
for t_t, t_r in parameterPairs:
    runSimulation(t_t, t_r, t_tot)
    legendStrings.append(f"$t_T={t_t}$, $t_R={t_r}$")
plt.title(f"Flight paths with assorted parameters\n$t_{{tot}}={t_tot}$, min. $u_T={MIN_THRUST}$, max. $u_T={MAX_THRUST}$")
plt.xlabel("Horizontal position")
plt.ylabel("Vertical position")
plt.legend(legendStrings)
plt.savefig("./local-figures/flight-data-multiple.png")

# Wait for a short time, then clean up simulator
time.sleep(5)
print("Cleaning up simulator...")
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
