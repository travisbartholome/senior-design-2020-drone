# Dependency imports
import airsim
from simple_pid import PID
import matplotlib.pyplot as plt
import numpy as np

# Standard imports
import time
import csv

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

def writeCSVFile(filePath, csvData):
    with open(filePath, 'w', newline='') as csvfile:
        csvWriter = csv.writer(csvfile)
        csvWriter.writerows(csvData)

# ~~~~~~~~~~~~~~~~~~~~

# Connect to simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Define system parameters
GROUND_Z_VAL = getDroneZPosition(client) # Starting height is considered the "ground"
Z_HOVER = -150 # Target hover height
MIN_THRUST = 0.53 # Min thrust to overcome gravity, minus a little # TODO: calibrate?
MAX_THRUST = 1.0
# Note: min thrust to overcome gravity: 0.58 in simulation
# Note: max thrust supported in airsim: 1
DELTA_TIME = 0.01 # Note: Crazyflie docs suggest tick rate of 100Hz (DELTA_TIME = 0.01)

globalTR = []
globalTData = []
globalZData = []
globalYData = []

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
    while (time.time() - startTime < 30): # Run hover loop for specified number of seconds
        client.moveByRollPitchYawThrottleAsync(0, 0, 0, thrust, DELTA_TIME).join()
        currentHeight = getDroneZPosition(client)
        thrust = hoverPid(currentHeight)
    currentHeight = getDroneZPosition(client)
    print("Hovering at z=%.3f" % currentHeight)

    # ~~~~~~~~~~~~~~~~~~~~

    # Run flight sequence
    ROT_SPEED = 1 # Rotation speed in rad/s # TODO: match to paper's assumptions?
    # Lists for data collection
    tData = []
    zData = []
    yData = []
    print("Starting flight sequence")
    currentTime = 0
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
        client.simPause(False)
        client.moveByRollPitchYawThrottleAsync(roll, 0, 0, thrust, DELTA_TIME).join()
        client.simPause(True)

        # Update simulator time
        currentTime += DELTA_TIME
    
    # Save data for run in global data matrices
    globalTData.append(tData)
    globalYData.append(yData)
    globalZData.append(zData)
    globalTR.append(t_r)

    # Reset simulator
    print("Resetting simulator...")
    client.simPause(False)
    client.reset()
    time.sleep(2)

# Run a series of simulations
print("Running multiple-simulation series...")
t_tot = 5
t_t = 0
for x in range(0, int(5 / 0.2) + 1):
    t_r = x * 0.2
    runSimulation(t_t, t_r, t_tot)

# Save data to CSV files
with open('./local-figures/t_r_data.csv', 'w', newline='') as csvfile:
    csvWriter = csv.writer(csvfile)
    csvWriter.writerow(globalTR)
writeCSVFile('./local-figures/t_data.csv', globalTData)
writeCSVFile('./local-figures/y_data.csv', globalYData)
writeCSVFile('./local-figures/z_data.csv', globalZData)

# Wait for a short time, then clean up simulator
time.sleep(5)
print("Cleaning up simulator...")
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
