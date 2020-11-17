import airsim
import time

# Connect to simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Flight sequence
# client.moveToPositionAsync(0, 0, -1, 1).join() # Takeoff
client.takeoffAsync().join()

z = -1 # base height
print("Flying on path...")
client.moveOnPathAsync([
    airsim.Vector3r(5, 0, z), # absolute position vectors
    airsim.Vector3r(5, 5, z),
    airsim.Vector3r(5, 5, z - 5),
    airsim.Vector3r(5, 0, z - 5),
    airsim.Vector3r(0, 0, z - 5),
    airsim.Vector3r(0, 5, z - 5),
    airsim.Vector3r(0, 5, z),
    airsim.Vector3r(0, 0, z)
], 4).join()

# Correct overshoot at end of flight path
client.moveOnPathAsync([
    airsim.Vector3r(0, 0, z)
], 3).join()

print("Landing...")
client.landAsync().join() # Land

# Try using moveByRollPitchYawrateThrottleAsync
# This seems to be what the Crazyflie API exposes?
# client.moveByRollPitchYawThrottleAsync(1, 0, 0, 1, 1).join()
# client.moveByRollPitchYawThrottleAsync(0, 1, 0, 1, 1).join()

# Clean up
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
