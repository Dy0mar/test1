from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time


HOME_LOCATION = [50.450739, 30.461242]
DEST_LOCATION = [50.443326, 30.448078]
ALTITUDE = 100  # in meters
TARGET_YAW = 350  # in degrees

# 1. Connection to SITL
print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

# 2. Ready GPS
while not vehicle.is_armable:
    print("Waiting for GPS...")
    time.sleep(1)

# 3. Entering GUIDED mode and starting
print("Arming motors...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

# 4. Take off
print(f"Taking off to {ALTITUDE} meters...")
vehicle.simple_takeoff(ALTITUDE)

# Wait until we reach the height.
while True:
    current_alt = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {current_alt:.1f}")
    if current_alt >= ALTITUDE * 0.95:
        print("Target altitude reached")
        break
    time.sleep(1)

# 5. Flying to point B
print("Flying to point B...")
point_b = LocationGlobalRelative(DEST_LOCATION[0], DEST_LOCATION[1], ALTITUDE)
vehicle.simple_goto(point_b)

time.sleep(30)


# 6. Yaw rotation
def condition_yaw(heading, relative=False):
    """Повертає дрон на заданий yaw."""
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0,
        0,  # target system, component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,  # confirmation
        heading,    # yaw in degrees
        0,          # yaw speed deg/s (0 = auto)
        1,          # direction -1 ccw, 1 cw
        is_relative,
        0, 0, 0     # unused
    )
    vehicle.send_mavlink(msg)

print("Rotating to yaw 350°...")
condition_yaw(TARGET_YAW)
time.sleep(5)

print("Mission complete. Holding position.")
