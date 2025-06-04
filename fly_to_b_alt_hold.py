import math
import time

from dronekit import connect, VehicleMode


class Joystick:
    Roll = 1
    Pitch = 2
    Throttle = 3
    Yaw = 4


def get_distance_meters(lat1, lon1, lat2, lon2):
    """Haversine formula to calculate distance between coordinates."""
    R = 6371000  # Earth's radius in meters
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(
        d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def get_bearing(lat1, lon1, lat2, lon2):
    """Calculates the azimuth (direction in degrees) from (lat1, lon1) to (lat2, lon2)."""
    d_lon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(d_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


class ConnectionConfig:
    def __init__(self, protocol='tcp', host='127.0.0.1', port=5762):
        self.protocol = protocol
        self.host = host
        self.port = port

    @property
    def connection_url(self):
        return f'{self.protocol}:{self.host}:{self.port}'


class Copter:
    vehicle = None
    __is_first_take_off = True

    def __init__(self, address):
        self.connect(address)
        self.set_armed()
        for rc in [Joystick.Roll, Joystick.Throttle, Joystick.Pitch, Joystick.Yaw]:
            self.vehicle.channels.overrides[rc] = 1500
        time.sleep(1)

    def connect(self, address):
        print(f"Connecting to address {address}...")
        self.vehicle = connect(address, wait_ready=True)
        print("Connected")

    def set_mode(self, mode):
        print("Setting mode to " + mode)
        self.vehicle.mode = VehicleMode(mode)

        if self.vehicle.mode != mode:
            while self.vehicle.mode.name != mode:
                print("Waiting for mode change...")
                time.sleep(1)

        print("The mode is set", self.vehicle.mode)

    def set_armed(self):
        while not self.vehicle.is_armable:
            print("Waiting for GPS...")
            time.sleep(1)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

    def take_off(self, target_altitude):
        self.set_mode("GUIDED")
        print("Taking off...")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {alt:.1f}")
            if alt >= target_altitude * 0.95:
                print("Target altitude reached")
                break
            time.sleep(1)

        self.set_mode("ALT_HOLD")

    def rotate_to_heading(self, target_heading: float, threshold: int = 5, yaw_speed: int = 1600):
        """
        Rotates the copter in the direction `target_heading` (0-359) using the RC YAW channel.
        Works in ALT_HOLD mode.
        """
        assert 0 <= target_heading < 360, "target_heading must be 0-359"

        def get_diff(current, target):
            diff = (target - current + 360) % 360
            return diff if diff <= 180 else diff - 360

        while True:
            current_heading = self.vehicle.heading
            diff = get_diff(current_heading, target_heading)

            print(f"Current heading: {current_heading}, Target: {target_heading}, Diff: {diff}")
            if abs(diff) <= threshold:
                break

            if diff > 0:
                self.vehicle.channels.overrides[Joystick.Yaw] = yaw_speed
            else:
                self.vehicle.channels.overrides[Joystick.Yaw] = 1400

            time.sleep(0.2)

        self.vehicle.channels.overrides[Joystick.Yaw] = 1500

    def disconnect(self):
        if self.vehicle is not None:
            self.vehicle.channels.overrides = {}
            self.vehicle.close()

    def fly_towards(self, target_lat, target_lon, tolerance_m=5.0):
        """
        Turns the copter towards the coordinate and flies towards it.
        """
        current = self.vehicle.location.global_relative_frame
        bearing = get_bearing(current.lat, current.lon, target_lat, target_lon)

        self.rotate_to_heading(bearing)

        print("Let's fly forward....")
        power = 1300
        self.vehicle.channels.overrides[Joystick.Pitch] = power

        prev_hundred = 800
        prev_ten = 40
        while True:
            current = self.vehicle.location.global_relative_frame
            dist = get_distance_meters(current.lat, current.lon, target_lat, target_lon)
            print(f"{current.lat:.6f}, {current.lon:.6f} | distance to target: {dist:.1f}м")

            if 100 < dist < 1000:
                cur_hundred = round(dist, -2)
                if cur_hundred < prev_hundred:
                    bearing = get_bearing(current.lat, current.lon, target_lat, target_lon)
                    self.rotate_to_heading(bearing)
                    print("Adjusted direction (100m step)")
                    prev_hundred = cur_hundred - 100

            elif 10 < dist < 50:
                power = 1450
                cur_ten = round(dist, -1)
                if cur_ten < prev_ten:
                    bearing = get_bearing(current.lat, current.lon, target_lat, target_lon)
                    self.rotate_to_heading(bearing)
                    print("Adjusted direction (10m step)")
                    prev_ten = cur_ten - 10

            if dist > 100 and power > 1000:
                power -= 100

            self.vehicle.channels.overrides[Joystick.Pitch] = power
            if dist < tolerance_m:
                break

            time.sleep(1)

        self.vehicle.channels.overrides[Joystick.Pitch] = 1500
        print("Target point reached!")


if __name__ == '__main__':
    config = ConnectionConfig()
    copter = Copter(address=config.connection_url)

    copter.take_off(target_altitude=100)

    copter.fly_towards(target_lat=50.443326, target_lon=30.448078)
    copter.rotate_to_heading(350)

    copter.take_off(target_altitude=1)

    copter.disconnect()
