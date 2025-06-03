import math

# Constants
SCALE = 0.38  # 1 pixel = 0.38 meters
DEG_LAT_M = 111_320  # length of 1° latitude in meters
EARTH_CIRCUMFERENCE = 40_075_000  # Earth's equatorial circumference in meters


def to_meters(pixels: float) -> float:
    """Converts pixels to meters using the predefined scale."""
    return pixels * SCALE


def meters_per_degree_lon(lat: float) -> float:
    """
    Calculates the number of meters per 1° longitude at the given latitude.

    :param lat: Latitude in degrees
    :return: Meters per 1° of longitude at the specified latitude
    """
    return EARTH_CIRCUMFERENCE * math.cos(math.radians(lat)) / 360


def compute_displacement(dx_px: float, dy_px: float, azimuth_deg: float) -> tuple[float, float]:
    """
    Calculates the northward and eastward displacement in meters based on pixel differences and azimuth.

    :param dx_px: X-axis difference in pixels
    :param dy_px: Y-axis difference in pixels
    :param azimuth_deg: Azimuth of drone flight in degrees (clockwise from North)
    :return: Tuple (delta_N, delta_E) — displacement to the North and East in meters
    """
    dx_m = to_meters(dx_px)
    dy_m = to_meters(dy_px)
    theta = math.radians(azimuth_deg)

    delta_N = dx_m * math.sin(theta) + dy_m * math.cos(theta)
    delta_E = dx_m * math.cos(theta) - dy_m * math.sin(theta)

    return delta_N, delta_E


def compute_image_center_coords(
    lat_cp: float,
    lon_cp: float,
    x_cp: int,
    y_cp: int,
    x_center: int,
    y_center: int,
    azimuth: float
) -> tuple[float, float]:
    """
    Computes the geographic coordinates of the image center based on a known control point.

    :param lat_cp: Latitude of the control point
    :param lon_cp: Longitude of the control point
    :param x_cp: X pixel coordinate of the control point
    :param y_cp: Y pixel coordinate of the control point
    :param x_center: X pixel coordinate of the image center
    :param y_center: Y pixel coordinate of the image center
    :param azimuth: Azimuth of the drone flight (in degrees)
    :return: Tuple (lat_center, lon_center) — geographic coordinates of the image center
    """
    dx = x_center - x_cp
    dy = y_center - y_cp

    delta_N, delta_E = compute_displacement(dx, dy, azimuth)

    delta_lat = delta_N / DEG_LAT_M
    delta_lon = delta_E / meters_per_degree_lon(lat_cp)

    lat_center = lat_cp + delta_lat
    lon_center = lon_cp + delta_lon

    return lat_center, lon_center


if __name__ == '__main__':
    # Geographic coordinates of the control point
    lat_cp = 50.603694
    lon_cp = 30.650625

    # Pixel coordinates of the control point
    x_cp = 558
    y_cp = 328

    # Pixel coordinates of the image center
    x_center = 320
    y_center = 256

    # Drone flight azimuth
    azimuth = 335

    # Compute image center coordinates
    lat_center, lon_center = compute_image_center_coords(lat_cp, lon_cp, x_cp, y_cp, x_center, y_center, azimuth)

    print("Image center coordinates:")
    print(f"Latitude: {lat_center:.7f}")
    print(f"Longitude: {lon_center:.7f}")
