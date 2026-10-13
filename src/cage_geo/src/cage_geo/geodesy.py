"""WGS84/ECEF/local-ENU conversions used by all mission planning code."""
import math
from dataclasses import dataclass

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class CageFrame:
    center_east: float
    center_north: float
    length: float
    unit_east: float
    unit_north: float
    normal_east: float
    normal_north: float


def _ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return ((n + alt_m) * cos_lat * cos_lon,
            (n + alt_m) * cos_lat * sin_lon,
            (n * (1.0 - WGS84_E2) + alt_m) * sin_lat)


def _geodetic(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        alt = p / max(math.cos(lat), 1e-15) - n
        next_lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)))
        if abs(next_lat - lat) < 1e-13:
            lat = next_lat
            break
        lat = next_lat
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / max(math.cos(lat), 1e-15) - n
    return math.degrees(lat), math.degrees(lon), alt


class LocalENU:
    """Local tangent plane whose origin is the UAV takeoff position."""
    def __init__(self, home_lat, home_lon, home_alt=0.0):
        if not (-90.0 <= home_lat <= 90.0 and -180.0 <= home_lon <= 180.0):
            raise ValueError("invalid WGS84 home")
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt = home_alt
        self._origin = _ecef(home_lat, home_lon, home_alt)
        lat, lon = math.radians(home_lat), math.radians(home_lon)
        self._slat, self._clat = math.sin(lat), math.cos(lat)
        self._slon, self._clon = math.sin(lon), math.cos(lon)

    def to_enu(self, lat, lon, alt=None):
        x, y, z = _ecef(lat, lon, self.home_alt if alt is None else alt)
        dx, dy, dz = x-self._origin[0], y-self._origin[1], z-self._origin[2]
        east = -self._slon*dx + self._clon*dy
        north = -self._slat*self._clon*dx - self._slat*self._slon*dy + self._clat*dz
        up = self._clat*self._clon*dx + self._clat*self._slon*dy + self._slat*dz
        return east, north, up

    def to_wgs84(self, east, north, up=0.0):
        dx = -self._slon*east - self._slat*self._clon*north + self._clat*self._clon*up
        dy = self._clon*east - self._slat*self._slon*north + self._clat*self._slon*up
        dz = self._clat*north + self._slat*up
        return _geodetic(self._origin[0]+dx, self._origin[1]+dy, self._origin[2]+dz)


def cage_frame(a_enu, b_enu, minimum_length=0.5):
    dx, dy = b_enu[0]-a_enu[0], b_enu[1]-a_enu[1]
    length = math.hypot(dx, dy)
    if length < minimum_length:
        raise ValueError("cage endpoints are coincident or too close")
    ux, uy = dx/length, dy/length
    return CageFrame((a_enu[0]+b_enu[0])/2.0, (a_enu[1]+b_enu[1])/2.0,
                     length, ux, uy, -uy, ux)
