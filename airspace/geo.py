"""Small-area geodesy: lat/lon <-> local east/north metres (flat-earth, fine for < 10 km)."""
import math

EARTH_R = 6371000.0


def to_enu(lat, lon, ref_lat, ref_lon):
    e = math.radians(lon - ref_lon) * EARTH_R * math.cos(math.radians(ref_lat))
    n = math.radians(lat - ref_lat) * EARTH_R
    return e, n


def from_enu(e, n, ref_lat, ref_lon):
    lat = ref_lat + math.degrees(n / EARTH_R)
    lon = ref_lon + math.degrees(e / (EARTH_R * math.cos(math.radians(ref_lat))))
    return lat, lon
