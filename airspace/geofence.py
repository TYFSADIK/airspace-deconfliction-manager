"""Dynamic geofences: polygons with an altitude band that can be edited live."""
from dataclasses import dataclass


@dataclass
class Geofence:
    name: str
    polygon: list            # [(e, n), ...] metres
    floor_m: float = 0.0
    ceiling_m: float = 120.0  # Canadian basic/advanced ops limit is 122 m AGL
    kind: str = "keep_out"    # keep_out | keep_in
    active: bool = True

    def contains(self, e, n, up) -> bool:
        if not (self.floor_m <= up <= self.ceiling_m):
            return False
        inside = False
        pts = self.polygon
        for i in range(len(pts)):
            (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % len(pts)]
            if (y1 > n) != (y2 > n) and e < (x2 - x1) * (n - y1) / (y2 - y1) + x1:
                inside = not inside
        return inside


def check(fences, track, now, lookahead=(0, 10, 20, 30)):
    """Return (fence, seconds_until_breach) for the first predicted breach of each fence."""
    hits = []
    for f in fences:
        if not f.active:
            continue
        for dt in lookahead:
            e, n, up = track.predict(now + dt)
            breach = f.contains(e, n, up) if f.kind == "keep_out" else not f.contains(e, n, up)
            if breach:
                hits.append((f, dt))
                break
    return hits
