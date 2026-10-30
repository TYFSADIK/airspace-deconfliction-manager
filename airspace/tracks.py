"""Track store: latest state per aircraft, with dead-reckoning when a beacon drops out."""
from dataclasses import dataclass, field


@dataclass
class Track:
    id: str
    t: float                 # seconds
    pos: tuple               # (e, n, up) metres
    vel: tuple               # (ve, vn, vu) m/s
    source: str = "beacon"   # beacon | sitl | manual
    history: list = field(default_factory=list)

    def predict(self, t_future: float) -> tuple:
        dt = t_future - self.t
        return tuple(p + v * dt for p, v in zip(self.pos, self.vel))

    def age(self, now: float) -> float:
        return now - self.t


class TrackStore:
    def __init__(self, stale_after_s=3.0, drop_after_s=15.0):
        self.tracks: dict[str, Track] = {}
        self.stale_after_s = stale_after_s
        self.drop_after_s = drop_after_s

    def update(self, track_id, t, pos, vel=None, source="beacon"):
        old = self.tracks.get(track_id)
        if vel is None and old is not None and t > old.t:
            vel = tuple((p - q) / (t - old.t) for p, q in zip(pos, old.pos))
        tr = Track(track_id, t, tuple(pos), tuple(vel or (0.0, 0.0, 0.0)), source)
        if old:
            tr.history = (old.history + [(old.t, old.pos)])[-200:]
        self.tracks[track_id] = tr
        return tr

    def quality(self, track_id, now) -> str:
        age = self.tracks[track_id].age(now)
        if age <= self.stale_after_s:
            return "live"
        return "coasting" if age <= self.drop_after_s else "lost"

    def active(self, now):
        return [t for t in self.tracks.values() if t.age(now) <= self.drop_after_s]
