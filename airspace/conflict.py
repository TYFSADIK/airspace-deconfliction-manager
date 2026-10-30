"""Predicted-conflict detection with closest point of approach (CPA).

Two tracks are in conflict if, within the look-ahead horizon, their horizontal
separation drops below `h_sep` AND vertical separation below `v_sep` at the
same time.
"""
from dataclasses import dataclass
import math

from .tracks import Track


@dataclass
class Conflict:
    a: str
    b: str
    t_cpa: float        # seconds from now until closest approach
    h_dist: float       # horizontal distance at CPA (m)
    v_dist: float       # vertical distance at CPA (m)
    severity: str       # advisory | warning | alert


def cpa(a: Track, b: Track, now: float, horizon: float):
    pa, pb = a.predict(now), b.predict(now)
    dp = [pb[i] - pa[i] for i in range(3)]
    dv = [b.vel[i] - a.vel[i] for i in range(3)]
    dv2 = dv[0] ** 2 + dv[1] ** 2
    # horizontal CPA time; clamp to [0, horizon]
    t = 0.0 if dv2 < 1e-9 else -(dp[0] * dv[0] + dp[1] * dv[1]) / dv2
    t = max(0.0, min(horizon, t))
    rel = [dp[i] + dv[i] * t for i in range(3)]
    return t, math.hypot(rel[0], rel[1]), abs(rel[2])


def find_conflicts(tracks, now, horizon=60.0, h_sep=50.0, v_sep=15.0):
    out = []
    for i in range(len(tracks)):
        for j in range(i + 1, len(tracks)):
            t, h, v = cpa(tracks[i], tracks[j], now, horizon)
            if h < h_sep and v < v_sep:
                sev = "alert" if t < 15 else "warning" if t < 30 else "advisory"
                out.append(Conflict(tracks[i].id, tracks[j].id, t, h, v, sev))
    return sorted(out, key=lambda c: c.t_cpa)
