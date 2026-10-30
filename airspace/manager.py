"""Alert manager: raises alerts, never acts on its own. A human acknowledges or overrides."""
import json
import time

from .conflict import find_conflicts
from . import geofence


class AirspaceManager:
    def __init__(self, store, fences=None, log_path=None):
        self.store = store
        self.fences = fences or []
        self.acked: set[tuple] = set()
        self.log_path = log_path

    def _log(self, event):
        event["wall"] = time.time()
        if self.log_path:
            with open(self.log_path, "a") as fh:
                fh.write(json.dumps(event) + "\n")

    def step(self, now):
        alerts = []
        active = self.store.active(now)
        for c in find_conflicts(active, now):
            key = ("conflict", c.a, c.b)
            if key in self.acked:
                continue
            degraded = any(self.store.quality(x, now) != "live" for x in (c.a, c.b))
            alerts.append({"type": "conflict", "a": c.a, "b": c.b, "severity": c.severity,
                           "t_cpa": round(c.t_cpa, 1), "h_m": round(c.h_dist, 1),
                           "v_m": round(c.v_dist, 1), "degraded_track": degraded})
        for tr in active:
            for f, dt in geofence.check(self.fences, tr, now):
                key = ("geofence", tr.id, f.name)
                if key not in self.acked:
                    alerts.append({"type": "geofence", "track": tr.id, "fence": f.name, "in_s": dt})
        for a in alerts:
            self._log({"now": now, **a})
        return alerts

    def acknowledge(self, key: tuple, operator: str):
        """Human override: silence an alert. Logged so it can be reviewed later."""
        self.acked.add(key)
        self._log({"type": "ack", "key": list(key), "operator": operator})
