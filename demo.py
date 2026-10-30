"""Run: python demo.py  -> replays the 'crossing paths' and 'geofence' scenarios."""
from airspace.geofence import Geofence
from airspace.manager import AirspaceManager
from airspace.tracks import TrackStore

store = TrackStore()
fence = Geofence("school", [(200, -50), (300, -50), (300, 50), (200, 50)], ceiling_m=150)
mgr = AirspaceManager(store, [fence])

# Two drones on crossing paths at the same altitude, plus one heading for the fence.
store.update("UAS-1", 0, (-300, 0, 60), (10, 0, 0))
store.update("UAS-2", 0, (0, -300, 62), (0, 10, 0))
store.update("UAS-3", 0, (100, 0, 40), (6, 0, 0))

for alert in mgr.step(now=0):
    print(alert)
mgr.acknowledge(("conflict", "UAS-1", "UAS-2"), operator="taki")
print("after operator ack:", mgr.step(now=1))
