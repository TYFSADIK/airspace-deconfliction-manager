# Mini Airspace Deconfliction Manager

Private research repo. A small-scale UAS airspace manager: ESP32 tracker beacons, ArduPilot SITL drones, dynamic geofences, predicted-conflict alerts, and human override.

Full write-up: [`docs/project-page.md`](docs/project-page.md)

## What's here

| File | Purpose |
|---|---|
| `airspace/geo.py` | lat/lon to local ENU metres |
| `airspace/tracks.py` | Track store with velocity estimate, dead-reckoning, and live/coasting/lost quality |
| `airspace/conflict.py` | Closest-point-of-approach conflict prediction with horizontal and vertical separation |
| `airspace/geofence.py` | Keep-out and keep-in polygons with altitude bands and look-ahead breach prediction |
| `airspace/manager.py` | Raises alerts only. Humans acknowledge or override, and every action is logged to JSONL |

## Quickstart

```bash
python demo.py
python -m unittest -v
```

## Next steps
- [ ] SITL feed: `sim_vehicle.py -v ArduCopter -I0` / `-I1`, read `GLOBAL_POSITION_INT` with pymavlink, convert with `geo.to_enu`, call `store.update`
- [ ] ESP32 beacon: GPS plus LoRa packets `{id, t, lat, lon, alt}` sent to a gateway on the Pi
- [ ] Web map (Leaflet) showing tracks, fences, and alerts, with an ack button that calls `acknowledge`
- [ ] Reproduce runs 1–5 from the write-up as unit tests (crossing, altitude convergence, live fence edit, quiet-sky soak, beacon drop-out)
