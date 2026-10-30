import unittest

from airspace.conflict import find_conflicts
from airspace.geofence import Geofence, check
from airspace.tracks import TrackStore


class TestAirspace(unittest.TestCase):
    def test_crossing_conflict_predicted(self):
        s = TrackStore()
        a = s.update("a", 0, (-300, 0, 60), (10, 0, 0))
        b = s.update("b", 0, (0, -300, 60), (0, 10, 0))
        c = find_conflicts([a, b], 0)
        self.assertEqual(len(c), 1)
        self.assertAlmostEqual(c[0].t_cpa, 30, delta=1)

    def test_vertical_separation_clears(self):
        s = TrackStore()
        a = s.update("a", 0, (-300, 0, 30), (10, 0, 0))
        b = s.update("b", 0, (0, -300, 90), (0, 10, 0))
        self.assertEqual(find_conflicts([a, b], 0), [])

    def test_geofence_breach_lookahead(self):
        f = Geofence("box", [(0, 0), (10, 0), (10, 10), (0, 10)])
        s = TrackStore()
        tr = s.update("a", 0, (-15, 5, 30), (1, 0, 0))
        self.assertEqual(check([f], tr, 0)[0][1], 20)

    def test_track_quality_degrades(self):
        s = TrackStore()
        s.update("a", 0, (0, 0, 0))
        self.assertEqual(s.quality("a", 1), "live")
        self.assertEqual(s.quality("a", 5), "coasting")
        self.assertEqual(s.quality("a", 20), "lost")


if __name__ == "__main__":
    unittest.main()
