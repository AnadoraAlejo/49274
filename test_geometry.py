import unittest
import numpy as np
from perception3_geometry import depth_point, transform_point, ArtefactTracks
from install_perception3 import patch

class GeometryTests(unittest.TestCase):
    def test_forward_and_right(self):
        image = np.full((480,720), 4.0)
        point = depth_point(image, (350,230,20,20))
        np.testing.assert_allclose(point, [4,0,0], atol=0.02)
        self.assertLess(depth_point(image, (450,230,20,20))[1], 0)
    def test_invalid_depth(self):
        self.assertIsNone(depth_point(np.full((480,720), np.nan), (350,230,20,20)))
        self.assertIsNone(depth_point(np.zeros((480,720)), (350,230,20,20)))
        self.assertIsNone(depth_point(np.ones((480,720)), (800,0,20,20)))
    def test_rotation_translation(self):
        q = [0,0,np.sqrt(0.5),np.sqrt(0.5)]
        np.testing.assert_allclose(transform_point([4,0,0],np.array([1,2,3]),q),[1,6,3],atol=1e-10)
    def test_fusion_and_class_separation(self):
        tracker = ArtefactTracks()
        for stamp, x in enumerate([1.0,1.1,0.9]):
            confirmed = tracker.update([('green_alien',[x,2,3])], stamp)
        self.assertEqual(len(confirmed),1)
        self.assertAlmostEqual(confirmed[0]['point'][0],1)
        tracker.update([('green_crystals',[1,2,3]),('green_alien',[5,2,3]),('negative',[8,2,3])],3)
        self.assertEqual(len(tracker.tracks),3)
    def test_one_vote_per_frame_and_expiry(self):
        tracker = ArtefactTracks()
        tracker.update([('stop_sign',[1,0,0]),('stop_sign',[1.1,0,0])],0)
        self.assertEqual([t['count'] for t in tracker.tracks],[1,1])
        self.assertEqual(tracker.update([],6),[])
        self.assertEqual(len(tracker.tracks),0)

if __name__ == '__main__':
    unittest.main()
