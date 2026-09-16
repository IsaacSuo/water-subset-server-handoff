import unittest
import numpy as np

from world_model_dataset.proportional_asset_batch import pad_layout


class ProportionalLayoutTests(unittest.TestCase):
    def setUp(self):
        self.settings = dict(pad_thickness_fraction=.1, pad_width_fraction=1.05,
                             pad_height_fraction=.45, approach_gap_fraction=.2,
                             ground_clearance_m=.003)

    def test_thin_banana_does_not_inherit_half_metre_wide_pad(self):
        bounds = [[-.14, -.03, -.05], [.16, .03, .125]]
        size, center = pad_layout(bounds, 'X', [0,0,.0505], 0., self.settings)
        np.testing.assert_allclose(size, [.03,.063,.07875])
        self.assertAlmostEqual(center[0]+size[0]/2, -.20)
        self.assertAlmostEqual(center[2]-size[2]/2, .003)

    def test_y_axis_uses_transverse_x_extent_and_original_floor(self):
        bounds = [[-.15,-.1,-.13], [.20,.11,.14]]
        size, center = pad_layout(bounds, 'Y', [-.4,1.1,.13], -.001, self.settings)
        np.testing.assert_allclose(size,[.3675,.035,.1215])
        self.assertAlmostEqual(center[1]+size[1]/2, .93)
        self.assertAlmostEqual(center[2]-size[2]/2, .002)
        self.assertAlmostEqual(center[0], -.375)

    def test_dimensions_scale_with_posed_bounds(self):
        bounds = np.array([[-.1,-.04,-.08],[.1,.04,.12]])
        a,_=pad_layout(bounds,'X',[0,0,0],0.,self.settings)
        b,_=pad_layout(bounds*2,'X',[0,0,0],0.,self.settings)
        np.testing.assert_allclose(b,2*a)


if __name__ == '__main__': unittest.main()
