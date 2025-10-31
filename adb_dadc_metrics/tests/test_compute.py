
import unittest
import pandas as pd
from adb_dadc_metrics.src.metrics.schema import ColumnMap
from adb_dadc_metrics.src.metrics.compute import (
    compute_angular_position_error, compute_angular_speed_error, compute_distance_error,
    compute_orientation_accuracy, compute_id_change_rate, ooca_per_class)

class TestCompute(unittest.TestCase):
    def setUp(self):
        # small synthetic dataset (2 samples)
        self.df = pd.DataFrame({
            'timestamp':[0.0, 1.0],
            'distance_gt_m':[10.0, 11.0],
            'distance_pred_m':[10.0, 11.0],
            'angle_left_gt_deg':[1.0, 2.0],
            'angle_left_pred_deg':[1.0, 2.0],
            'angle_right_gt_deg':[1.5, 2.5],
            'angle_right_pred_deg':[1.5, 2.5],
            'angle_top_gt_deg':[0.8, 1.8],
            'angle_top_pred_deg':[0.8, 1.8],
            'angle_bottom_gt_deg':[0.6, 1.6],
            'angle_bottom_pred_deg':[0.6, 1.6],
            'ang_speed_left_gt_dps':[0.0, 1.0],
            'ang_speed_left_pred_dps':[0.0, 1.0],
            'ang_speed_right_gt_dps':[0.0, 1.0],
            'ang_speed_right_pred_dps':[0.0, 1.0],
            'ang_speed_top_gt_dps':[0.0, 1.0],
            'ang_speed_top_pred_dps':[0.0, 1.0],
            'ang_speed_bottom_gt_dps':[0.0, 1.0],
            'ang_speed_bottom_pred_dps':[0.0, 1.0],
            'orientation_gt':['oncoming','preceding'],
            'orientation_pred':['oncoming','preceding'],
            'object_id_pred':[1,1],
            'object_track_id_gt':[1,1],
            'object_type':['vehicle','vehicle']
        })
        self.cm = ColumnMap({
            'timestamp':'timestamp',
            'distance_gt':'distance_gt_m',
            'distance_pred':'distance_pred_m',
            'angle_left_gt':'angle_left_gt_deg',
            'angle_left_pred':'angle_left_pred_deg',
            'angle_right_gt':'angle_right_gt_deg',
            'angle_right_pred':'angle_right_pred_deg',
            'angle_top_gt':'angle_top_gt_deg',
            'angle_top_pred':'angle_top_pred_deg',
            'angle_bottom_gt':'angle_bottom_gt_deg',
            'angle_bottom_pred':'angle_bottom_pred_deg',
            'ang_speed_left_gt':'ang_speed_left_gt_dps',
            'ang_speed_left_pred':'ang_speed_left_pred_dps',
            'ang_speed_right_gt':'ang_speed_right_gt_dps',
            'ang_speed_right_pred':'ang_speed_right_pred_dps',
            'ang_speed_top_gt':'ang_speed_top_gt_dps',
            'ang_speed_top_pred':'ang_speed_top_pred_dps',
            'ang_speed_bottom_gt':'ang_speed_bottom_gt_dps',
            'ang_speed_bottom_pred':'ang_speed_bottom_pred_dps',
            'orientation_gt':'orientation_gt',
            'orientation_pred':'orientation_pred',
            'object_id_pred':'object_id_pred',
            'object_track_id_gt':'object_track_id_gt',
            'object_type':'object_type'
        })

    def test_ape_zero_when_pred_eq_gt(self):
        ape = compute_angular_position_error(self.df, self.cm)
        self.assertTrue((ape == 0).all())

    def test_ase_zero_when_pred_eq_gt(self):
        ase = compute_angular_speed_error(self.df, self.cm)
        self.assertTrue((ase == 0).all())

    def test_dee_zero_when_pred_eq_gt(self):
        dee = compute_distance_error(self.df, self.cm)
        self.assertTrue((dee == 0).all())

    def test_ooca_accuracy_100(self):
        acc = compute_orientation_accuracy(self.df, self.cm)
        self.assertEqual(round(acc,1), 100.0)

    def test_idcr_zero_with_constant_id(self):
        idcr = compute_id_change_rate(self.df, self.cm)
        self.assertEqual(round(idcr,1), 0.0)

    def test_ooca_per_class_with_fake_motorcycle(self):
        res = ooca_per_class(self.df, self.cm, duplicate_fake_motorcycle=True)
        self.assertSetEqual(set(res['object_type']), {'vehicle','motorcycle'})
        self.assertTrue((res['accuracy_%'] == 100.0).all())

if __name__ == '__main__':
    unittest.main()
