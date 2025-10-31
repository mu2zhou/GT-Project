
import unittest
from pathlib import Path
import pandas as pd
from adb_dadc_metrics.src.scripts.prepare_from_part3 import prepare

class TestPrepare(unittest.TestCase):
    def test_prepare_generates_required_columns(self):
        in_csv = Path('part3.csv')
        out_csv = Path('adb_dadc_metrics/src/data/part3_prepared.csv')
        prepare(in_csv, out_csv)
        df = pd.read_csv(out_csv)
        req = ['timestamp','distance_gt_m','distance_pred_m','angle_left_gt_deg','ang_speed_left_gt_dps','orientation_gt','object_id_pred','object_track_id_gt','object_type']
        for c in req:
            self.assertIn(c, df.columns)
        self.assertGreater(len(df), 0)

if __name__ == '__main__':
    unittest.main()
