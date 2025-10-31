
import unittest
from pathlib import Path
import pandas as pd
from adb_dadc_metrics.src.metrics.pipeline import run_pipeline

class TestPipeline(unittest.TestCase):
    def test_run_pipeline_end_to_end(self):
        in_csv = Path('adb_dadc_metrics/src/data/part3_prepared.csv')
        out_dir = Path('adb_dadc_metrics/outputs')
        schema = Path('adb_dadc_metrics/src/configs/schema_config.json')
        outs = run_pipeline(in_csv, out_dir, schema)
        # artifacts exist
        for k in ['per_sample_metrics_csv','summary_by_bin_csv','scalar_metrics_txt','ooca_per_class_csv']:
            self.assertTrue(Path(outs[k]).exists())
        # check summary has expected columns
        df = pd.read_csv(outs['summary_by_bin_csv'])
        for c in ['distance_bin_m','ape_deg','ase_deg_per_s','dee_m','samples']:
            self.assertIn(c, df.columns)

if __name__ == '__main__':
    unittest.main()
