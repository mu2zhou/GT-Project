#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
from pathlib import Path
import numpy as np
import pandas as pd

def prepare(in_csv: Path, out_csv: Path, width_m: float = 2.0, height_m: float = 1.5) -> None:
    with open(in_csv, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.read().splitlines()
    header_idx = None
    for i, line in enumerate(lines[:200]):
        if line.strip().startswith('Time,'):
            header_idx = i; break
    if header_idx is None:
        raise RuntimeError("Could not locate header line starting with 'Time,'")
    df = pd.read_csv(in_csv, header=header_idx)
    for c in ['Time','E1','N1','E2','N2']:
        if c not in df.columns:
            raise RuntimeError(f'Missing required column: {c}')
    dE = df['E2'].astype(float) - df['E1'].astype(float)
    dN = df['N2'].astype(float) - df['N1'].astype(float)
    range_m = np.sqrt(dE**2 + dN**2)
    angle_center = np.degrees(np.arctan2(dE, dN))
    half_w = np.degrees(np.arctan2(width_m/2.0, range_m.clip(lower=0.1)))
    half_h = np.degrees(np.arctan2(height_m/2.0, range_m.clip(lower=0.1)))
    ang_left = angle_center - half_w
    ang_right = angle_center + half_w
    ang_top = angle_center + half_h
    ang_bottom = angle_center - half_h
    t = df['Time'].astype(float)
    ang_speed_left = np.gradient(ang_left, t)
    ang_speed_right = np.gradient(ang_right, t)
    ang_speed_top = np.gradient(ang_top, t)
    ang_speed_bottom = np.gradient(ang_bottom, t)
    drdt = np.gradient(range_m, t)
    orientation = np.where(drdt < 0, 'oncoming', 'preceding')
    out = pd.DataFrame({
        'timestamp': t,
        'distance_gt_m': range_m,
        'distance_pred_m': range_m,
        'angle_left_gt_deg': ang_left,
        'angle_left_pred_deg': ang_left,
        'angle_right_gt_deg': ang_right,
        'angle_right_pred_deg': ang_right,
        'angle_top_gt_deg': ang_top,
        'angle_top_pred_deg': ang_top,
        'angle_bottom_gt_deg': ang_bottom,
        'angle_bottom_pred_deg': ang_bottom,
        'ang_speed_left_gt_dps': ang_speed_left,
        'ang_speed_left_pred_dps': ang_speed_left,
        'ang_speed_right_gt_dps': ang_speed_right,
        'ang_speed_right_pred_dps': ang_speed_right,
        'ang_speed_top_gt_dps': ang_speed_top,
        'ang_speed_top_pred_dps': ang_speed_top,
        'ang_speed_bottom_gt_dps': ang_speed_bottom,
        'ang_speed_bottom_pred_dps': ang_speed_bottom,
        'orientation_gt': orientation,
        'orientation_pred': orientation,
        'object_id_pred': 1,
        'object_track_id_gt': 1,
        'object_type': 'vehicle'
    })
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(out_csv, index=False)
    print('Prepared CSV saved to', out_csv)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--in_csv', type=Path, default=Path('part3.csv'))
    ap.add_argument('--out_csv', type=Path, default=Path('adb_dadc_metrics_final/src/data/part3_prepared.csv'))
    ap.add_argument('--width_m', type=float, default=2.0)
    ap.add_argument('--height_m', type=float, default=1.5)
    args = ap.parse_args()
    prepare(args.in_csv, args.out_csv, args.width_m, args.height_m)
