#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pandas as pd
from typing import Optional, Dict
from .schema import ColumnMap

ROUNDING = {'ape': 3, 'ase': 1, 'dee': 1, 'ooca': 1, 'idcr': 1}

def check_columns(df: pd.DataFrame, cm: ColumnMap, keys):
    missing = []
    for k in keys:
        col = cm.get(k)
        if col is None or col not in df.columns:
            missing.append(k)
    return missing

def compute_angular_position_error(df: pd.DataFrame, cm: ColumnMap) -> pd.Series:
    req = ['angle_left_gt','angle_left_pred','angle_right_gt','angle_right_pred','angle_top_gt','angle_top_pred','angle_bottom_gt','angle_bottom_pred']
    missing = check_columns(df, cm, req)
    if missing:
        raise ValueError(f"Missing columns for APE: {missing}")
    edges = ['left','right','top','bottom']
    errs = []
    for e in edges:
        gt = df[cm.get(f'angle_{e}_gt')].astype(float)
        pred = df[cm.get(f'angle_{e}_pred')].astype(float)
        errs.append((pred - gt).abs())
    return pd.concat(errs, axis=1).mean(axis=1)

def compute_angular_speed_error(df: pd.DataFrame, cm: ColumnMap) -> pd.Series:
    req = ['ang_speed_left_gt','ang_speed_left_pred','ang_speed_right_gt','ang_speed_right_pred','ang_speed_top_gt','ang_speed_top_pred','ang_speed_bottom_gt','ang_speed_bottom_pred']
    missing = check_columns(df, cm, req)
    if missing:
        raise ValueError(f"Missing columns for ASE: {missing}")
    edges = ['left','right','top','bottom']
    errs = []
    for e in edges:
        gt = df[cm.get(f'ang_speed_{e}_gt')].astype(float)
        pred = df[cm.get(f'ang_speed_{e}_pred')].astype(float)
        errs.append((pred - gt).abs())
    return pd.concat(errs, axis=1).mean(axis=1)

def compute_distance_error(df: pd.DataFrame, cm: ColumnMap) -> pd.Series:
    req = ['distance_gt','distance_pred']
    missing = check_columns(df, cm, req)
    if missing:
        raise ValueError(f"Missing columns for DEE: {missing}")
    gt = df[cm.get('distance_gt')].astype(float)
    pred = df[cm.get('distance_pred')].astype(float)
    return (pred - gt).abs()

def compute_orientation_accuracy(df: pd.DataFrame, cm: ColumnMap) -> Optional[float]:
    req = ['orientation_gt','orientation_pred']
    missing = check_columns(df, cm, req)
    if missing:
        return None
    gt = df[cm.get('orientation_gt')].astype(str)
    pred = df[cm.get('orientation_pred')].astype(str)
    return float((gt == pred).mean() * 100.0)

def compute_id_change_rate(df: pd.DataFrame, cm: ColumnMap) -> Optional[float]:
    req = ['timestamp','object_track_id_gt','object_id_pred']
    missing = check_columns(df, cm, req)
    if missing:
        return None
    ts = df[cm.get('timestamp')].astype(float)
    duration_s = ts.max() - ts.min()
    if duration_s <= 0:
        return None
    changes = 0
    for _, g in df.sort_values(cm.get('timestamp')).groupby(cm.get('object_track_id_gt')):
        ids = g[cm.get('object_id_pred')].astype(str).tolist()
        prev = None
        for pid in ids:
            if prev is None:
                prev = pid
                continue
            if pid != prev:
                changes += 1
            prev = pid
    return float(changes / duration_s)

def distance_bin(distance_m: float, bin_size: int = 100) -> int:
    if pd.isna(distance_m) or distance_m < 0:
        return -1
    return int(distance_m // bin_size) * bin_size

def summarize_by_distance_bins(df: pd.DataFrame, cm: ColumnMap, metrics: Dict[str, pd.Series], bin_size: int = 100) -> pd.DataFrame:
    bins = df[cm.get('distance_gt')].apply(lambda d: distance_bin(d, bin_size))
    out = df[[cm.get('distance_gt')]].copy()
    out['distance_bin_m'] = bins
    for name, s in metrics.items():
        out[name] = s
    grouped = out.groupby('distance_bin_m').agg({**{name:'mean' for name in metrics.keys()}, cm.get('distance_gt'):'count'})
    grouped = grouped.rename(columns={cm.get('distance_gt'):'samples'}).reset_index().sort_values('distance_bin_m')
    return grouped

def ooca_per_class(df: pd.DataFrame, cm: ColumnMap, duplicate_fake_motorcycle: bool = True) -> pd.DataFrame:
    if cm.get('object_type') not in df.columns:
        df = df.copy()
        df['object_type'] = 'vehicle'
    cat = df.copy()
    if duplicate_fake_motorcycle:
        moto = df.copy(); moto['object_type'] = 'motorcycle'; cat = pd.concat([cat, moto], ignore_index=True)
    acc = (cat.assign(match=cat[cm.get('orientation_gt')] == cat.get(cm.get('orientation_pred')))
             .groupby('object_type')['match'].mean().reset_index())
    acc['accuracy_%'] = (acc['match'] * 100.0).round(1)
    return acc.drop(columns=['match'])
