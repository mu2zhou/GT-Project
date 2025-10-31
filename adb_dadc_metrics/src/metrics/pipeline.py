#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import json
import pandas as pd

# 相对导入（在包上下文下工作）
from .schema import ColumnMap
from .compute import (
    compute_angular_position_error,
    compute_angular_speed_error,
    compute_distance_error,
    compute_orientation_accuracy,
    compute_id_change_rate,
    summarize_by_distance_bins,
    ROUNDING,
    ooca_per_class,
)
from .plotting import plot_bars, plot_scatter_with_exp_fit


# 默认 schema 配置路径（工程内路径）
DEFAULT_SCHEMA_PATH = Path("adb_dadc_metrics/src/configs/schema_config.json")


def run_pipeline(csv_path: Path, out_dir: Path, schema_path: Path = DEFAULT_SCHEMA_PATH):
    # 加载列映射
    with open(schema_path, "r", encoding="utf-8") as f:
        mapping = json.load(f)
    cm = ColumnMap(mapping)

    # 读数据
    df = pd.read_csv(csv_path)

    # 计算各项指标
    ape = compute_angular_position_error(df, cm).round(ROUNDING["ape"])
    ase = compute_angular_speed_error(df, cm).round(ROUNDING["ase"])
    dee = compute_distance_error(df, cm).round(ROUNDING["dee"])

    # 可选指标（可能返回 None）
    ooca = compute_orientation_accuracy(df, cm)
    idcr = compute_id_change_rate(df, cm)

    # 每样本表
    per_sample = pd.DataFrame({
        "distance_gt_m": df[cm.get("distance_gt")],
        "ape_deg": ape,
        "ase_deg_per_s": ase,
        "dee_m": dee
    })
    if ooca is not None:
        per_sample["orientation_gt"]   = df[cm.get("orientation_gt")]
        per_sample["orientation_pred"] = df[cm.get("orientation_pred")]
    if idcr is not None:
        per_sample["object_id_pred"]      = df[cm.get("object_id_pred")]
        per_sample["object_track_id_gt"]  = df[cm.get("object_track_id_gt")]

    # 输出目录
    out_dir.mkdir(parents=True, exist_ok=True)

    # 保存每样本结果
    per_sample_path = out_dir / "per_sample_metrics.csv"
    per_sample.to_csv(per_sample_path, index=False)

    # 100m 分箱汇总
    df_bins = summarize_by_distance_bins(df, cm, {
        "ape_deg": ape,
        "ase_deg_per_s": ase,
        "dee_m": dee
    }, bin_size=100)

    summary_path = out_dir / "summary_by_bin.csv"
    df_bins.to_csv(summary_path, index=False)

    # 10% 参考线（柱图）
    ref_ape = 0.1 * df_bins["ape_deg"].max()         if df_bins["ape_deg"].max()         > 0 else 0.1
    ref_ase = 0.1 * df_bins["ase_deg_per_s"].max()   if df_bins["ase_deg_per_s"].max()   > 0 else 0.1
    ref_dee = 0.1 * df_bins["dee_m"].max()           if df_bins["dee_m"].max()           > 0 else 0.1

    # 出图（柱图 + 散点指数拟合）
    ape_bars     = plot_bars(df_bins, "ape_deg",        "Angular Position Error (deg)",    out_dir, ref_ape)
    ase_bars     = plot_bars(df_bins, "ase_deg_per_s",  "Angular Speed Error (deg/s)",     out_dir, ref_ase)
    dee_bars     = plot_bars(df_bins, "dee_m",          "Distance Estimation Error (m)",   out_dir, ref_dee)

    ape_scatter  = plot_scatter_with_exp_fit(df, cm.get("distance_gt"), ape, "ape_deg",       "Angular Position Error (deg)", out_dir)
    ase_scatter  = plot_scatter_with_exp_fit(df, cm.get("distance_gt"), ase, "ase_deg_per_s", "Angular Speed Error (deg/s)",  out_dir)
    dee_scatter  = plot_scatter_with_exp_fit(df, cm.get("distance_gt"), dee, "dee_m",         "Distance Estimation Error (m)",out_dir)

    # OOCA 按类（vehicle + fake motorcycle）
    ooca_pc = ooca_per_class(df, cm, duplicate_fake_motorcycle=True)
    ooca_pc_path = out_dir / "ooca_per_class.csv"
    ooca_pc.to_csv(ooca_pc_path, index=False)

    # 标量写入（稳妥：不用 f-string；先拼再写）
    scalar_lines = [
        "Object Orientation Classification Accuracy (%): {}".format(ooca if ooca is not None else "N/A"),
        "Object ID Change Rate (id/s): {}".format(idcr if idcr is not None else "N/A"),
    ]
    scalar_path = out_dir / "scalar_metrics.txt"
    with open(scalar_path, "w", encoding="utf-8") as f:
        f.write("\n".join(scalar_lines) + "\n")

    return {
        "per_sample_metrics_csv": str(per_sample_path),
        "summary_by_bin_csv": str(summary_path),
        "charts": [
            str(ape_bars), str(ase_bars), str(dee_bars),
            str(ape_scatter), str(ase_scatter), str(dee_scatter)
        ],
        "scalar_metrics_txt": str(scalar_path),
        "ooca_per_class_csv": str(ooca_pc_path)
    }