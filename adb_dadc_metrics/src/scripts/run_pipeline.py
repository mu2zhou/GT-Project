#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import argparse
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in_csv",  type=Path, default=Path("adb_dadc_metrics/src/data/part3_prepared.csv"))
    ap.add_argument("--out_dir", type=Path, default=Path("adb_dadc_metrics/outputs"))
    ap.add_argument("--schema",  type=Path, default=Path("adb_dadc_metrics/src/configs/schema_config.json"))
    args = ap.parse_args()

    # 当以“脚本路径直跑”时，确保包导入可用：把工程父目录加入 sys.path
    project_root   = Path(__file__).resolve().parents[2]   # .../adb_dadc_metrics
    project_parent = project_root.parent                   # 工程父目录
    sys.path.insert(0, str(project_parent))

    # 用“包导入”加载 pipeline（此时相对导入的上下文是完整的）
    from adb_dadc_metrics.src.metrics.pipeline import run_pipeline

    outs = run_pipeline(args.in_csv, args.out_dir, args.schema)
    print("Pipeline finished. Outputs:")
    for k, v in outs.items():
        print(f"{k} => {v}")


if __name__ == "__main__":
    main()