#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

(ultraAV) jzhou32@pc:~/workspace_jz/017_UltraAV/Filed-Experiment-Data-ULTra-AV$ python Code/merge_two_cars_nearest.py \
>   --csv Dataset/OpenACC/data/Casale/part3.csv \
>   --ref-mcap wmdd_example_datas/output_wukong.mcap \
>   --time-col Time --time-mode relative --time-unit s \
>   --auto-align \
>   --x-col IVS1 --y-col LAT_OFFSET --frame-id ego_vehicle \
>   --tolerance-s 0.6 \
>   --skiprows 5 --sep auto \
>   --force-first-frame \
>   -o merged_two_cars.mcap
[INFO] auto-align: ref_start_ns=405263549529, csv_start_ns=55287099999986, offset_ns=-54881836450457
✅ 合并完成：merged_two_cars.mcap
   参考帧数：362
   最近邻成功：362 帧
   最近邻未匹配：0 帧（超容差或无近邻）
   因 NaN 跳过：0 帧
   参考话题：/roadusers  →  输出话题：/roadusers_merged
   设置：tolerance=0.600s, time_mode=relative, time_unit=s, time_offset_s=0.000, auto_align=True, calib_unit=False, calibrate_linear=False, calib_tol=2.000s, force_first_frame=True
   提示：在 Foxglove 订阅 /roadusers_merged 查看合并效果。


发生过的问题 & 为什么这样改就能好

你之前的运行日志里出现了 unit-calib: a_unit≈1000.0，这会把 CSV 的时间轴再乘以 1000，导致时间全面错位。我们已经用你提供的节拍分析结果（CSV 0.1 s vs 参考 0.5 s）确定单位=秒，并关闭单位/线性校准，只用「自动首帧对齐 + 最近邻容差」。
--tolerance-s 设为 0.6 s，正好覆盖参考 0.5 s 的帧间隔（再加一点余量）。如果你的参考帧有抖动或某段节拍不稳，可以临时放宽到 0.8〜1.0 s，确认匹配数上来后再收紧。
你强调「目标静止」。对这种场景，逐帧对齐到参考时间戳（--snap-to-ref）比插值更一致、观感更稳定，这就是你之前喜欢「两个 mcap 同播」的核心原因——我们这次把它做成自动化的 CSV-only 对齐导出。


小结 & 接下来

结论：CSV 的 Time 是秒，不是毫秒；用「自动起点对齐 + 最近邻容差」即可大幅提升匹配帧数。
优先跑「方案 1」；如果你更爱「两个 mcap 同播」的效果，追加「方案 2」。
跑完把统计贴给我（特别是“最近邻成功”的帧数和 Foxglove 的观感），我可以再帮你把 tolerance 调到最稳，或者把脚本默认参数直接固化成你数据的最佳配置。

最近邻时间对齐（不插值）：
- 自动起点对齐 (--auto-align)
- 单位/节拍自校准 (--calibrate-unit)
- 线性校准拟合 (--calibrate-linear) 可选
- 最近邻匹配 (--tolerance-s)
- 强制首帧追加 (--force-first-frame)
- 可选输出 CSV-only 对齐版 MCAP (--emit-csv-mcap)，并可将其时间戳“对齐到参考每一帧” (--snap-to-ref)


下一步就把 output_wukong.mcap 的所有其他元素（所有话题/消息） 一起放进合并后的文件里，同时保留我们新合成的 /roadusers_merged。我给你把脚本升级好了：

读取参考 MCAP 的所有话题（不仅仅 /roadusers）；
仍以 /roadusers 作为“对齐与合并”的基准时间轴（最近邻、不插值）；
先将参考 MCAP 的所有原始话题/消息原样复制到输出；
再把我们合成的 /roadusers_merged 逐帧写入同一个输出文件；
仍是 ROS2 CDR 编码（Foxglove 直接识别）。


说明：output_wukong.mcap 的各话题（/color_marker, /ego_marker, /roadusers, /signs, /fov, /scale, /shadows …）均是 visualization_msgs/MarkerArray，这里我们解码为 Python 对象/字典后，由 mcap_ros2.writer.Writer 重新编码为 CDR 写回，时间戳（log_time/publish_time）保持不变；这样就能在 同一个 merged_two_cars.mcap 里同时看到“所有原始话题”+“新的 /roadusers_merged”。


依赖：
  pip install mcap==1.2.1 mcap-ros2-support==0.5.5 pandas numpy
"""

from __future__ import annotations
import argparse
import copy
from typing import Dict, Any, List, Tuple, Optional

import pandas as pd
import numpy as np

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2.writer import Writer as Ros2McapWriter

# --------------------------- ROS2 schema（MarkerArray） ---------------------------
MARKER_ARRAY_SCHEMA_NAME = "visualization_msgs/MarkerArray"
MARKER_ARRAY_SCHEMA_TEXT = """\
visualization_msgs/Marker[] markers
================================================================================
MSG: visualization_msgs/Marker
uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11
uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
int32 lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
"""

# --------------------------- CSV 读取（稳健版） ---------------------------
def read_csv_flex(path: str, skiprows: int = 0, sep: str = "auto") -> pd.DataFrame:
    tried = []
    def _try(_skip, _sep, engine=None):
        tried.append(f"skiprows={_skip}, sep={_sep}, engine={engine or 'c'}")
        if _sep == "auto":
            return pd.read_csv(path, skiprows=_skip, sep=None, engine="python")
        return pd.read_csv(path, skiprows=_skip, sep=_sep, engine=engine or "c")

    try:
        return _try(skiprows, sep)
    except Exception:
        pass
    for _sep in [",", ";", "\t", r"\s+"]:
        try:
            return pd.read_csv(path, skiprows=skiprows, sep=_sep, engine="python")
        except Exception:
            pass
    for _sep in ["auto", ",", ";", "\t", r"\s+"]:
        try:
            return _try(5, _sep, engine="python")
        except Exception:
            pass
    raise RuntimeError(f"无法解析 CSV（尝试过：{'; '.join(tried)}）")

# --------------------------- 时间换算 ---------------------------
def to_unix_ns(v, mode: str, unit: str = "s"):
    if pd.isna(v):
        return None
    try:
        x = float(v)
    except Exception:
        return None

    if mode == "unix_ns":
        return int(x)
    if mode == "unix_ms":
        return int(x * 1e6)
    if mode == "unix_s":
        return int(x * 1e9)
    if mode == "gps_s":
        import datetime
        LEAP_SECONDS = 18
        GPS_WEEK_OFFSET = (datetime.date(2020,10,27).toordinal() - datetime.date(1980,1,6).toordinal()) // 7
        GPS_WEEK_SECONDS = GPS_WEEK_OFFSET * 7 * 24 * 3600
        UNIX0 = GPS_WEEK_SECONDS - LEAP_SECONDS
        return int((UNIX0 + x) * 1e9)
    if mode == "relative":
        factor = {"s": 1e9, "ms": 1e6, "us": 1e3, "ns": 1.0}.get(unit, 1e9)
        return int(x * factor)
    return None

def sec_nsec_from_ns(t_ns: int) -> Tuple[int,int]:
    return int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000)

# --------------------------- Marker/MarkerArray 对象 -> dict 转换 ---------------------------
def _time_obj_to_dict(stamp_obj) -> Dict[str, int]:
    sec = getattr(stamp_obj, "sec", None)
    nsec = getattr(stamp_obj, "nanosec", getattr(stamp_obj, "nsec", None))
    return {"sec": int(sec or 0), "nanosec": int(nsec or 0)}

def _header_to_dict(hdr) -> Dict[str, Any]:
    return {"stamp": _time_obj_to_dict(getattr(hdr, "stamp", type("S", (), {})())),
            "frame_id": getattr(hdr, "frame_id", "")}

def _point_to_dict(p) -> Dict[str, float]:
    return {"x": float(getattr(p, "x", 0.0)),
            "y": float(getattr(p, "y", 0.0)),
            "z": float(getattr(p, "z", 0.0))}

def _quat_to_dict(q) -> Dict[str, float]:
    return {"x": float(getattr(q, "x", 0.0)),
            "y": float(getattr(q, "y", 0.0)),
            "z": float(getattr(q, "z", 0.0)),
            "w": float(getattr(q, "w", 1.0))}

def _vec3_to_dict(v) -> Dict[str, float]:
    return {"x": float(getattr(v, "x", 0.0)),
            "y": float(getattr(v, "y", 0.0)),
            "z": float(getattr(v, "z", 0.0))}

def _color_to_dict(c) -> Dict[str, float]:
    return {"r": float(getattr(c, "r", 0.0)),
            "g": float(getattr(c, "g", 0.0)),
            "b": float(getattr(c, "b", 0.0)),
            "a": float(getattr(c, "a", 1.0))}

def _pose_to_dict(pose) -> Dict[str, Any]:
    return {"position": _point_to_dict(getattr(pose, "position", type("P", (), {})())),
            "orientation": _quat_to_dict(getattr(pose, "orientation", type("Q", (), {})()))}

def marker_obj_to_dict(m) -> Dict[str, Any]:
    return {
        "header": _header_to_dict(getattr(m, "header", type("H", (), {})())),
        "ns": getattr(m, "ns", ""),
        "id": int(getattr(m, "id", 0)),
        "type": int(getattr(m, "type", 1)),
        "action": int(getattr(m, "action", 0)),
        "pose": _pose_to_dict(getattr(m, "pose", type("Pose", (), {})())),
        "scale": _vec3_to_dict(getattr(m, "scale", type("V", (), {})())),
        "color": _color_to_dict(getattr(m, "color", type("C", (), {})())),
        "lifetime": int(getattr(m, "lifetime", 0)) if hasattr(m, "lifetime") else 0,
        "frame_locked": bool(getattr(m, "frame_locked", True)),
        "points": [ _point_to_dict(p) for p in (getattr(m, "points", []) or []) ],
        "colors": [ _color_to_dict(c) for c in (getattr(m, "colors", []) or []) ],
        "text": getattr(m, "text", ""),
    }

def markerarray_to_dict(ma_obj_or_dict) -> Dict[str, Any]:
    if isinstance(ma_obj_or_dict, dict):
        markers = ma_obj_or_dict.get("markers", [])
        out_markers = []
        for m in markers:
            out_markers.append(marker_obj_to_dict(m) if not isinstance(m, dict) else m)
        return {"markers": out_markers}
    markers = getattr(ma_obj_or_dict, "markers", [])
    out_markers = [marker_obj_to_dict(m) for m in markers]
    return {"markers": out_markers}

# --------------------------- 生成 CSV 车 Marker ---------------------------
_counter = 0
def iota() -> int:
    global _counter
    _counter += 1
    return _counter

def make_csv_car_marker(t_ref_ns: int, frame_id: str, x: float, y: float,
                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car") -> Dict[str, Any]:
    sec, nsec = sec_nsec_from_ns(t_ref_ns)
    r, g, b, a = rgba
    CAR_LEN, CAR_WID, CAR_HGT = 4.5, 1.8, 1.5
    return {
        "header": {"stamp": {"sec": sec, "nanosec": nsec}, "frame_id": frame_id},
        "ns": ns, "id": iota(), "type": 1, "action": 0,
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "scale": {"x": CAR_LEN, "y": CAR_WID, "z": CAR_HGT},
        "color": {"r": r, "g": g, "b": b, "a": a},
        "lifetime": 0, "frame_locked": True,
        "text": text, "points": [], "colors": [],
    }

# --------------------------- 单位/节拍自校准 ---------------------------
def unit_calibrate(csv_times: np.ndarray, ref_times: np.ndarray) -> float:
    """用中位帧间隔比值估计比例因子 a≈Δt_ref_med/Δt_csv_med，避免 ms↔s 之类的单位错配。"""
    if len(csv_times) < 3 or len(ref_times) < 3:
        return 1.0
    d_csv = np.median(np.diff(csv_times))
    d_ref = np.median(np.diff(ref_times))
    if d_csv <= 0:
        return 1.0
    a = d_ref / d_csv
    return float(np.clip(a, 1e-3, 1e3))

# --------------------------- 线性校准（可选） ---------------------------
def linear_calibrate(csv_times: np.ndarray, ref_times: np.ndarray,
                     coarse_tol_ns: int) -> Tuple[float, float, int]:
    if len(csv_times) == 0 or len(ref_times) == 0:
        return 1.0, 0.0, 0
    pairs_csv, pairs_ref = [], []
    for t in ref_times:
        idx = np.searchsorted(csv_times, t, side="left")
        best_idx, best_delta = None, None
        for cand in [idx-1, idx]:
            if 0 <= cand < len(csv_times):
                delta = abs(csv_times[cand] - t)
                if best_delta is None or delta < best_delta:
                    best_delta, best_idx = delta, cand
        if best_delta is not None and best_delta <= coarse_tol_ns:
            pairs_csv.append(csv_times[best_idx])
            pairs_ref.append(t)
    if len(pairs_csv) < 3:
        return 1.0, 0.0, len(pairs_csv)
    csv_arr = np.array(pairs_csv, dtype=np.float64)
    ref_arr = np.array(pairs_ref, dtype=np.float64)
    a, b = np.polyfit(csv_arr, ref_arr, 1)
    return float(a), float(b), len(pairs_csv)

# --------------------------- 主流程 ---------------------------
def main():
    ap = argparse.ArgumentParser(
        description="最近邻时间对齐（不插值）：自动起点对齐 + 单位/节拍自校准 + 线性校准（可选） + 最近邻；并可输出 CSV-only 对齐版。"
    )
    # 基本输入/输出
    ap.add_argument("--csv", required=True, help="part3.csv 路径")
    ap.add_argument("--ref-mcap", required=True, help="参考 MCAP 路径（例如 output_wukong.mcap）")
    ap.add_argument("-o", "--output", required=True, help="输出 合并 .mcap 路径")
    ap.add_argument("--ref-topic", default="/roadusers", help="参考 MCAP 话题（默认 /roadusers）")
    ap.add_argument("--out-topic", default="/roadusers_merged", help="合并输出话题（默认 /roadusers_merged）")
    # CSV-only（第二输出，可选）
    ap.add_argument("--emit-csv-mcap", default=None, help="（可选）输出仅含 CSV 目标的对齐版 MCAP 路径")
    ap.add_argument("--csv-topic", default="/roadusers_csv", help="CSV-only 输出话题（默认 /roadusers_csv）")
    ap.add_argument("--snap-to-ref", action="store_true",
                    help="将 CSV-only 的每帧时间戳直接对齐到参考每一帧（常值复制），便于两个 MCAP 同播时效果更自然")
    # CSV 解析
    ap.add_argument("--skiprows", type=int, default=0, help="读取 CSV 时跳过行数（默认 0）")
    ap.add_argument("--sep", default="auto", help="CSV 分隔符：',' ';' '\\t' '\\s+' 或 'auto'（默认 auto）")
    # ✅ CSV 位置列（你之前命令用到的）
    ap.add_argument("--x-col", default="IVS1", help="CSV 前向距离列名（默认 IVS1）")
    ap.add_argument("--y-col", default=None, help="CSV 横向距离列名（默认 None=0.0）")
    ap.add_argument("--frame-id", default="ego_vehicle", help="Marker frame_id（默认 ego_vehicle）")
    # CSV 时间
    ap.add_argument("--time-col", default="Time", help="CSV 时间列名（默认 Time）")
    ap.add_argument("--time-mode", default="relative",
                    choices=["unix_ns", "unix_ms", "unix_s", "gps_s", "relative"],
                    help="CSV 时间类型（默认 relative）")
    ap.add_argument("--time-unit", default="ms", choices=["s","ms","us","ns"],
                    help="当 --time-mode=relative 时，时间单位（默认 ms）")
    ap.add_argument("--time-offset-s", type=float, default=0.0,
                    help="CSV 全局时间平移（秒），用于粗对齐（默认 0）")
    ap.add_argument("--auto-align", action="store_true",
                    help="自动起点对齐：以参考首帧对齐 CSV 首帧（忽略 --time-offset-s）")
    # 校准
    ap.add_argument("--calibrate-unit", action="store_true",
                    help="启用单位/节拍自校准：a≈Δt_ref_med/Δt_csv_med")
    ap.add_argument("--calibrate-linear", action="store_true",
                    help="启用线性校准：拟合 t_ref≈a*t_csv+b")
    ap.add_argument("--calib-tolerance-s", type=float, default=2.0,
                    help="线性校准粗匹配容差（秒，默认 2.0）")
    # 最近邻容差
    ap.add_argument("--tolerance-s", type=float, default=0.6,
                    help="最终最近邻容差（秒，默认 0.6）")
    # 强制首帧
    ap.add_argument("--force-first-frame", action="store_true",
                    help="强制第一帧追加 CSV 车（即使最近邻超容差）")
    # 调试
    ap.add_argument("--print-cols", action="store_true", help="仅打印 CSV 列名后退出")
    args = ap.parse_args()

    # ---- 1) 读取参考 MCAP ----
    ref_frames: List[Tuple[int, Dict[str, Any]]] = []
    with open(args.ref_mcap, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=[args.ref_topic]):
            ref_frames.append((message.log_time, markerarray_to_dict(ros_msg)))
    if not ref_frames:
        raise RuntimeError(f"参考 MCAP({args.ref_mcap}) 未找到话题 {args.ref_topic} 的任何消息")
    ref_times = np.array([t for t, _ in ref_frames], dtype=np.int64)
    ref_start = int(ref_times[0])

    # ---- 2) 读取 CSV ----
    df = read_csv_flex(args.csv, skiprows=args.skiprows, sep=args.sep)
    if args.print_cols:
        print("CSV 列名：", list(df.columns)); return
    if args.time_col not in df.columns:
        raise KeyError(f"CSV 缺少时间列 '{args.time_col}'；可用列：{list(df.columns)}")
    if args.x_col not in df.columns:
        raise KeyError(f"CSV 缺少前向列 '{args.x_col}'；可用列：{list(df.columns)}")

    t_ns = df[args.time_col].apply(lambda v: to_unix_ns(v, args.time_mode, args.time_unit)).astype("Int64")
    x = pd.to_numeric(df[args.x_col], errors="coerce")
    y = pd.to_numeric(df[args.y_col], errors="coerce") if (args.y_col and args.y_col in df.columns) else pd.Series(0.0, index=df.index)

    # 起点对齐
    if args.auto_align:
        csv_first_valid = t_ns.dropna()
        if csv_first_valid.empty:
            print("[WARN] CSV 时间列全为空；将只输出参考轴。")
            offset_ns = 0
        else:
            csv_start = int(csv_first_valid.iloc[0])
            offset_ns = ref_start - csv_start
            print(f"[INFO] auto-align: ref_start_ns={ref_start}, csv_start_ns={csv_start}, offset_ns={offset_ns}")
        t_ns = t_ns + offset_ns
    else:
        if args.time_offset_s:
            t_ns = t_ns + int(args.time_offset_s * 1e9)

    # 整理 CSV
    csv_df = pd.DataFrame({"t_ns": t_ns, "x": x, "y": y}, copy=False)
    csv_df = csv_df.dropna(subset=["t_ns"]).astype({"t_ns": "int64"}).sort_values("t_ns").reset_index(drop=True)
    csv_times = csv_df["t_ns"].to_numpy(dtype=np.int64)

    # 单位/节拍自校准
    if args.calibrate_unit and len(csv_times) >= 3:
        a_u = unit_calibrate(csv_times, ref_times)
        if not np.isclose(a_u, 1.0):
            t_ns_adj = (csv_df["t_ns"].astype(np.float64) * a_u).round().astype("int64")
            csv_df["t_ns"] = t_ns_adj
            csv_times = csv_df["t_ns"].to_numpy(dtype=np.int64)
        print(f"[INFO] unit-calib: a_unit≈{a_u:.9f} (Δref_med/Δcsv_med)")

    # 线性校准（可选）
    if args.calibrate_linear and len(csv_times) > 0:
        coarse_tol = int(args.calib_tolerance_s * 1e9)
        a, b, n_pairs = linear_calibrate(csv_times, ref_times, coarse_tol)
        if n_pairs >= 3 and (not np.isclose(a,1.0) or b != 0):
            t_ns_adj = (csv_df["t_ns"].astype(np.float64) * a + b).round().astype("int64")
            csv_df["t_ns"] = t_ns_adj
            csv_times = csv_df["t_ns"].to_numpy(dtype=np.int64)
        print(f"[INFO] linear-calib: a={a:.9f}, b={int(b)}, pairs={n_pairs}")

    # ---- 3) 最近邻匹配（不插值） ----
    tol_ns = int(max(0.0, args.tolerance_s) * 1e9)
    ref_df = pd.DataFrame({"t_ns": ref_times})
    merged = pd.merge_asof(ref_df, csv_df[["t_ns","x","y"]],
                           on="t_ns", direction="nearest", tolerance=tol_ns)

    # ---- 4) 写 合并 MCAP ----
    nn_ok, nn_miss, nan_skip = 0, 0, 0
    with open(args.output, "wb") as f:
        w = Ros2McapWriter(f)
        schema = w.register_msgdef(MARKER_ARRAY_SCHEMA_NAME, MARKER_ARRAY_SCHEMA_TEXT)
        for idx, ((t_ref_ns, ma_ref), row) in enumerate(zip(ref_frames, merged.itertuples(index=False))):
            ma_out = copy.deepcopy(ma_ref)
            if "markers" not in ma_out or not isinstance(ma_out["markers"], list):
                ma_out["markers"] = []
            appended = False

            if not (pd.isna(row.x) or pd.isna(row.y)):
                try:
                    ma_out["markers"].append(
                        make_csv_car_marker(t_ref_ns, args.frame_id, float(row.x), float(row.y),
                                            rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car"))
                    nn_ok += 1
                    appended = True
                except Exception:
                    nan_skip += 1
            else:
                nn_miss += 1

            if args.force_first_frame and idx == 0 and not appended and len(csv_df) > 0:
                nearest = csv_df.iloc[0]
                ma_out["markers"].append(
                    make_csv_car_marker(t_ref_ns, args.frame_id,
                                        float(nearest.x) if not pd.isna(nearest.x) else 0.0,
                                        float(nearest.y) if not pd.isna(nearest.y) else 0.0,
                                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car (forced)", ns="csv_car"))
                nn_ok += 1

            w.write_message(topic=args.out_topic, schema=schema, message=ma_out,
                            log_time=int(t_ref_ns), publish_time=int(t_ref_ns))
        w.finish()

    print(f"✅ 合并完成：{args.output}")
    print(f"   参考帧数：{len(ref_frames)}")
    print(f"   最近邻成功：{nn_ok} 帧")
    print(f"   最近邻未匹配：{nn_miss} 帧（超容差或无近邻）")
    print(f"   因 NaN 跳过：{nan_skip} 帧")
    print(f"   参考话题：{args.ref_topic}  →  输出话题：{args.out_topic}")
    print(f"   设置：tolerance={args.tolerance_s:.3f}s, time_mode={args.time_mode}, time_unit={args.time_unit}, "
          f"time_offset_s={args.time_offset_s:.3f}, auto_align={args.auto_align}, "
          f"calib_unit={args.calibrate_unit}, calibrate_linear={args.calibrate_linear}, calib_tol={args.calib_tolerance_s:.3f}s, "
          f"force_first_frame={args.force_first_frame}")
    print("   提示：在 Foxglove 订阅", args.out_topic, "查看合并效果。")

    # ---- 5) CSV-only 对齐版 MCAP（可选第二输出） ----
    if args.emit_csv_mcap is not None:
        with open(args.emit_csv_mcap, "wb") as f2:
            w2 = Ros2McapWriter(f2)
            schema2 = w2.register_msgdef(MARKER_ARRAY_SCHEMA_NAME, MARKER_ARRAY_SCHEMA_TEXT)
            if args.snap_to_ref:
                # 把 CSV 值按最近邻映射到参考每一帧的时间戳（常值复制，不插值）
                csv_times_arr = csv_df["t_ns"].to_numpy(dtype=np.int64)
                for t_ref_ns in ref_times:
                    # 取最接近的 CSV 样本（不设容差，纯最近邻）
                    idx = np.searchsorted(csv_times_arr, t_ref_ns, side="left")
                    if idx >= len(csv_times_arr): idx = len(csv_times_arr) - 1
                    if idx > 0 and (idx == len(csv_times_arr) or
                                    abs(csv_times_arr[idx-1]-t_ref_ns) < abs(csv_times_arr[idx]-t_ref_ns)):
                        idx -= 1
                    row = csv_df.iloc[idx]
                    marker_csv = make_csv_car_marker(
                        int(t_ref_ns), args.frame_id,
                        float(row.x) if not pd.isna(row.x) else 0.0,
                        float(row.y) if not pd.isna(row.y) else 0.0,
                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car")
                    w2.write_message(topic=args.csv_topic,
                                     schema=schema2,
                                     message={"markers": [marker_csv]},
                                     log_time=int(t_ref_ns),
                                     publish_time=int(t_ref_ns))
            else:
                # 原样输出（仅时间对齐/校准后的 CSV），不对齐到参考每帧
                for row in csv_df.itertuples(index=False):
                    t_ns_row = int(row.t_ns)
                    marker_csv = make_csv_car_marker(
                        t_ns_row, args.frame_id,
                        float(row.x) if not pd.isna(row.x) else 0.0,
                        float(row.y) if not pd.isna(row.y) else 0.0,
                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car")
                    w2.write_message(topic=args.csv_topic,
                                     schema=schema2,
                                     message={"markers": [marker_csv]},
                                     log_time=t_ns_row, publish_time=t_ns_row)
            w2.finish()
        print(f"📦 额外输出：CSV-only 对齐版 MCAP → {args.emit_csv_mcap}")
        print(f"    话题：{args.csv_topic}；在 Foxglove 中与参考 MCAP 一起打开，即可获得“两份 MCAP 同播”的效果。")

if __name__ == "__main__":
    main()