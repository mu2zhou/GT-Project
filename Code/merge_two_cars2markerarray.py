#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
"""
流程：

读取 output_wukong.mcap（作为基准时间轴，读取 /roadusers 的 MarkerArray）
读取 part3.csv（从列里取你关心的车距，生成“第二车”的 Marker）
按基准 MCAP 的每个时间戳 找到 CSV 的最近一帧（可设容差），把两边的 Marker 合并到同一个 MarkerArray
用 mcap_ros2.writer.Writer 以 ROS 2 CDR 编码写出新的 MCAP（Foxglove 直接识别）


读取/写入 ROS 2 数据使用官方 mcap / mcap_ros2-support 的 Python API：

读取：make_reader(..., decoder_factories=[mcap_ros2.decoder.DecoderFactory()]) 迭代解码后的 ROS 2 消息（无需 ROS 环境）
写入：mcap_ros2.writer.Writer 注册 visualization_msgs/MarkerArray 的 msgdef，并将 Python dict 自动编码为 CDR 写入

将 output_wukong.mcap 的 /roadusers 作为“基准时间轴”，
把 part3.csv 生成的“第二车” Marker 合并到同一 MarkerArray，
并输出一个新的 MCAP（ROS2 CDR 编码，Foxglove 直接识别）。

依赖：
  pip install mcap==1.2.1 mcap-ros2-support==0.5.5 pandas

参考：
  - 读取ROS2：mcap.reader.make_reader + mcap_ros2.decoder.DecoderFactory（无需ROS运行时）
  - 写入ROS2：mcap_ros2.writer.Writer.register_msgdef + write_message（CDR编码）


# 1) 常见情况：CSV 的时间列是 GPS秒，需要平移到参考轴（如已知 offset）
python Code/merge_two_cars2markerarray.py \
  --csv ./Dataset/OpenACC/data/Casale/part3.csv \
  --ref-mcap wmdd_example_datas/output_wukong.mcap \
  --time-col Time --time-mode gps_s --time-offset-s 0 \
  --x-col IVS1 \
  --tolerance-s 0.6 \
  --resample linear --extrapolate hold \
  --out-topic /roadusers_merged \
  --skiprows 5 \
  -o ./Dataset/OpenACC/data/Casale/merged_two_cars.mcap

# 2) 如果 CSV 时间本来就是 epoch 纳秒（unix_ns），并且需要整体后移 0.15s 才能贴合
python Code/merge_two_cars2markerarray.py \
  --csv Dataset/OpenACC/data/Casale/part3.csv \
  --ref-mcap wmdd_example_datas/output_wukong.mcap \
  --time-col Time --time-mode unix_ns --time-offset-s 0.15 \
  --x-col IVS1 --y-col LAT_OFFSET \
  --tolerance-s 0.3 \
  --sep auto --skiprows 5 \
  -o merged_two_cars.mcap  


小贴士：

如果你希望严格“一帧一帧对应”，把 --tolerance-s 设小一些（如 0.05–0.10s），并尽量把 --time-offset-s 调到最佳；否则就会有些帧没有匹配，从而只显示参考 MCAP 的 Marker。
上面脚本默认只用 CSV 的前向距离 IVS1 作为 X 轴，Y=0；如有横向列（如 LatOffset），用 --y-col 指定即可。
输出话题默认是 /roadusers_merged，不覆盖原始话题，避免混淆。

自动时间对齐（--auto-align）：

取参考轴的首帧时间 ref_start_ns 与 CSV 的首个有效时间 csv_start_ns；
计算 offset_ns = ref_start_ns - csv_start_ns，自动平移 CSV 全部时间到参考轴；
再做最近邻匹配（可设 --tolerance-s，默认 0.5s）。



强制首帧匹配（--force-first-frame）：

即使最近邻超出容差，也会确保第一帧追加一条 CSV 车（取 CSV 第一条记录或最近记录）；
这样就满足你“至少第一帧同步”的要求。



详细统计与提示：

输出匹配统计（成功匹配帧数 / 因容差跳过帧数 / CSV NaN 跳过帧数）；
若 y 列名缺失或全是 NaN，会提醒并自动用 0.0（沿 x 轴）；
明确输出写到了哪个 topic（注意在 Foxglove 订阅 /roadusers_merged）。

关键改动


明确/可配置时间单位：新增 --time-mode relative 与 --time-unit {s,ms,us,ns}。

若 CSV 是相对时间（常见：毫秒、秒），请用 --time-mode relative --time-unit ms（或 s/us/ns）。
仍支持 unix_* 和 gps_s。



自动对齐（--auto-align）：以参考轴首帧对齐 CSV 首帧（保留你之前的功能）。


线性插值重采样（--resample linear）：

即使最近邻超容差，也会在参考时间点对 CSV 数据线性插值（可选常量外推）。
这样每一帧都会有 CSV 车，避免“看不到”问题。
若仍希望只在容差内附加（不插值），可用 --resample nearest（默认）。



外推策略（--extrapolate {none|hold|linear}）：

none（默认）：超出 CSV 范围的不生成 CSV 车；
hold：用边界值常量外推；
linear：用边界线性外推。



详细统计：匹配成功/插值生成/因 NaN 跳过/因越界跳过，帮助你判断对齐质量。

"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
以参考 MCAP 的 /roadusers 时间轴，将 CSV 生成的第二车合并到同一 MarkerArray（ROS2 CDR 输出）。
增强点：
- 时间单位可配置：--time-mode relative + --time-unit {s,ms,us,ns}
- 自动时间对齐 (--auto-align)：对齐 CSV 首帧到参考轴首帧
- 线性插值重采样 (--resample linear)，并支持外推 (--extrapolate none|hold|linear)
- 强制首帧匹配 (--force-first-frame)
- 详细统计与提示
"""

# from __future__ import annotations
import argparse
import copy
from typing import Dict, Any, List, Tuple, Optional

import pandas as pd
import numpy as np

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory           # 读取并解码 ROS 2 消息（可能返回对象）
from mcap_ros2.writer import Writer as Ros2McapWriter  # 写 ROS2（CDR编码）

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
        # 当 sep == "auto" 时，强制用 python 引擎（c 引擎不支持 sep=None 自动推断）
        if _sep == "auto":
            return pd.read_csv(path, skiprows=_skip, sep=None, engine="python")
        return pd.read_csv(path, skiprows=_skip, sep=_sep, engine=engine or "c")

    # 1) 用户参数
    try:
        return _try(skiprows, sep)
    except Exception:
        pass
    # 2) 常见分隔符轮换（python 引擎）
    for _sep in [",", ";", "\t", r"\s+"]:
        try:
            return pd.read_csv(path, skiprows=skiprows, sep=_sep, engine="python")
        except Exception:
            pass
    # 3) 回退：假设前 5 行为说明
    for _sep in ["auto", ",", ";", "\t", r"\s+"]:
        try:
            return _try(5, _sep, engine="python")
        except Exception:
            pass

    raise RuntimeError(f"无法解析 CSV（尝试过：{'; '.join(tried)}）")

# --------------------------- 时间换算 ---------------------------
def to_unix_ns(v, mode: str, unit: str = "s"):
    """将 CSV 时间列转换为 epoch 纳秒。支持：
       mode: 'unix_ns'|'unix_ms'|'unix_s'|'gps_s'|'relative'
       unit: 当 mode='relative' 时使用 {s,ms,us,ns}
    """
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

def make_csv_car_marker(
    t_ref_ns: int,
    frame_id: str,
    x: float,
    y: float,
    rgba=(0.0, 0.9, 0.2, 0.95),
    text="CSV Car",
    ns="csv_car",
) -> Dict[str, Any]:
    sec, nsec = sec_nsec_from_ns(t_ref_ns)
    r, g, b, a = rgba
    CAR_LEN, CAR_WID, CAR_HGT = 4.5, 1.8, 1.5
    return {
        "header": {"stamp": {"sec": sec, "nanosec": nsec}, "frame_id": frame_id},
        "ns": ns,
        "id": iota(),
        "type": 1,      # CUBE
        "action": 0,    # ADD/MODIFY
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "scale": {"x": CAR_LEN, "y": CAR_WID, "z": CAR_HGT},
        "color": {"r": r, "g": g, "b": b, "a": a},
        "lifetime": 0,
        "frame_locked": True,
        "text": text,
        "points": [],
        "colors": [],
    }

# --------------------------- 主流程 ---------------------------
def main():
    ap = argparse.ArgumentParser(
        description="以参考 MCAP 的 /roadusers 时间轴，将 CSV 生成的第二车合并到同一 MarkerArray（ROS2 CDR 输出）"
    )
    # 基本输入/输出
    ap.add_argument("--csv", required=True, help="part3.csv 路径")
    ap.add_argument("--ref-mcap", required=True, help="参考 MCAP 路径（例如 output_wukong.mcap）")
    ap.add_argument("-o", "--output", required=True, help="输出 .mcap 路径")
    ap.add_argument("--ref-topic", default="/roadusers", help="参考 MCAP 的话题（默认 /roadusers）")
    ap.add_argument("--out-topic", default="/roadusers_merged", help="输出话题名（默认 /roadusers_merged）")
    # CSV 解析
    ap.add_argument("--skiprows", type=int, default=0, help="读取 CSV 时跳过的行数（默认 0）")
    ap.add_argument("--sep", default="auto", help="CSV 分隔符：',' ';' '\\t' '\\s+' 或 'auto'（默认 auto）")
    # CSV 时间
    ap.add_argument("--time-col", default="Time", help="CSV 时间列名（默认 Time）")
    ap.add_argument("--time-mode", default="relative",
                    choices=["unix_ns", "unix_ms", "unix_s", "gps_s", "relative"],
                    help="CSV 时间类型（默认 relative，相对时间）")
    ap.add_argument("--time-unit", default="ms", choices=["s","ms","us","ns"],
                    help="当 --time-mode=relative 时，时间单位（默认 ms）")
    ap.add_argument("--time-offset-s", type=float, default=0.0,
                    help="CSV 全局时间平移（秒），用于粗对齐到参考轴（默认 0）")
    ap.add_argument("--auto-align", action="store_true",
                    help="自动对齐：以参考轴首帧对齐 CSV 首帧（忽略 --time-offset-s）")
    ap.add_argument("--force-first-frame", action="store_true",
                    help="强制第一帧追加 CSV 车（即使最近邻超容差）")
    # CSV 位置
    ap.add_argument("--x-col", default="IVS1", help="CSV 前向距离列（默认 IVS1）")
    ap.add_argument("--y-col", default=None, help="CSV 横向距离列（默认 None=0.0）")
    ap.add_argument("--frame-id", default="ego_vehicle", help="Marker frame_id（默认 ego_vehicle）")
    # 匹配/重采样策略
    ap.add_argument("--tolerance-s", type=float, default=0.5,
                    help="最近邻容差（秒，默认 0.5；超出则该帧不放 CSV 车，除非插值）")
    ap.add_argument("--resample", default="linear", choices=["nearest","linear"],
                    help="最近邻失败时的重采样方式（默认 linear）")
    ap.add_argument("--extrapolate", default="none", choices=["none","hold","linear"],
                    help="参考时间超出 CSV 范围时的外推策略（默认 none）")
    # 调试
    ap.add_argument("--print-cols", action="store_true", help="仅打印 CSV 列名后退出")
    args = ap.parse_args()

    # ---- 1) 读取参考 MCAP：每帧 (t_ns_ref, MarkerArray_dict) ----
    ref_frames: List[Tuple[int, Dict[str, Any]]] = []
    with open(args.ref_mcap, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=[args.ref_topic]):
            ref_frames.append((message.log_time, markerarray_to_dict(ros_msg)))

    if not ref_frames:
        raise RuntimeError(f"参考 MCAP({args.ref_mcap}) 未找到话题 {args.ref_topic} 的任何消息")
    ref_times_ns = np.array([t for t, _ in ref_frames], dtype=np.int64)
    ref_start_ns = int(ref_times_ns[0])

    # ---- 2) 读取 CSV：转成 (t_ns, x, y) 并按时间排序 ----
    df = read_csv_flex(args.csv, skiprows=args.skiprows, sep=args.sep)
    if args.print_cols:
        print("CSV 列名：", list(df.columns)); return

    if args.time_col not in df.columns:
        raise KeyError(f"CSV 缺少时间列 '{args.time_col}'；可用列：{list(df.columns)}")
    if args.x_col not in df.columns:
        raise KeyError(f"CSV 缺少前向距离列 '{args.x_col}'；可用列：{list(df.columns)}")

    t_ns = df[args.time_col].apply(lambda v: to_unix_ns(v, args.time_mode, args.time_unit)).astype("Int64")
    x = pd.to_numeric(df[args.x_col], errors="coerce")
    if args.y_col and args.y_col in df.columns:
        y = pd.to_numeric(df[args.y_col], errors="coerce")
    else:
        y = pd.Series(0.0, index=df.index)

    # 自动对齐：以参考首帧对齐 CSV 首帧（忽略用户提供的 time-offset-s）
    if args.auto_align:
        csv_first_valid = t_ns.dropna()
        if csv_first_valid.empty:
            print("[WARN] CSV 时间列全为空，无法自动对齐；将只输出参考轴 Marker。")
            time_offset_ns = 0
        else:
            csv_start_ns = int(csv_first_valid.iloc[0])
            time_offset_ns = ref_start_ns - csv_start_ns
            print(f"[INFO] auto-align: ref_start_ns={ref_start_ns}, csv_start_ns={csv_start_ns}, offset_ns={time_offset_ns}")
        t_ns = t_ns + time_offset_ns
    else:
        if args.time_offset_s:
            t_ns = t_ns + int(args.time_offset_s * 1e9)

    # 整理 CSV 时间/位置
    csv_df = pd.DataFrame({"t_ns": t_ns, "x": x, "y": y}, copy=False)
    csv_df = csv_df.dropna(subset=["t_ns"]).astype({"t_ns": "int64"}).sort_values("t_ns").reset_index(drop=True)
    if csv_df.empty:
        print("[WARN] CSV 有效时间列为空，合并将只包含参考 MCAP 的 Marker。")

    # ---- 3) 最近邻匹配（参考时间 -> CSV 最近样本） ----
    tol_ns = int(max(0.0, args.tolerance_s) * 1e9)
    ref_df = pd.DataFrame({"t_ns": ref_times_ns})
    merged = pd.merge_asof(ref_df, csv_df[["t_ns","x","y"]],
                           on="t_ns", direction="nearest", tolerance=tol_ns)

    # 为插值准备数据
    csv_times = csv_df["t_ns"].to_numpy(dtype=np.int64)
    csv_x = csv_df["x"].to_numpy(dtype=float)
    csv_y = csv_df["y"].to_numpy(dtype=float)

    def interp_at(t: int) -> Optional[Tuple[float,float]]:
        if len(csv_times) == 0:
            return None
        # 边界处理
        if t < csv_times[0]:
            if args.extrapolate == "hold":
                return float(csv_x[0]), float(csv_y[0])
            elif args.extrapolate == "linear" and len(csv_times) >= 2:
                t0, t1 = csv_times[0], csv_times[1]
                w = (t - t0) / (t1 - t0) if (t1 != t0) else 0.0
                xval = csv_x[0] + w * (csv_x[1] - csv_x[0])
                yval = csv_y[0] + w * (csv_y[1] - csv_y[0])
                return float(xval), float(yval)
            else:
                return None
        if t > csv_times[-1]:
            if args.extrapolate == "hold":
                return float(csv_x[-1]), float(csv_y[-1])
            elif args.extrapolate == "linear" and len(csv_times) >= 2:
                t0, t1 = csv_times[-2], csv_times[-1]
                w = (t - t1) / (t1 - t0) if (t1 != t0) else 0.0
                xval = csv_x[-1] + w * (csv_x[-1] - csv_x[-2])
                yval = csv_y[-1] + w * (csv_y[-1] - csv_y[-2])
                return float(xval), float(yval)
            else:
                return None
        # 区间内线性插值
        idx = np.searchsorted(csv_times, t, side="left")
        if idx == 0:
            return float(csv_x[0]), float(csv_y[0])
        t0, t1 = csv_times[idx-1], csv_times[idx]
        x0, x1 = csv_x[idx-1], csv_x[idx]
        y0, y1 = csv_y[idx-1], csv_y[idx]
        w = (t - t0) / (t1 - t0) if (t1 != t0) else 0.0
        xval = x0 + w * (x1 - x0)
        yval = y0 + w * (y1 - y0)
        return float(xval), float(yval)

    # ---- 4) 写合并后的 MCAP（ROS2 CDR） ----
    matched_ok = 0
    interpolated_ok = 0
    skipped_nan = 0
    skipped_out = 0

    with open(args.output, "wb") as f:
        w = Ros2McapWriter(f)      # profile=ros2，CDR 序列化
        schema = w.register_msgdef(MARKER_ARRAY_SCHEMA_NAME, MARKER_ARRAY_SCHEMA_TEXT)

        for idx, ((t_ref_ns, ma_ref_dict), row) in enumerate(zip(ref_frames, merged.itertuples(index=False))):
            ma_out = copy.deepcopy(ma_ref_dict)
            if "markers" not in ma_out or not isinstance(ma_out["markers"], list):
                ma_out["markers"] = []

            appended = False

            # 先尝试最近邻（在容差内）
            if args.resample == "nearest" and not (pd.isna(row.x) or pd.isna(row.y)):
                try:
                    marker_csv = make_csv_car_marker(
                        t_ref_ns, args.frame_id, float(row.x), float(row.y),
                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car")
                    ma_out["markers"].append(marker_csv)
                    matched_ok += 1
                    appended = True
                except Exception:
                    skipped_nan += 1

            # 最近邻失败或选择线性插值
            if not appended and args.resample == "linear":
                pair = interp_at(int(t_ref_ns))
                if pair is not None:
                    xval, yval = pair
                    try:
                        marker_csv = make_csv_car_marker(
                            t_ref_ns, args.frame_id, float(xval), float(yval),
                            rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car (interp)", ns="csv_car")
                        ma_out["markers"].append(marker_csv)
                        interpolated_ok += 1
                        appended = True
                    except Exception:
                        skipped_nan += 1
                else:
                    skipped_out += 1

            # 强制首帧追加（即使还没追加）
            if args.force_first_frame and idx == 0 and not appended and not csv_df.empty:
                nearest = csv_df.iloc[0]
                marker_csv = make_csv_car_marker(
                    t_ref_ns, args.frame_id,
                    float(nearest.x) if not pd.isna(nearest.x) else 0.0,
                    float(nearest.y) if not pd.isna(nearest.y) else 0.0,
                    rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car (forced)", ns="csv_car"
                )
                ma_out["markers"].append(marker_csv)
                matched_ok += 1
                appended = True

            w.write_message(topic=args.out_topic, schema=schema, message=ma_out,
                            log_time=int(t_ref_ns), publish_time=int(t_ref_ns))

        w.finish()

    print(f"✅ 合并完成：{args.output}")
    print(f"   参考帧数：{len(ref_frames)}")
    print(f"   最近邻成功：{matched_ok} 帧")
    print(f"   线性插值生成：{interpolated_ok} 帧")
    print(f"   因 NaN 跳过：{skipped_nan} 帧")
    print(f"   因越界跳过：{skipped_out} 帧（超出 CSV 时间范围且 extrapolate={args.extrapolate}）")
    print(f"   参考话题：{args.ref_topic}  →  输出话题：{args.out_topic}")
    print(f"   设置：tolerance={args.tolerance_s:.3f}s, time_mode={args.time_mode}, time_unit={args.time_unit}, time_offset_s={args.time_offset_s:.3f}, auto_align={args.auto_align}, resample={args.resample}, extrapolate={args.extrapolate}, force_first_frame={args.force_first_frame}")
    print("   提示：在 Foxglove 中订阅", args.out_topic, "来查看合并结果。")

if __name__ == "__main__":
    main()