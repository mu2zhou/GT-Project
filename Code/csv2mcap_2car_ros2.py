#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenACC CSV  ->  ROS2-MCAP (CDR, visualization_msgs/MarkerArray)
目标：与 generate_mcap.py 生成的目标 MCAP 完全兼容（同主题/同消息类型），
     以便在 Foxglove 中合并播放两车。

依赖：
  pip install mcap==1.2.1 mcap-ros2-support==0.5.5 pandas pyproj
"""

import os, sys, argparse, datetime, json
import pandas as pd
import numpy as np

# === 关键：使用 ROS2 写入器，自动完成 CDR 序列化 ===
from mcap_ros2.writer import Writer as Ros2McapWriter   # [1](https://mcap.dev/docs/python/mcap-apidoc/mcap.writer)

# ------------------------ CLI ------------------------
def build_args():
    ap = argparse.ArgumentParser(
        prog="csv2mcap_2car_ros2.py",
        description="OpenACC CSV -> MCAP（ROS2 CDR，MarkerArray），用于与目标MCAP并轨播放",
    )
    ap.add_argument("csv", help="输入 CSV 路径（默认首5行为说明）")
    ap.add_argument("-o", "--output", default=None, help="输出 .mcap 路径（默认 <csv>_ros2.mcap）")
    ap.add_argument("--skiprows", type=int, default=5, help="读CSV时跳过的行数（默认 5）")
    # 时间轴设置：与目标MCAP对齐
    ap.add_argument("--time-col", default="Time", help="时间列名（默认 'Time'）")
    ap.add_argument("--time-mode", default="gps_s",
                    choices=["gps_s", "unix_ns", "unix_ms", "unix_s"],
                    help="时间列单位/类型（默认 gps_s：GPS周秒）")
    ap.add_argument("--time-offset-s", type=float, default=0.0,
                    help="整体时间偏移（秒，正值向后平移），用于与目标MCAP对齐（默认 0）")
    # 数据列设置：两车的纵向相对距离（单位米）
    ap.add_argument("--ivs1-col", default="IVS1", help="车A前向距离列（默认 'IVS1'）")
    ap.add_argument("--ivs2-col", default=None, help="车B前向距离列（可选；缺省不写第二车）")
    # 主题与坐标系
    ap.add_argument("--topic", default="/roadusers", help="输出主题（默认 '/roadusers'）")
    ap.add_argument("--frame-id", default="ego_vehicle", help="Marker坐标系（默认 'ego_vehicle'）")
    return ap.parse_args()

# ------------------------ 时间换算 ------------------------
# 你的历史代码：GPS周秒 -> Unix epoch ns
LEAP_SECONDS = 18
# 参考点：2020-10-27 的 GPS 周偏移（保留你的做法）
GPS_WEEK_OFFSET = (datetime.date(2020, 10, 27).toordinal() - datetime.date(1980, 1, 6).toordinal()) // 7
GPS_WEEK_SECONDS = GPS_WEEK_OFFSET * 7 * 24 * 3600
UNIX0 = GPS_WEEK_SECONDS - LEAP_SECONDS

def to_unix_ns(v, mode: str):
    if pd.isna(v):
        return None
    try:
        if mode == "gps_s":
            return int((UNIX0 + float(v)) * 1_000_000_000)
        elif mode == "unix_ns":
            return int(v)
        elif mode == "unix_ms":
            return int(float(v) * 1_000_000)
        elif mode == "unix_s":
            return int(float(v) * 1_000_000_000)
        else:
            return None
    except Exception:
        return None

# ------------------------ ROS2 schema（与目标MCAP一致） ------------------------
MARKER_ARRAY_SCHEMA_NAME = "visualization_msgs/MarkerArray"
MARKER_ARRAY_SCHEMA_TEXT = b"""\
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

# ------------------------ 工具函数 ------------------------
_car_len, _car_wid, _car_hgt = 4.5, 1.8, 1.5
_colors = [(1.0, 0.0, 0.0, 0.9), (0.0, 0.3, 1.0, 0.9)]  # A:红, B:蓝

def safe_num(v, default=0.0):
    try:
        f = float(v)
        return f if np.isfinite(f) else default
    except Exception:
        return default

_counter = 0
def iota():
    global _counter
    _counter += 1
    return _counter

def stamp_dict(t_ns: int):
    sec = int(t_ns // 1_000_000_000)
    nsec = int(t_ns % 1_000_000_000)
    # 注意：ROS2 的 builtin_interfaces/Time 字段为 sec + nanosec
    return {"sec": sec, "nanosec": nsec}

def mk_delete_marker(t_ns: int):
    return {
        "header": {"stamp": stamp_dict(t_ns)},
        "action": 3,   # DELETEALL
    }

def mk_car_marker(t_ns: int, frame_id: str, x: float, y: float, color_rgba, text: str, ns="cars"):
    r, g, b, a = color_rgba
    return {
        "header": {"stamp": stamp_dict(t_ns), "frame_id": frame_id},
        "ns": ns,
        "id": iota(),
        "type": 1,      # CUBE
        "action": 0,    # ADD/MODIFY
        "pose": {
            "position": {"x": safe_num(x), "y": safe_num(y), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "scale": {"x": _car_len, "y": _car_wid, "z": _car_hgt},
        "color": {"r": r, "g": g, "b": b, "a": a},
        "frame_locked": True,
        "text": text,
    }

# ------------------------ 主流程 ------------------------
def main():
    args = build_args()
    csv_path = args.csv
    out_path = args.output or (os.path.splitext(csv_path)[0] + "_ros2.mcap")

    # 1) 读取 CSV
    df = pd.read_csv(csv_path, skiprows=args.skiprows)

    # 2) 打开 MCAP（ROS2）
    with open(out_path, "wb") as f:
        writer = Ros2McapWriter(f)  # 库会写入 profile=ros2 且用 CDR 编码  [1](https://mcap.dev/docs/python/mcap-apidoc/mcap.writer)
        schema = writer.register_msgdef(MARKER_ARRAY_SCHEMA_NAME, MARKER_ARRAY_SCHEMA_TEXT.decode("utf-8"))

        # 3) 逐行写 MarkerArray（/roadusers），与目标MCAP主题一致
        topic = args.topic
        frame_id = args.frame_id
        t_offset_ns = int(args.time_offset_s * 1e9)

        # 统计
        n_rows = 0
        n_msgs = 0

        for _, row in df.iterrows():
            # 时间
            t_ns = to_unix_ns(row.get(args.time_col), args.time_mode)
            if t_ns is None:
                continue
            t_ns += t_offset_ns

            # 车辆前向距离（默认：A 用 IVS1，B 可选 IVS2）
            x_a = safe_num(row.get(args.ivs1_col), None)
            x_b = safe_num(row.get(args.ivs2_col), None) if args.ivs2_col else None

            # 生成 MarkerArray
            ma = {"markers": [mk_delete_marker(t_ns)]}
            if x_a is not None:
                ma["markers"].append(mk_car_marker(t_ns, frame_id, x_a, 0.0, _colors[0], text="Car A"))
            if x_b is not None:
                ma["markers"].append(mk_car_marker(t_ns, frame_id, x_b, 0.0, _colors[1], text="Car B"))

            # 若两车都缺失则跳过
            if len(ma["markers"]) == 1:
                continue

            writer.write_message(
                topic=topic,
                schema=schema,
                message=ma,
                log_time=t_ns,       # publish_time 默认为 log_time  [3](https://blog.csdn.net/yk_rainy/article/details/136047285)
            )

            n_rows += 1
            n_msgs += 1

        writer.finish()

    print(f"✅ 已生成 MCAP：{out_path}")
    print(f"   行数参与写入：{n_rows}，消息数：{n_msgs}")
    print(f"   主题：{topic}，schema：{MARKER_ARRAY_SCHEMA_NAME}（ROS2 CDR）")

if __name__ == "__main__":
    main()