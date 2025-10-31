#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, os, copy, subprocess
from typing import Dict, Any, List, Tuple
import pandas as pd, numpy as np

from mcap.reader import make_reader
from mcap.writer import Writer as CoreWriter                    # raw 复制（官方 Writer）  [1](https://mcap.dev/docs/python/raw_reader_writer_example)
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2.writer import Writer as Ros2Writer               # 写 ROS2 CDR  [3](https://github.com/foxglove/mcap/blob/main/python/examples/ros2-noenv/writer.py)
# --------------------------- ROS2 MarkerArray schema ---------------------------
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

# --------------------------- 工具函数 ---------------------------
def abspath(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))

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

_counter = 0
def iota() -> int:
    global _counter
    _counter += 1
    return _counter

def _sec_nsec(t_ns: int) -> Tuple[int, int]:
    return int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000)

def make_csv_car_marker(t_ref_ns: int, frame_id: str, x: float, y: float,
                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car") -> Dict[str, Any]:
    sec, nsec = _sec_nsec(t_ref_ns)
    r, g, b, a = rgba
    CAR_LEN, CAR_WID, CAR_HGT = 4.5, 1.8, 1.5
    return {
        "header": {"stamp": {"sec": sec, "nanosec": nsec}, "frame_id": frame_id},
        "ns": ns, "id": iota(), "type": 1, "action": 0,
        "pose": {"position": {"x": float(x), "y": float(y), "z": 0.0},
                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "scale": {"x": CAR_LEN, "y": CAR_WID, "z": CAR_HGT},
        "color": {"r": r, "g": g, "b": b, "a": a},
        "lifetime": 0, "frame_locked": True,
        "text": text, "points": [], "colors": [],
    }

# --------------------------- 主流程 ---------------------------
def main():
    ap = argparse.ArgumentParser("make_full_merged.py")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--ref-mcap", required=True)
    ap.add_argument("--ref-topic", default="/roadusers")
    ap.add_argument("--time-col", default="Time")
    ap.add_argument("--time-mode", default="relative",
                    choices=["relative","unix_s","unix_ms","unix_ns","gps_s"])
    ap.add_argument("--time-unit", default="s", choices=["s","ms","us","ns"])
    ap.add_argument("--auto-align", action="store_true")
    ap.add_argument("--x-col", default="IVS1")
    ap.add_argument("--y-col", default=None)
    ap.add_argument("--frame-id", default="ego_vehicle")
    ap.add_argument("--tolerance-s", type=float, default=0.6)
    ap.add_argument("--skiprows", type=int, default=5)
    ap.add_argument("--sep", default="auto")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    ref_mcap = abspath(args.ref_mcap)
    csv_path = abspath(args.csv)
    final_out = abspath(args.output)
    ref_copy_path = final_out.replace(".mcap", "_ref_copy_all.mcap")
    ref_plus_path = final_out.replace(".mcap", "_ref_plus_merged.mcap")

    print(f"[INFO] ref_mcap: {ref_mcap}")
    print(f"[INFO] csv     : {csv_path}")
    print(f"[INFO] out(final): {final_out}")

    # ---------- 阶段1：原样复制所有话题（schema/channel/message/metadata/attachments） ----------
    with open(ref_mcap, "rb") as rf, open(ref_copy_path, "wb") as wf:
        reader = make_reader(rf)
        writer = CoreWriter(wf)
        writer.start()  # ★ 必须先 start  [1](https://mcap.dev/docs/python/raw_reader_writer_example)

        schema_id_map: Dict[int, int] = {}
        channel_id_map: Dict[int, int] = {}

        summary = reader.get_summary()

        def ensure_schema(schema):
            if schema is None: return None
            if schema.id in schema_id_map: return schema_id_map[schema.id]
            new_id = writer.register_schema(
                name=schema.name, encoding=schema.encoding, data=schema.data)  # ★ register_schema  [1](https://mcap.dev/docs/python/raw_reader_writer_example)
            schema_id_map[schema.id] = new_id
            return new_id

        def ensure_channel(channel, schema_id):
            if channel.id in channel_id_map: return channel_id_map[channel.id]
            new_cid = writer.register_channel(                                   # ★ register_channel  [1](https://mcap.dev/docs/python/raw_reader_writer_example)
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_id if schema_id is not None else 0,
                metadata=channel.metadata or {},
            )
            channel_id_map[channel.id] = new_cid
            return new_cid

        if summary:
            for sch in summary.schemas.values(): ensure_schema(sch)
            for ch in summary.channels.values():
                sid = schema_id_map.get(ch.schema_id)
                ensure_channel(ch, sid)

        for schema, channel, message in reader.iter_messages():
            sid = ensure_schema(schema)
            cid = ensure_channel(channel, sid)
            writer.add_message(                                                   # ★ add_message  [2](https://mcap.dev/docs/python/mcap-apidoc/mcap.writer)
                channel_id=cid,
                log_time=message.log_time,
                publish_time=message.publish_time,
                data=message.data,
            )
        for meta in reader.iter_metadata():
            writer.add_metadata(meta.name, meta.metadata)
        for att in reader.iter_attachments():
            writer.add_attachment(att.create_time, att.log_time, att.name, att.media_type, att.data)

        writer.finish()

    print(f"[OK] 参考所有话题复制到: {ref_copy_path}  ({os.path.getsize(ref_copy_path)} bytes)")

    # ---------- 收集基准时间轴（只读 ref-topic） ----------
    base_frames: List[Tuple[int, Dict[str, Any]]] = []
    base_times: List[int] = []
    with open(ref_mcap, "rb") as rf:
        rdr = make_reader(rf, decoder_factories=[DecoderFactory()])
        for sch, ch, msg, ros in rdr.iter_decoded_messages(topics=[args.ref_topic]):
            base_frames.append((msg.log_time, markerarray_to_dict(ros)))
            base_times.append(msg.log_time)
    if not base_frames:
        raise RuntimeError(f"参考文件中无基准话题 {args.ref_topic}")

    # ---------- 阶段2：生成 /roadusers_merged ----------
    def to_ns(v):
        if pd.isna(v): return None
        x = float(v)
        if args.time_mode == "relative":
            factor = {"s":1e9, "ms":1e6, "us":1e3, "ns":1.0}[args.time_unit]
            return int(x * factor)
        if args.time_mode == "unix_ns": return int(x)
        if args.time_mode == "unix_ms": return int(x*1e6)
        if args.time_mode == "unix_s":  return int(x*1e9)
        if args.time_mode == "gps_s":
            import datetime
            LEAP_SECONDS = 18
            GPS_WEEK_OFFSET = (datetime.date(2020,10,27).toordinal() - datetime.date(1980,1,6).toordinal())//7
            GPS_WEEK_SECONDS = GPS_WEEK_OFFSET * 7 * 24 * 3600
            UNIX0 = GPS_WEEK_SECONDS - LEAP_SECONDS
            return int((UNIX0 + x) * 1e9)
        return None

    df = pd.read_csv(csv_path, skiprows=args.skiprows,
                     engine="python" if args.sep=="auto" else "c",
                     sep=None if args.sep=="auto" else args.sep)
    assert args.time_col in df.columns, f"CSV缺少时间列 {args.time_col}"
    assert args.x_col    in df.columns, f"CSV缺少前向列 {args.x_col}"

    t_ns = df[args.time_col].apply(to_ns).astype("Int64")
    x = pd.to_numeric(df[args.x_col], errors="coerce")
    y = pd.to_numeric(df[args.y_col], errors="coerce") if (args.y_col and args.y_col in df.columns) else pd.Series(0.0, index=df.index)

    base_times_arr = np.array(base_times, dtype=np.int64)
    ref_start = int(base_times_arr[0])
    if args.auto_align:
        csv_first_valid = t_ns.dropna()
        if not csv_first_valid.empty:
            csv_start = int(csv_first_valid.iloc[0])
            offset_ns = ref_start - csv_start
            t_ns = t_ns + offset_ns
            print(f"[INFO] auto-align: ref_start_ns={ref_start}, csv_start_ns={csv_start}, offset_ns={offset_ns}")

    csv_df = pd.DataFrame({"t_ns": t_ns, "x": x, "y": y}, copy=False)
    csv_df = csv_df.dropna(subset=["t_ns"]).astype({"t_ns":"int64"}).sort_values("t_ns").reset_index(drop=True)

    tol_ns = int(max(0.0, args.tolerance_s) * 1e9)
    ref_df = pd.DataFrame({"t_ns": base_times_arr})
    merged_nn = pd.merge_asof(ref_df, csv_df[["t_ns","x","y"]],
                              on="t_ns", direction="nearest", tolerance=tol_ns)

    nn_ok = 0
    with open(ref_plus_path, "wb") as wf2:
        w2 = Ros2Writer(wf2)  # 官方 ROS2 写法：register_msgdef + write_message  [3](https://github.com/foxglove/mcap/blob/main/python/examples/ros2-noenv/writer.py)
        schema2 = w2.register_msgdef(MARKER_ARRAY_SCHEMA_NAME, MARKER_ARRAY_SCHEMA_TEXT)
        for (t_ref_ns, ma_ref), row in zip(base_frames, merged_nn.itertuples(index=False)):
            ma_out = copy.deepcopy(ma_ref)
            if "markers" not in ma_out or not isinstance(ma_out["markers"], list):
                ma_out["markers"] = []
            if not (pd.isna(row.x) or pd.isna(row.y)):
                ma_out["markers"].append(
                    make_csv_car_marker(t_ref_ns, args.frame_id, float(row.x), float(row.y),
                                        rgba=(0.0, 0.9, 0.2, 0.95), text="CSV Car", ns="csv_car"))
                nn_ok += 1
            w2.write_message(topic="/roadusers_merged", schema=schema2, message=ma_out,
                             log_time=int(t_ref_ns), publish_time=int(t_ref_ns))
        w2.finish()
    print(f"[OK] /roadusers_merged 写入: {ref_plus_path}  ({os.path.getsize(ref_plus_path)} bytes), 最近邻成功 {nn_ok} 帧")

    # ---------- 阶段3：CLI merge（先 --output，失败则 -o；并校验结果存在性） ----------
    def run_merge(cmd):
        print("[INFO]", " ".join(cmd))
        return subprocess.run(cmd, capture_output=True, text=True)

    p = run_merge(["mcap", "merge", "--output", final_out, ref_copy_path, ref_plus_path])
    if p.returncode != 0:
        print("[WARN] mcap merge --output 失败：", p.stderr.strip())

    if not (os.path.exists(final_out) and os.path.getsize(final_out) > 0):
        q = run_merge(["mcap", "merge", "-o", final_out, ref_copy_path, ref_plus_path])
        if q.returncode != 0:
            print("[ERROR] mcap merge -o 也失败：", q.stderr.strip())

    if os.path.exists(final_out) and os.path.getsize(final_out) > 0:
        print(f"✅ 最终文件生成：{final_out}  ({os.path.getsize(final_out)} bytes)")
        print(f"建议：mcap info {final_out} 查看 channels 是否包含原始全部话题 + /roadusers_merged")
    else:
        print("[FATAL] merged_all 未生成。请手动检查：")
        print("  1) which mcap && mcap --version")
        print(f"  2) ls -lh '{ref_copy_path}' '{ref_plus_path}'")
        print(f"  3) 手动合并：mcap merge --output '{final_out}' '{ref_copy_path}' '{ref_plus_path}'")
        print("     或：mcap merge -o '{final_out}' '{ref_copy_path}' '{ref_plus_path}'")

if __name__ == "__main__":
    main()