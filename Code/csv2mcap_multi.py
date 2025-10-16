#!/usr/bin/env python3
"""
csv2mcap_multi.py
OpenACC 多车 CSV → MCAP（Foxglove 可视化）
每辆车真实 GPS 位置 + 立方体模型 + IVS 文字
"""
import pandas as pd
import numpy as np
from pyproj import Transformer
from mcap_ros1.writer import Writer as McapWriter
from geometry_msgs.msg import Point, Vector3, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage
import rospy, os, sys

CSV_PATH  = sys.argv[1] if len(sys.argv) > 1 else 'JRC-VC_260219_part2.csv'
MCAP_PATH = os.path.splitext(CSV_PATH)[0] + '_multi.mcap'

# ---------------- 1. 读取 ----------------
df = pd.read_csv(CSV_PATH, skiprows=5)

# ---------------- 2. 坐标系 ----------------
ref_lat, ref_lon = df['Lat1'].iloc[0], df['Lon1'].iloc[0]
transformer = Transformer.from_crs(4326, 4978, always_xy=True)
ox, oy, _ = transformer.transform(ref_lon, ref_lat, 0)

def enu(lat, lon):
    x, y, _ = transformer.transform(lon, lat, 0)
    return x - ox, y - oy

# ---------------- 3. 参数 ----------------
CAR_LEN, CAR_WID, CAR_HGT = 4.5, 1.8, 1.5
RULER_FRONT, RULER_BACK  = 50.0, 10.0          # 前后标尺长度
colors = [
    (1.0, 0.0, 0.0, 0.9),   # 0 人工 红
    (0.0, 0.0, 1.0, 0.9),   # 1 ACC 蓝
    (0.0, 1.0, 0.0, 0.9),   # 2 ACC 绿
    (1.0, 0.6, 0.0, 0.9),   # 3 ACC 橙
    (0.6, 0.0, 1.0, 0.9),   # 4 ACC 紫
]
N_CARS = 5

# ---------------- 4. 写 MCAP ----------------
with open(MCAP_PATH, 'wb') as f:
    writer = McapWriter(output=f)

    # static map→world
    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.from_sec(df['Time'].iloc[0])
    static_tf.header.frame_id = 'map'
    static_tf.child_frame_id  = 'world'
    static_tf.transform.rotation.w = 1.0
    writer.write_message('/tf_static', TFMessage([static_tf]),
                         static_tf.header.stamp.to_nsec())

    for _, row in df.iterrows():
        t_ros = rospy.Time.from_sec(row['Time'])
        ma = MarkerArray()

        # ---- 4.1 每辆车立方体 + 文字 ----
        # 在循环里替换掉 4.1 节的 x,y 计算
        lat0, lon0 = row['Lat1'], row['Lon1']
        x0, y0 = enu(lat0, lon0)          # 头车真实 ENU

        for i in range(N_CARS):
            # ----- 纵向用 IVS 累加，横向用真实 GPS -----
            if i == 0:
                x, y = x0, y0
            else:
                ivs_sum = sum(row[f'IVS{j}'] for j in range(1, i+1))
                x = x0 + ivs_sum          # 沿 x 轴展开
                lat_i, lon_i = row[f'Lat{i+1}'], row[f'Lon{i+1}']
                _, y = enu(lat_i, lon_i)  # 保留真实横向

            # 立方体
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = t_ros
            m.ns, m.id = 'cars', i
            m.type, m.action = Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.0
            m.pose.orientation.w = 1.0
            m.scale = Vector3(CAR_LEN, CAR_WID, CAR_HGT)
            m.color.r, m.color.g, m.color.b, m.color.a = colors[i]
            ma.markers.append(m)

            # 速度箭头
            spd = row[f'Speed{i+1}']
            arr = Marker()
            arr.header.frame_id = 'map'
            arr.header.stamp = t_ros
            arr.ns, arr.id = 'vel_arrow', i
            arr.type, arr.action = Marker.ARROW, Marker.ADD
            arr.pose.position.x, arr.pose.position.y, arr.pose.position.z = x, y, 0.0
            arr.pose.orientation.w = 1.0
            arr.scale = Vector3(spd * 2, 0.3, 0.3)
            arr.color.r, arr.color.g, arr.color.b, arr.color.a = 0, 1, 0, 1
            ma.markers.append(arr)

        # ---- 4.2 IVS 文字 ----
        # ---------- inside the row loop, after writing the car cubes ----------
        for ivs_idx in range(1, 5):          # IVS1 … IVS4
            val = row[f'IVS{ivs_idx}']
            # front car of this pair
            lat_f = row[f'Lat{ivs_idx}']
            lon_f = row[f'Lon{ivs_idx}']
            xf, yf = enu(lat_f, lon_f)

            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp = t_ros
            txt.ns, txt.id = 'ivs_text', ivs_idx - 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = xf
            txt.pose.position.y = yf
            txt.pose.position.z = 2.5          # hover above the roof
            txt.scale.z = 1.3
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = colors[ivs_idx]  # same as FOLLOWER
            txt.text = f'IVS{ivs_idx}: {val:.1f} m'
            ma.markers.append(txt)

        writer.write_message('/cars_markers', ma, t_ros.to_nsec())

        # ---- 4.3 标尺（以头车为参考） ----
        x1, y1 = enu(row['Lat1'], row['Lon1'])
        ruler = Marker()
        ruler.header.frame_id = 'map'
        ruler.header.stamp = t_ros
        ruler.ns, ruler.id = 'ruler', 0
        ruler.type, ruler.action = Marker.LINE_STRIP, Marker.ADD
        ruler.points = [Point(x1 - RULER_BACK, y1, 0),
                        Point(x1 + RULER_FRONT, y1, 0)]
        ruler.scale.x = 0.3
        ruler.color.r, ruler.color.g, ruler.color.b, ruler.color.a = 1, 1, 0, 1
        writer.write_message('/ruler', ruler, t_ros.to_nsec())

    writer.finish()

print(f'✅  Multi-car MCAP ready  ->  {MCAP_PATH}')