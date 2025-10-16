#!/usr/bin/env python3
"""
csv2mcap.py  :  OpenACC CSV → MCAP
               绝对无旋转：两车均画在 ego 系，
               纵向距离直接取 IVS1，横向=0，朝向=0
"""
import pandas as pd
import numpy as np
from pyproj import Transformer
from mcap_ros1.writer import Writer as McapWriter
from geometry_msgs.msg import Point, Vector3, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage
import rospy
import os
import sys

CSV_PATH  = sys.argv[1] if len(sys.argv) > 1 else 'part3.csv'
MCAP_PATH = os.path.splitext(CSV_PATH)[0] + '.mcap'

# ---------- 1. 读取 ----------
df = pd.read_csv(CSV_PATH, skiprows=5)

# ---------- 2. 坐标系（仅用于 TF） ----------
ref_lat, ref_lon = df['Lat1'].iloc[0], df['Lon1'].iloc[0]
transformer = Transformer.from_crs(4326, 4978, always_xy=True)
ox, oy, _ = transformer.transform(ref_lon, ref_lat, 0)

def enu(lat, lon):
    x, y, _ = transformer.transform(lon, lat, 0)
    return x - ox, y - oy

# ---------- 3. 写 MCAP ----------
with open(MCAP_PATH, 'wb') as f:
    writer = McapWriter(output=f)

    CAR_LEN, CAR_WID, CAR_HGT = 4.5, 1.8, 1.5
    colors = {0: (1.0, 0.0, 0.0, 0.8), 1: (0.0, 0.0, 1.0, 0.8)}

    # static map→world
    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.from_sec(df['Time'].iloc[0])
    static_tf.header.frame_id = 'map'
    static_tf.child_frame_id  = 'world'
    static_tf.transform.rotation.w = 1.0
    writer.write_message('/tf_static', TFMessage([static_tf]), static_tf.header.stamp.to_nsec())

    for t_idx, row in df.iterrows():
        t_ros = rospy.Time.from_sec(row['Time'])

        # ---- 3.1 两车：ego 系，纵向距离=IVS1，横向=0，朝向=0 ----
        ivs = row['IVS1']          # 车间距（m）
        ma = MarkerArray()

        # Rexton（红）在前方 ivs 米处
        m1 = Marker()
        m1.header.frame_id = 'ego'
        m1.header.stamp = t_ros
        m1.ns, m1.id, m1.type, m1.action = 'cars', 0, Marker.CUBE, Marker.ADD
        m1.pose.position.x = ivs
        m1.pose.position.y = 0.0
        m1.pose.position.z = 0.0
        m1.pose.orientation.w = 1.0   # 0°
        m1.scale = Vector3(CAR_LEN, CAR_WID, CAR_HGT)
        m1.color.r, m1.color.g, m1.color.b, m1.color.a = colors[0]
        ma.markers.append(m1)

        # 现代（蓝）固定在 ego 原点
        m2 = Marker()
        m2.header.frame_id = 'ego'
        m2.header.stamp = t_ros
        m2.ns, m2.id, m2.type, m2.action = 'cars', 1, Marker.CUBE, Marker.ADD
        m2.pose.position.x = m2.pose.position.y = m2.pose.position.z = 0.0
        m2.pose.orientation.w = 1.0
        m2.scale = Vector3(CAR_LEN, CAR_WID, CAR_HGT)
        m2.color.r, m2.color.g, m2.color.b, m2.color.a = colors[1]
        ma.markers.append(m2)
        writer.write_message('/cars_markers', ma, t_ros.to_nsec())

        # ---- 3.2 车头 50 m 标尺（ego 系） ----
        ruler = Marker()
        ruler.header.frame_id = 'ego'
        ruler.header.stamp = t_ros
        ruler.ns, ruler.id = 'ego_ruler', 0
        ruler.type, ruler.action = Marker.LINE_STRIP, Marker.ADD
        ruler.points = [Point(0, 0, 0), Point(50, 0, 0)]
        ruler.scale.x = 0.3
        ruler.color.r, ruler.color.g, ruler.color.b, ruler.color.a = 1, 0, 0, 1
        writer.write_message('/ego_ruler', ruler, t_ros.to_nsec())

        # ---- 3.3 heading 箭头（ego 系，固定方向） ----
        spd = row['Speed2']
        arrow = Marker()
        arrow.header.frame_id = 'ego'
        arrow.header.stamp = t_ros
        arrow.ns, arrow.id = 'ego_heading', 0
        arrow.type, arrow.action = Marker.ARROW, Marker.ADD
        arrow.pose.orientation.w = 1.0   # 始终指向前方
        arrow.scale = Vector3(spd * 2, 0.3, 0.3)
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0, 1, 0, 1
        writer.write_message('/ego_heading', arrow, t_ros.to_nsec())

        # ---- 3.4 TF: map → ego（Hyundai 真实轨迹，盒子在 ego 里不转） ----
        x2, y2 = enu(row['Lat2'], row['Lon2'])
        tf = TransformStamped()
        tf.header.stamp = t_ros
        tf.header.frame_id = 'map'
        tf.child_frame_id  = 'ego'
        tf.transform.translation.x = x2
        tf.transform.translation.y = y2
        tf.transform.translation.z = 0.0
        # 这里我们用平滑后的真实航向，但盒子在 ego 系内固定为 0°
        yaw = 0.0   # 如果还想再保险，可注释掉下面两行，直接 w=1
        tf.transform.rotation.z = np.sin(yaw / 2)
        tf.transform.rotation.w = np.cos(yaw / 2)
        writer.write_message('/tf', TFMessage([tf]), t_ros.to_nsec())

    writer.finish()

print(f'✅  MCAP written  ->  {MCAP_PATH}')