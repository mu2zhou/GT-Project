场景速读 – part3.csv 里到底发生了什么？
Table
Copy
字段	含义（根据列名+数值推断）
Date	2020-10-27，实验日期
Vehicle_order	Rexton（车0，leader）、Hyundai（车1，follower）
ACC	1 → 全程自适应巡航开启
Distance_setting	min → 跟车时距设为“近”
Time	从 0 到 1800 s（30 min）连续
Speed1 / Speed2	0 → 25 m/s（≈90 km/h）逐渐提速；两车速度曲线几乎重合
Lat1 / Lat2	同步缓慢减小（≈0.79882 → 0.79863 °）
Lon1 / Lon2	同步缓慢增大（≈0.15147 → 0.15146 °）
E1/N1、E2/N2	东北向局部坐标，横坐标 E 持续增大，纵坐标 N 持续减小 → 车队整体向 东南 方向行驶
IVS1	7.5 → 67 m，车间距逐渐拉大
Driver2	全程 “ACC” → 后车完全由 ACC 控制
结论 – 两车的相对运动
并非“侧向漂移”
纬度每秒只变化 ~1×10⁻⁷ °（≈1 cm），看起来“横着走”是因为：
道路近似东西走向，纬度方向分辨率远高于经度方向；
Foxglove 默认 ENU 坐标系未旋转，你把整条轨迹画在 rviz 里就像横向滑移。
真实行为
Rexton 领航，匀速拉到 25 m/s 后保持；
Hyundai 跟驰，ACC 按“min 时距”自动调节，因此 IVS 随车速升高而增大；
横向两车保持几乎相同 y（车道内），纵向间距 7 → 67 m 拉开。