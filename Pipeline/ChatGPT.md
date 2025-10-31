基于你上传的 part3.csv（10 Hz） 和要求，我已经把整条计算流水线打通并在你这份数据上跑通了：它会把真值文件转成评估所需的字段，然后计算与出图 APE / ASE / DEE / OOCA / IDCR（其中 OOCA 与 IDCR 在当前数据下按你的要求用占位做法先跑通）。所有出图与表格都按《ADB DADC Evaluation Metric and Standard》里的规范来做：100 m距离迭代、x 轴每 10 m 参考线、散点图指数拟合、数据表保留位数等。 [part3]

你现在已经具备的可执行工程
目录结构（已为你生成并执行一次）：
pipeline/
├─ adb_dadc_metrics_pipeline.py        # 指标计算主脚本（含出图）
├─ prepare_from_part3.py               # 将 part3.csv 转成评估所需字段的预处理脚本
├─ compute_ooca_per_class.py           # 生成 “vehicle + fake motorcycle” 的OOCA按类汇总
├─ schema_config.json                  # 列名映射（已与预处理输出对齐，可后续替换）
├─ inputs/
│  └─ part3_prepared.csv               # 从 part3.csv 预处理得到的输入（已生成）
└─ outputs/                            # 运行结果（已生成）
   ├─ per_sample_metrics.csv           # 逐样本数据表（按规范保留小数位）
   ├─ summary_by_bin.csv               # 100m 分箱汇总（含样本数）
   ├─ scalar_metrics.txt               # OOCA、IDCR标量结果
   ├─ ooca_per_class.csv               # OOCA按类别（vehicle 与 fake motorcycle）
   ├─ ape_deg_bars.png                 # 各指标柱图（含10%参考线）
   ├─ ase_deg_per_s_bars.png
   ├─ dee_m_bars.png
   ├─ ape_deg_scatter.png              # 各指标散点（指数拟合）
   ├─ ase_deg_per_s_scatter.png
   └─ dee_m_scatter.png


我们对你提供的文件做了如下预处理：

从首部定位到 Time, Speed1, Lat1, Lon1, E1, N1, ... E2, N2 ... 的数据表头；
使用 E1/N1（自车）与 E2/N2（目标车）计算目标相对距离与相对方位角；样本的时间戳直接使用 Time 列（10 Hz，Δt≈0.1 s）；
由中心方位角 + 车辆外形假设（默认：宽 2.0 m、高 1.5 m，可后续配置）推得四边缘角（left/right/top/bottom），再用时间差分求四边缘角速度；
OOCA（oncoming/preceding）：用径向速度符号判定（range-rate < 0 → oncoming；≥0 → preceding）；
ID：按你的要求先给定占位 ID（object_track_id_gt=1、object_id_pred=1）；
预测列当前等于真值列（占位），以便先完整打通指标计算的 pipeline。 [part3]



计算口径与文档一致（要点）

APE（角位置误差）：四边缘角（left/right/top/bottom）的绝对误差后平均（单位：deg）。
ASE（角速度误差）：四边缘角速度的绝对误差后平均（单位：deg/s）。
DEE（距离估计误差）：预测距离与真值距离的绝对差（单位：m）。
OOCA（朝向分类准确率）：orientation_pred 与 orientation_gt 的逐样本一致率（%）。
IDCR（ID变化率）：测试时长内，按物理目标轨迹统计预测 ID 的切换次数并归一化到 id/s。
展示规范：100 m 分箱统计、柱图 x 轴每 10 m 参考线、散点指数拟合、表格数值保留位数（APE 三位小数；其余一位小数）。 [part3]


参考线统一设置为“10%”

根据你的要求，柱状图中为每个指标加入了10%参考线。当前实现为：取该指标在各分箱均值的最大值的 10% 作为参考线（若全为 0，则固定显示 0.1 以便可视）。后续你给出正式目标阈值后，我可替换为固定数值参考线。 [part3]


本次数据跑通后的结果小结（用于确认流程）

数据范围：共 2530 帧，距离 13.07–73.11 m，因此全部样本都落在 0–100 m 分箱内。 [part3]


APE / ASE / DEE：由于当前“预测=真值”的占位设定，误差曲线为 0，柱图也呈 0 水平线（用于验证流程正确性）。
OOCA（总体）：100.0%（逐样本 pred==gt）
IDCR：0.0 id/s（占位 ID 不发生切换）
OOCA 按类（vehicle + fake motorcycle）：已在 ooca_per_class.csv 中输出，二者均 100.0%（本次仅为演示“fake motorcycle”流程）。


提示：等你提供真实的预测输出（或我基于你的预测接口接入在线结果）后，APE/ASE/DEE 就会呈现真实的非零曲线/散点，IDCR 也会体现跟踪稳定性。此时 10% 参考线也可以替换为功能需求中的明确阈值或性能目标。 [part3]


如何在你本地复现 / 二次运行


准备输入（已完成）

你的 part3.csv 已放在当前目录，文件首部包含 Time,E1,N1,E2,N2... 等列。 [part3]



一步到位运行（会自动覆盖旧输出）


Shell# 预处理（把 part3.csv → pipeline/inputs/part3_prepared.csv）python pipeline/prepare_from_part3.py# 计算指标 + 出图（含10%参考线）python -c "from pathlib import Path; import importlib.util, sys; \p='pipeline/adb_dadc_metrics_pipeline.py'; \spec=importlib.util.spec_from_file_location('adb', p); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m); \print(m.run_pipeline(Path('pipeline/inputs/part3_prepared.csv')))"显示更多行

生成“vehicle + fake motorcycle”的按类 OOCA

Shellpython pipeline/compute_ooca_per_class.py显示更多行

产物会统一落在：pipeline/outputs/ 目录（表格 + PNG 图）


你关心的几个定制点（都已经可配）

帧率 10 Hz：已直接使用 Time 列（Δt≈0.1 s）；若需要强制重采样到严格 10 Hz，可在预处理阶段加入重采样。 [part3]
车辆几何尺寸（用于由中心角推四边缘角）：当前默认 宽 2.0 m / 高 1.5 m，可在预处理脚本里改成你们车型或按目标类型（如 motorcycle）配置不同数值。
OOCA 标签：按径向速度符号自动给 oncoming/preceding；你也可以后期上传人工标注列，直接替换 orientation_gt，pipeline 会据此重算准确率。
ID 策略：当前按你的要求给了占位 ID（恒为 1），所以本次 IDCR = 0。若你希望模拟某个 ID 切换率以验证报表效果，我可以加入受控的 ID 切换发生器（例如按 0.05 id/s 随机切换），等你一句话即可开启。
10% 参考线：临时用于对齐你“先设为 10%”的需求；待有正式目标后可改为固定阈值线（如 APE ≤ 0.3°、DEE ≤ 5 m 等），并在柱图/表格中标注超限统计。 [part3]


接下来我建议这样迭代

把真实预测结果 接上来：

方案 A：你给我一个预测结果 CSV，我来把 schema_config.json 的映射对齐（无需改代码）；
方案 B：告诉我预测结果里有哪些列名（四边缘角/角速度/距离/ID/朝向），我直接改好 schema_config.json 并跑一版。


给出各指标的目标阈值（替换临时 10% 参考线），我会更新柱图中的红色虚线；
如需按对象类型（vehicle/motorcycle/…）分别统计：告诉我你的类型字段/口径，我把汇总与图表拆分到各类型并按文档中不同有效距离覆盖范围进行迭代（例如行人/自行车与前/对向车辆的覆盖距离不同）。 [part3]


如果你愿意，我现在也可以把“motorcycle”的几何尺寸（比如 W=0.8 m、H=1.2 m）加进去，产生一份对照版四边缘角与 ASE/APE 的“fake motorcycle”曲线；或者先等你给到正式尺寸与预测列名我再改。跑步训练之余，把这条流水线交给我，我帮你盯 😊

附注：

预处理时识别到 Time,E1,N1,E2,N2 等字段，说明你这份 CSV 自带两车的真值轨迹（含米级东/北坐标），这也正好满足我们构造相对角度/距离/朝向所需的最小信息。 [part3]
指标的定义、展示与精度保留位等均严格参照你上传的 ADB DADC 评估规范文档执行。 [part3]