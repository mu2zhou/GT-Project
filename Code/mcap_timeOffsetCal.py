'''
先确认两个文件的时间概念


output_wukong.mcap
start: 401.455183761
→ 这是一个相对时间（秒），不是 Unix epoch，而是文件内部的 log_time 基准（可能从 0 或某个 offset 开始）。


part3_ros2.mcap
start: (1287674469.100000000)
→ 这是 Unix epoch 秒（2010-10-21T23:21:09.1+08:00），因为它是 GPS 转换来的真实时间戳。


Foxglove 播放时，log_time 是排序依据。如果两个文件的 log_time 差距太大（这里差距是 1287674469.1 - 401.455 ≈ 1287674067.645 秒 ≈ 40.8 年），就会触发“超过 90 天”错误。

频率不同

output_wukong.mcap：每个 topic ~1.47 Hz（约 0.68 秒一帧）。
part3_ros2.mcap：/roadusers 10 Hz（约 0.1 秒一帧）。
→ 两者帧率差异很大，Foxglove 播放时不会自动插值，只按时间戳显示。

时间戳分布不一致
即使起点对齐，后续每条消息的时间戳仍然不同步。例如：

文件 A：401.455 → 402.135 → 402.815...
文件 B：401.455 → 401.555 → 401.655...
→ 播放时，Foxglove会在 401.455 显示两者，但下一帧就错开。

Foxglove的同步逻辑
它不会“锁帧”，而是按时间轴连续播放。如果两个文件的消息时间戳不完全一致，就不会同帧显示。
'''



import subprocess, re, sys

def get_start_seconds(path: str) -> float:
    p = subprocess.run(["mcap", "info", path], capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stderr, file=sys.stderr); sys.exit(2)
    start_line = next((ln for ln in p.stdout.splitlines() if ln.strip().startswith("start:")), None)
    if not start_line:
        print(f"[ERROR] no 'start:' in mcap info for {path}", file=sys.stderr); sys.exit(3)
    # 先匹配括号里的 epoch 秒 (xxx.xxxxxxxxx)，不行再匹配裸秒
    m = re.search(r"\(([0-9]+\.[0-9]+)\)", start_line) or re.search(r"start:\s*([0-9]+\.[0-9]+)", start_line)
    if not m:
        print(f"[ERROR] cannot parse start line: {start_line}", file=sys.stderr); sys.exit(4)
    return float(m.group(1))


TARGET="/home/jzhou32/workspace_jz/017_UltraAV/Filed-Experiment-Data-ULTra-AV/Dataset/OpenACC/data/Casale/part3_ros2.mcap"
MINE="./Dataset/OpenACC/data/Casale/output_wukong.mcap"


ts = get_start_seconds(TARGET)
ms = get_start_seconds(MINE)
print(ts - ms)


'''
python csv2mcap_2car_ros2.py \
  ./Dataset/OpenACC/data/Casale/part3.csv \
  --time-mode gps_s \
  --ivs1-col IVS1 --ivs2-col IVS2 \
  --time-offset-s -1287674067.644816239 \
  -o ./Dataset/OpenACC/data/Casale/part3_ros2_aligned.mcap
'''