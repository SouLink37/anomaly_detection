

# import matplotlib.pyplot as plt
# import re

# def parse_data(filename):
#     # 初始化存储数据的列表
#     timestamps = []
#     imu_linear_diffs = []
#     imu_angle_diffs = []
#     odom_linear_diffs = []
#     odom_angle_diffs = []

#     # 读取文件并处理每一行
#     with open(filename, 'r') as file:
#         lines = file.readlines()

#     # 数据正则表达式
#     odom_pattern = r"odom_diff between \d+\.\d+ and (\d+\.\d+), odom:\[linear:(\d+\.\d+)\], \[angle:(\d+\.\d+)\]"
#     imu_pattern = r"imu_diff between \d+\.\d+ and (\d+\.\d+), imu:\[linear:(\d+\.\d+)\], \[angle:(\d+\.\d+)\]"

#     # 确保我们在处理两行一组的数据
#     for i in range(0, len(lines), 2):
#         if i+1 >= len(lines):
#             break  # 避免越界
#         odom_line = lines[i]
#         imu_line = lines[i+1]

#         odom_match = re.search(odom_pattern, odom_line)
#         imu_match = re.search(imu_pattern, imu_line)

#         if odom_match and imu_match:
#             # 使用Odom的结束时间戳作为两者的共同时间戳
#             timestamp = float(odom_match.group(1))
#             timestamps.append(timestamp)

#             # 存储Odom数据
#             odom_linear_diffs.append(float(odom_match.group(2)))
#             odom_angle_diffs.append(float(odom_match.group(3)))

#             # 存储IMU数据
#             imu_linear_diffs.append(float(imu_match.group(2)))
#             imu_angle_diffs.append(float(imu_match.group(3)))

#     return timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs

# def plot_data(timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs):
#     plt.figure(figsize=(12, 8))

#     # 绘制线性差异对比图
#     plt.subplot(2, 1, 1)
#     plt.plot(timestamps, imu_linear_diffs, 'r-', label='IMU Linear Difference')
#     plt.plot(timestamps, odom_linear_diffs, 'b-', label='Odom Linear Difference')
#     plt.xlabel('Timestamp')
#     plt.ylabel('Linear Difference')
#     plt.title('Linear Differences Over Time')
#     plt.legend()

#     # 绘制角度差异对比图
#     plt.subplot(2, 1, 2)
#     plt.plot(timestamps, imu_angle_diffs, 'r-', label='IMU Angle Difference')
#     plt.plot(timestamps, odom_angle_diffs, 'b-', label='Odom Angle Difference')
#     plt.xlabel('Timestamp')
#     plt.ylabel('Angle Difference')
#     plt.title('Angle Differences Over Time')
#     plt.legend()

#     plt.tight_layout()
#     plt.show()

# # 指定数据文件路径
# # filename = '/home/gechai/下载/26.09.24/log/self-0926-092305/diff_1.txt'
# filename = '/home/gechai/下载/26.09.24/log/self-0926-092639/diff.txt'
# timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs = parse_data(filename)
# plot_data(timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs)




import plotly.graph_objects as go
import re

def parse_data(filename):
    timestamps = []
    imu_linear_diffs = []
    imu_angle_diffs = []
    odom_linear_diffs = []
    odom_angle_diffs = []
    with open(filename, 'r') as file:
        lines = file.readlines()

    odom_pattern = r"odom_diff between \d+\.\d+ and (\d+\.\d+), odom:\[linear:(\d+\.\d+)\], \[angle:(\d+\.\d+)\]"
    imu_pattern = r"imu_diff between \d+\.\d+ and (\d+\.\d+), imu:\[linear:(\d+\.\d+)\], \[angle:(\d+\.\d+)\]"

    for i in range(0, len(lines), 2):
        if i+1 >= len(lines):
            break
        odom_line = lines[i]
        imu_line = lines[i+1]
        odom_match = re.search(odom_pattern, odom_line)
        imu_match = re.search(imu_pattern, imu_line)
        if odom_match and imu_match:
            timestamp = float(odom_match.group(1)) % 10000
            timestamps.append(timestamp)
            odom_linear_diffs.append(float(odom_match.group(2)))
            odom_angle_diffs.append(float(odom_match.group(3)))
            imu_linear_diffs.append(float(imu_match.group(2)))
            imu_angle_diffs.append(float(imu_match.group(3)))

    return timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs

def plot_data(timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs):
    # Plotting Linear Differences
    linear_fig = go.Figure()
    linear_fig.add_trace(go.Scatter(x=timestamps, y=imu_linear_diffs, mode='lines+markers', name='IMU Linear Difference', hoverinfo='x+y'))
    linear_fig.add_trace(go.Scatter(x=timestamps, y=odom_linear_diffs, mode='lines+markers', name='Odom Linear Difference', hoverinfo='x+y'))
    linear_fig.update_layout(
        title='Linear Differences Over Time',
        xaxis_title='Timestamp',
        yaxis_title='Linear Difference',
        legend_title='Data Type'
    )
    linear_fig.show()

    # Plotting Angle Differences
    angle_fig = go.Figure()
    angle_fig.add_trace(go.Scatter(x=timestamps, y=imu_angle_diffs, mode='lines+markers', name='IMU Angle Difference', hoverinfo='x+y'))
    angle_fig.add_trace(go.Scatter(x=timestamps, y=odom_angle_diffs, mode='lines+markers', name='Odom Angle Difference', hoverinfo='x+y'))
    angle_fig.update_layout(
        title='Angle Differences Over Time',
        xaxis_title='Timestamp',
        yaxis_title='Angle Difference',
        legend_title='Data Type'
    )
    angle_fig.show()

# filename = '/home/gechai/下载/26.09.24/log/self-0926-092305/diff_1.txt'
# filename = '/home/gechai/下载/26.09.24/log/self-0926-092639/diff.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.6cm/self/self-0926-150937/diff.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.7cm/self/self-0926-150655/diff.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/巴塞罗纳椅/机器人平地上椅子腿---触发了前碰撞/self/self-0926-161233/diff.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---窄道/self/self-0926-153354/diff.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---后退/self/self-0926-150029/diff.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/1.7cm/self/self-0926-190323/diff.txt'

# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.6cm/self/self-0926-150937/diff_1.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.6cm/self/self-0926-150937/diff_2.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.6cm/self/self-0926-150937/diff_3.txt'
# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.6cm/self/self-0926-150937/diff_4.txt'

# filename = '/home/gechai/下载/26.09.24/log/self-0926-092305/diff_2.txt'
# filename = '/home/gechai/下载/26.09.24/log/self-0926-092305/diff_3.txt'

# filename = '/home/gechai/下载/26.09.24/打滑数据采集/打滑---圆管/1.7cm/self/self-0926-150655/diff_1.txt'

# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff_1.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff_2.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff_3.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff_4.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff_5.txt'
filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/diff_6.txt'

# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/斜坡---没有触发打滑/8度---没有触发打滑---slip刚刚开始上斜坡时录制，在斜坡上结束slip录制/self/self-0926-192620/diff.txt'

# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/斜坡---没有触发打滑/15度---没有触发打滑---slip刚刚开始上斜坡时录制，在斜坡上结束slip录制/self/self-0926-192213/diff.txt'

# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/后退---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-185600/diff.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/后退---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-185600/diff_1.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/后退---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-185600/diff_2.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/后退---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-185600/diff_3.txt'








timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs = parse_data(filename)
plot_data(timestamps, imu_linear_diffs, imu_angle_diffs, odom_linear_diffs, odom_angle_diffs)




