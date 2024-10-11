import re
import plotly.graph_objects as go

def parse_new_data_from_file(filename):
    timestamps = []
    imu_values = []
    sensor_values = []

    # 正则表达式，用于匹配时间戳、IMU 和传感器的值
    pattern = r"acceleration_x at (\d+\.\d+), imu:([+-]?\d+\.\d+), sensor([+-]?\d+\.\d+)"

    # 从文件读取数据
    with open(filename, 'r') as file:
        lines = file.readlines()

    for line in lines:
        match = re.search(pattern, line)
        if match:
            # timestamp = float(match.group(1)) % 100000  # 用时间戳的最后几位表示，用于x轴
            timestamp = float(match.group(1)) % 10000  # 用时间戳的最后几位表示，用于x轴
            imu_value = float(match.group(2))
            sensor_value = float(match.group(3))

            timestamps.append(timestamp)
            imu_values.append(imu_value)
            sensor_values.append(sensor_value)

    return timestamps, imu_values, sensor_values

def plot_combined_data(timestamps, imu_values, sensor_values):
    # 创建图表对象
    fig = go.Figure()

    # 添加IMU数据曲线
    fig.add_trace(go.Scatter(x=timestamps, y=imu_values, mode='lines+markers', name='IMU Data', hoverinfo='x+y'))

    # 添加传感器数据曲线
    fig.add_trace(go.Scatter(x=timestamps, y=sensor_values, mode='lines+markers', name='Sensor Data', hoverinfo='x+y'))

    # 更新图表的布局
    fig.update_layout(
        title='IMU and Sensor Data Over Time',
        xaxis_title='Timestamp',
        yaxis_title='Value',
        legend_title='Data Type'
    )

    # 显示图表
    fig.show()

# 文件路径
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/1.7cm/self/self-0926-190323/head_stamp/self-0926-190323_0.db3/acc.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/head_stamp/self-0926-190937_0.db3/acc_x.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/窄道触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-191248/header/self-0926-191248_0.db3/acc_x.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/斜坡---没有触发打滑/8度---没有触发打滑---slip刚刚开始上斜坡时录制，在斜坡上结束slip录制/self/self-0926-192620/header/self-0926-192620_0.db3/acc_x.txt'
# filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/巴塞罗纳椅---上去时没有触发打滑，下来时人为把机器往上推一点，触发了一点点打滑，在slip话题中间/机器人平地上椅子腿---触发了前碰撞/self/self-0926-191505/header/self-0926-191505_0.db3/acc_x.txt'
filename = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/巴塞罗纳椅---上去时没有触发打滑，下来时人为把机器往上推一点，触发了一点点打滑，在slip话题中间/机器人在椅子腿上下来---轮子悬空，发生了空转/self/self-0926-191833/header/self-0926-191833_0.db3/acc_x.txt'


# 从文件解析数据并生成图
timestamps, imu_values, sensor_values = parse_new_data_from_file(filename)
plot_combined_data(timestamps, imu_values, sensor_values)
