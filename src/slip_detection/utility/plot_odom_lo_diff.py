import plotly.graph_objects as go
import re

def parse_data(filename):
    timestamps = []
    lo_linear_diffs = []
    lo_angle_diffs = []
    odom_linear_diffs = []
    odom_angle_diffs = []
    with open(filename, 'r') as file:
        lines = file.readlines()

    odom_pattern = r"odom_diff between \d+\.\d+ and (\d+\.\d+), odom:\[linear:(\d+\.\d+)\], \[angle:(\d+\.\d+)\]"
    lo_pattern = r"lo_diff between \d+\.\d+ and (\d+\.\d+), lo:\[linear:(\d+\.\d+)\], \[angle:(\d+\.\d+)\]"

    for i in range(0, len(lines), 2):
        if i + 1 >= len(lines):
            break
        odom_line = lines[i]
        lo_line = lines[i + 1]

        

        if re.search(odom_pattern, odom_line) and re.search(lo_pattern, lo_line):
            odom_match = re.search(odom_pattern, odom_line)
            lo_match = re.search(lo_pattern, lo_line)
            if odom_match and lo_match:
                timestamp = float(odom_match.group(1)) % 10000  # Adjusted for better x-axis representation
                timestamps.append(timestamp)
                odom_linear_diffs.append(float(odom_match.group(2)))
                odom_angle_diffs.append(float(odom_match.group(3)))
                lo_linear_diffs.append(float(lo_match.group(2)))
                lo_angle_diffs.append(float(lo_match.group(3)))

    return timestamps, lo_linear_diffs, lo_angle_diffs, odom_linear_diffs, odom_angle_diffs

def plot_data(timestamps, lo_linear_diffs, lo_angle_diffs, odom_linear_diffs, odom_angle_diffs):
    # Plotting Linear Differences
    linear_fig = go.Figure()
    linear_fig.add_trace(go.Scatter(x=timestamps, y=lo_linear_diffs, mode='lines+markers', name='LO Linear Difference', hoverinfo='x+y'))
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
    angle_fig.add_trace(go.Scatter(x=timestamps, y=lo_angle_diffs, mode='lines+markers', name='LO Angle Difference', hoverinfo='x+y'))
    angle_fig.add_trace(go.Scatter(x=timestamps, y=odom_angle_diffs, mode='lines+markers', name='Odom Angle Difference', hoverinfo='x+y'))
    angle_fig.update_layout(
        title='Angle Differences Over Time',
        xaxis_title='Timestamp',
        yaxis_title='Angle Difference',
        legend_title='Data Type'
    )
    angle_fig.show()




# filename = '/home/gechai/ros2_ws/recorded_bag/30.09/slip_1.7cm/diff.txt'
# filename = '/home/gechai/ros2_ws/recorded_bag/30.09/slip_narrow/diff.txt'
# filename = '/home/gechai/ros2_ws/recorded_bag/30.09/slope_noslipe/diff.txt'
# filename = '/home/gechai/ros2_ws/recorded_bag/30.09/chair_slipe/diff.txt'
# filename = '/home/gechai/ros2_ws/recorded_bag/30.09/slope_noslipe_15/diff.txt'
filename = '/home/gechai/ros2_ws/recorded_bag/30.09/slope_noslipe_10/diff.txt'

timestamps, lo_linear_diffs, lo_angle_diffs, odom_linear_diffs, odom_angle_diffs = parse_data(filename)
plot_data(timestamps, lo_linear_diffs, lo_angle_diffs, odom_linear_diffs, odom_angle_diffs)
