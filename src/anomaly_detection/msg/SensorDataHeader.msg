std_msgs/Header header

#底盘系统时间
uint64 time_ses                                  #底盘开机后时间累计单位ses
uint64 time_nsec                                 #底盘开机后时间累计单位nsec

#沿边传感器
uint8 tof_data                   #离墙距离，单位mm

#回充座信号
uint8 station_signal_middle_left			    #中左接收
uint8 station_signal_middle_right			    #中右接收
uint8 station_signal_back_left			        #后左接收
uint8 station_signal_back_right			        #后右接收


#上报周期内的脉冲个数
int16 left_wheel_pulse                    #左轮脉冲数据
int16 right_wheel_pulse                   #右轮脉冲数据

#左右轮速度数据
float32 l_speed                                 #左轮速度值，单位m/s
float32 r_speed                                 #右轮速度值，单位m/s

#地检
# bit0:左地检
# bit1:中左地检(左前地检)
# bit2:中右地检(右前地检)
# bit3:右地检
# bit4:左后地检
# bit5:右后地检
uint8 floor_detect_status

#碰撞
# bit0: 前撞左碰撞
# bit1: 前撞右碰撞
# bit2: 雷达盖左
# bit3: 雷达盖右
# bit4: 雷达盖前
# bit5: 雷达盖顶
uint8 crash_data

#离地传感器
# bit0: 左轮离地
# bit1: 右轮离地
uint8 lift_off

#超声波材质传感器
# 0:地面 1:地毯
int8 material

# 错误信号
# uint64 error_signal
# bit:0 左侧驱动轮开路
# bit:1 左侧驱动轮短路
# bit:2 右侧驱动轮开路
# bit:3 右侧驱动轮短路
# bit:4 左侧拖布电机异常
# bit:5 右侧拖布电机异常
# bit:6 托盘堵转
# bit:7 滚刷电机开路
# bit:8 滚刷电机短路
# bit:9 边刷电机开路
# bit:10 边刷电机短路
# bit:11 集尘风机异常
# bit:12 蠕动泵开路
# bit:13 蠕动泵短路
# bit:14 悬崖传感器异常
# bit:15 前碰撞传感器异常
# bit:16 雷达盖碰撞传感器异常
# bit:17 超声波传感器异常-通信异常
uint64 error_signal

#接触充电座信号
# Bit0:是否接触充电座
# Bit1:是否是充电状态
# Bit2:是否充电完成
uint8 charge_status


#bit:0 左拖布电机，0-正常，1-缠绕
#bit:1 右拖布电机，0-正常，1-缠绕
#bit:2 尘道堵塞，0-正常，1-堵塞
uint8   worker_error

#超声波原始数据-类型
uint8 ultrasonic_type

#超声波原始数据-距离
uint8 ultrasonic_distance

#超声波原始数据-反射强度(参考值) 
uint32  ultrasonic_intensity0

#超声波原始数据-反射强度(设定值) 
uint32  ultrasonic_intensity1

#左拖布盘电流
uint16 mop_current

#左右轮电流数据
uint16 left_wheel_current
uint16 right_wheel_current

#右拖布盘电流
uint16 mop_current_right