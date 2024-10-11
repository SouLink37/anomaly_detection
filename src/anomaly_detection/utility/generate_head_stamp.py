
# import rosbag2_py
# import rclpy.serialization
# from anomaly_detection.msg import SensorData, SensorDataHeader  # 包含原始和新的消息类型
# from rosbag2_py._storage import TopicMetadata


# # 读取bag文件的元数据
# def get_bag_metadata(input_bag_path):
#     storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
#     converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#     info = rosbag2_py.Info()
#     metadata = info.read_metadata(input_bag_path, 'sqlite3')
    
#     # 使用 topic_metadata 而不是直接访问 name 属性
#     topic_metadata = {topic_info.topic_metadata.name: topic_info.topic_metadata for topic_info in metadata.topics_with_message_count}
    
#     return topic_metadata

# # 读取bag文件
# def read_bag(input_bag_path):
#     reader = rosbag2_py.SequentialReader()
#     storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
#     converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#     reader.open(storage_options, converter_options)

#     messages = []
#     while reader.has_next():
#         (topic, data, t) = reader.read_next()
#         messages.append((topic, data, t))
    
#     return messages

# # 将 time_ses 和 time_nsec 转换为 stamp，并包含在 header 中
# def convert_to_header(time_ses, time_nsec, frame_id="sensor_data"):
#     from std_msgs.msg import Header
#     from builtin_interfaces.msg import Time
#     # 创建一个 Header 消息
#     header = Header()
    
#     # 创建并设置 Time 对象
#     stamp = Time()
#     stamp.sec = int(time_ses)  # 确保 time_ses 是秒
#     stamp.nanosec = int(time_nsec)  # 确保 time_nsec 是纳秒
    
#     # 将 stamp 赋值给 header 的 stamp 字段
#     header.stamp = stamp
#     header.frame_id = frame_id  # 设置 frame_id
    
#     return header

# # 写入新的bag文件，并修改话题名称
# def write_bag(output_bag_path, messages, original_metadata):
#     writer = rosbag2_py.SequentialWriter()
#     storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
#     converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#     writer.open(storage_options, converter_options)

#     # 遍历消息，创建对应的话题元数据
#     topic_metadata_created = {}

#     for topic, data, timestamp in messages:
#         new_data = None  # 初始化 new_data
#         if topic == "/sensor_data":
#             print(f"Processing topic: {topic}")
#             # 反序列化数据为原始 SensorData 类型
#             original_data = rclpy.serialization.deserialize_message(data, SensorData)

#             # 打印原始消息中的时间数据，确保它们是有效的
#             print(f"Original time_ses: {original_data.time_ses}, time_nsec: {original_data.time_nsec}")

#             # 创建一个新的消息类型 SensorDataHeader
#             new_data = SensorDataHeader()
#             new_data.header = convert_to_header(original_data.time_ses, original_data.time_nsec)
#             new_data.time_ses = original_data.time_ses
#             new_data.time_nsec = original_data.time_nsec

#             # 确保 header 被正确赋值
#             print(f"Created new header with timestamp sec: {new_data.header.stamp.sec}, nanosec: {new_data.header.stamp.nanosec}")

#             # **序列化 new_data 消息**
#             serialized_new_data = rclpy.serialization.serialize_message(new_data)

#             # 写入新消息到新的话题 /sensor_data_with_head
#             new_topic_name = "/sensor_data_with_head"
#             print(f"Writing new topic: {new_topic_name} with serialized message")

#         else:
#             # 对于其他话题，保持原始的名称和数据
#             new_topic_name = topic
#             serialized_new_data = data  # 原始数据已经是序列化形式

#         # 从原始的 bag 中获取 topic 的元数据（类型等）
#         topic_metadata = original_metadata.get(topic)
#         if topic_metadata:
#             topic_type = topic_metadata.type
#         else:
#             # 如果没有找到相应的元数据，默认类型设置为 bytes
#             topic_type = 'builtins/bytes'

#         # 创建新的话题元数据（如果还没有创建）
#         if new_topic_name not in topic_metadata_created:
#             new_topic_metadata = TopicMetadata(
#                 name=new_topic_name,
#                 type="anomaly_detection/msg/SensorDataHeader" if new_topic_name == "/sensor_data_with_head" else topic_type,
#                 serialization_format=topic_metadata.serialization_format if topic_metadata else 'cdr',
#                 offered_qos_profiles=''  # 可以根据需要设置 QoS
#             )
#             writer.create_topic(new_topic_metadata)
#             topic_metadata_created[new_topic_name] = True
#             print(f"Topic created: {new_topic_name} with type {new_topic_metadata.type}")

#         # 写入消息到对应的话题
#         writer.write(new_topic_name, serialized_new_data, timestamp)

# # 主程序入口
# input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/1.7cm/self/self-0926-190323/self-0926-190323_0.db3'
# output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/1.7cm/self/self-0926-190323/head_stamp/self-0926-190323_0.db3'

# # 获取原始的bag文件元数据
# original_metadata = get_bag_metadata(input_bag_path)

# # 读取原始的bag文件
# messages = read_bag(input_bag_path)

# # 写入新的bag文件
# write_bag(output_bag_path, messages, original_metadata)






import rosbag2_py
import rclpy.serialization
from anomaly_detection.msg import SensorData, SensorDataHeader  # 包含原始和新的消息类型
from rosbag2_py._storage import TopicMetadata


# 读取bag文件的元数据
def get_bag_metadata(input_bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    info = rosbag2_py.Info()
    metadata = info.read_metadata(input_bag_path, 'sqlite3')
    
    # 使用 topic_metadata 而不是直接访问 name 属性
    topic_metadata = {topic_info.topic_metadata.name: topic_info.topic_metadata for topic_info in metadata.topics_with_message_count}
    
    return topic_metadata

# 读取bag文件
def read_bag(input_bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    messages = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        messages.append((topic, data, t))
    
    return messages

# 动态复制一个消息对象的所有字段到另一个消息对象
def copy_message_fields(source_msg, target_msg):
    for field in source_msg.__slots__:
        # 动态获取和设置属性值
        setattr(target_msg, field, getattr(source_msg, field))

# 将 time_ses 和 time_nsec 转换为 stamp，并包含在 header 中
def convert_to_header(time_ses, time_nsec, frame_id="sensor_data"):
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    # 创建一个 Header 消息
    header = Header()
    
    # 创建并设置 Time 对象
    stamp = Time()
    stamp.sec = int(time_ses)  # 确保 time_ses 是秒
    stamp.nanosec = int(time_nsec)  # 确保 time_nsec 是纳秒
    
    # 将 stamp 赋值给 header 的 stamp 字段
    header.stamp = stamp
    header.frame_id = frame_id  # 设置 frame_id
    
    return header

# 写入新的bag文件，并修改话题名称
def write_bag(output_bag_path, messages, original_metadata):
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    # 遍历消息，创建对应的话题元数据
    topic_metadata_created = {}

    for topic, data, timestamp in messages:
        new_data = None  # 初始化 new_data
        if topic == "/sensor_data":
            print(f"Processing topic: {topic}")
            # 反序列化数据为原始 SensorData 类型
            original_data = rclpy.serialization.deserialize_message(data, SensorData)

            # 打印原始消息中的时间数据，确保它们是有效的
            print(f"Original time_ses: {original_data.time_ses}, time_nsec: {original_data.time_nsec}")

            # 创建一个新的消息类型 SensorDataHeader，并动态复制字段
            new_data = SensorDataHeader()
            new_data.header = convert_to_header(original_data.time_ses, original_data.time_nsec)
            
            # 使用动态字段复制函数将所有字段复制到新的 SensorDataHeader 消息中
            copy_message_fields(original_data, new_data)

            # 确保 header 被正确赋值
            print(f"Created new header with timestamp sec: {new_data.header.stamp.sec}, nanosec: {new_data.header.stamp.nanosec}")

            # **序列化 new_data 消息**
            serialized_new_data = rclpy.serialization.serialize_message(new_data)

            # 写入新消息到新的话题 /sensor_data_with_head
            new_topic_name = "/sensor_data_with_head"
            print(f"Writing new topic: {new_topic_name} with serialized message")

        else:
            # 对于其他话题，保持原始的名称和数据
            new_topic_name = topic
            serialized_new_data = data  # 原始数据已经是序列化形式

        # 从原始的 bag 中获取 topic 的元数据（类型等）
        topic_metadata = original_metadata.get(topic)
        if topic_metadata:
            topic_type = topic_metadata.type
        else:
            # 如果没有找到相应的元数据，默认类型设置为 bytes
            topic_type = 'builtins/bytes'

        # 创建新的话题元数据（如果还没有创建）
        if new_topic_name not in topic_metadata_created:
            new_topic_metadata = TopicMetadata(
                name=new_topic_name,
                type="anomaly_detection/msg/SensorDataHeader" if new_topic_name == "/sensor_data_with_head" else topic_type,
                serialization_format=topic_metadata.serialization_format if topic_metadata else 'cdr',
                offered_qos_profiles=''  # 可以根据需要设置 QoS
            )
            writer.create_topic(new_topic_metadata)
            topic_metadata_created[new_topic_name] = True
            print(f"Topic created: {new_topic_name} with type {new_topic_metadata.type}")

        # 写入消息到对应的话题
        writer.write(new_topic_name, serialized_new_data, timestamp)

# 主程序入口
# input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/1.7cm/self/self-0926-190323/self-0926-190323_0.db3'
# output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/1.7cm/self/self-0926-190323/head_stamp/self-0926-190323_0.db3'

# input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/self-0926-190937_0.db3'
# output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/圆管---触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/2.0cm/self/self-0926-190937/head_stamp/self-0926-190937_0.db3'

# input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/窄道触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-191248/self-0926-191248_0.db3'
# output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/窄道触发打滑---打滑开始时开始slip话题，结束slip话题后再结束录包/self/self-0926-191248/header/self-0926-191248_0.db3'


# input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/斜坡---没有触发打滑/8度---没有触发打滑---slip刚刚开始上斜坡时录制，在斜坡上结束slip录制/self/self-0926-192620/self-0926-192620_0.db3'
# output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/斜坡---没有触发打滑/8度---没有触发打滑---slip刚刚开始上斜坡时录制，在斜坡上结束slip录制/self/self-0926-192620/header/self-0926-192620_0.db3'

# input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/巴塞罗纳椅---上去时没有触发打滑，下来时人为把机器往上推一点，触发了一点点打滑，在slip话题中间/机器人平地上椅子腿---触发了前碰撞/self/self-0926-191505/self-0926-191505_0.db3'
# output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/巴塞罗纳椅---上去时没有触发打滑，下来时人为把机器往上推一点，触发了一点点打滑，在slip话题中间/机器人平地上椅子腿---触发了前碰撞/self/self-0926-191505/header/self-0926-191505_0.db3'

input_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/巴塞罗纳椅---上去时没有触发打滑，下来时人为把机器往上推一点，触发了一点点打滑，在slip话题中间/机器人在椅子腿上下来---轮子悬空，发生了空转/self/self-0926-191833/self-0926-191833_0.db3'
output_bag_path = '/home/gechai/下载/27.09/1.5代样机---打滑数据采集---rom1105+driver2636/巴塞罗纳椅---上去时没有触发打滑，下来时人为把机器往上推一点，触发了一点点打滑，在slip话题中间/机器人在椅子腿上下来---轮子悬空，发生了空转/self/self-0926-191833/header/self-0926-191833_0.db3'






# 获取原始的bag文件元数据
original_metadata = get_bag_metadata(input_bag_path)

# 读取原始的bag文件
messages = read_bag(input_bag_path)

# 写入新的bag文件
write_bag(output_bag_path, messages, original_metadata)
