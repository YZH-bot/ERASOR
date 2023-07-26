#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import tf2_ros
import rosbag
import os
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform, Pose
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointField
from erasor.msg import node

import argparse

def process_parameters():
    parser = argparse.ArgumentParser(description='Convert own data to erasor-type data')
    parser.add_argument('--lidar_topic', type=str, default="/rslidar_points", help='lidar_topic')
    parser.add_argument('--world_frame', type=str, default="world", help='world_frame')
    parser.add_argument('--lidar_frame', type=str, default="lidar", help='lidar_frame')
    
    args = parser.parse_args()
    print(f"lidar_topic: {args.lidar_topic}")
    print(f"world_frame: {args.world_frame}")
    print(f"lidar_frame: {args.lidar_frame}")
    return args

def filter_nan_points(point_cloud):
    # 创建一个新的点云消息
    filtered_points = PointCloud2()
    filtered_points.header = point_cloud.header
    filtered_points.height = point_cloud.height
    filtered_points.width = point_cloud.width

    # 解析点云数据
    points = list(pcl2.read_points(point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True))

    # 将数据转换为numpy数组
    points_np = np.array(points, dtype=np.float32)

    # 将数据重新打包成PointCloud2格式
    filtered_points = pcl2.create_cloud_xyz32(filtered_points.header, points_np[:, :3])

    # 添加强度字段（如果有的话）
    if "intensity" in point_cloud.fields:
        intensity_field = next(f for f in point_cloud.fields if f.name == "intensity")
        intensity_values = points_np[:, intensity_field.offset].tolist()
        intensity_field_data = pcl2.create_cloud_field("float32", intensity_field.count, "intensity", intensity_values)
        filtered_points.fields.append(intensity_field_data)

    return filtered_points

class PointCloudSubscriber(object):

    def __init__(self,args) -> None:
        self.bag = rosbag.Bag(os.path.join("/media/mapping/YZH2/bag/ERASOR/", "own_data_to_erasor_data.bag"),'w', compression=rosbag.Compression.NONE)
        self.listener = tf.TransformListener()
        self.lidar_topic = args.lidar_topic
        self.world_frame = args.world_frame
        self.lidar_frame = args.lidar_frame
        self.sub = rospy.Subscriber("/rslidar_points",
                                     PointCloud2,
                                     self.callback, queue_size=5)
        print(f"lidar_topic: {self.lidar_topic}")
        print(f"world_frame: {self.world_frame}")
        print(f"lidar_frame: {self.lidar_frame}")

    def callback(self, msg):
        assert isinstance(msg, PointCloud2)

        (trans, rot) = self.listener.lookupTransform('world', 'lidar', rospy.Time(0))
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = rospy.Time.now()
        tf_stamped.header.frame_id = 'map'
        tf_stamped.child_frame_id = 'camera_left'

        transform = Transform()
        transform.translation.x = trans[0]
        transform.translation.y = trans[1]
        transform.translation.z = trans[2]
        transform.rotation.x = rot[0]
        transform.rotation.y = rot[1]
        transform.rotation.z = rot[2]
        transform.rotation.w = rot[3]
        tf_stamped.transform = transform

        tf_msg = TFMessage()
        tf_msg.transforms.append(tf_stamped)
        self.bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)


        pose_msg = Pose()
        pose_msg.position.x = trans[0]
        pose_msg.position.y = trans[1]
        pose_msg.position.z = trans[2]
        pose_msg.orientation.x = rot[0]
        pose_msg.orientation.y = rot[1]
        pose_msg.orientation.z = rot[2]
        pose_msg.orientation.w = rot[3]

        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('intensity', 12, PointField.FLOAT32, 1)]
        out = node()
        out.header = header
        out.odom = pose_msg
        
        # lidar_msg = PointCloud2()
        # lidar_msg.header.frame_id = ""
        msg.header.frame_id="map"
        msg.header.stamp=rospy.Time.now()
        print(msg.header.frame_id)
        filtered_points = filter_nan_points(msg)
        out.lidar = filtered_points

        self.bag.write('/node/combined/optimized', out, out.header.stamp)
        self.bag.write('/debug/pc_raw', filtered_points, out.header.stamp)
        self.bag.write('/debug/pc_label', filtered_points, out.header.stamp)

        print(pose_msg)

    def __del__(self):
        print("## OVERVIEW ##")
        print(self.bag)
        self.bag.close()
        
if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    args = process_parameters()
    PointCloudSubscriber(args)
    rospy.spin()
