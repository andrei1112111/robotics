#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import struct
import math
import std_msgs.msg

class LidarTo3D(Node):
    def __init__(self):
        super().__init__("lidar_to_3d_pointcloud")

        self.declare_parameter("scan_topic", "/robot/scan")
        self.declare_parameter("output_topic", "/robot/virtual_depth/points")
        self.declare_parameter("frame_id", "robot/base_link")

        self.scan_topic = self.get_parameter("scan_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.declare_parameter("vertical_fov_deg", 30.0)
        self.declare_parameter("num_layers", 16)

        self.vertical_fov_deg = float(self.get_parameter("vertical_fov_deg").value)
        self.num_layers = int(self.get_parameter("num_layers").value)

        # Правильные вертикальные углы (никаких углов назад!)
        v_fov = np.radians(self.vertical_fov_deg)
        self.layer_angles = np.linspace(-v_fov / 2, +v_fov / 2, self.num_layers)

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f"Virtual 3D lidar running. Layers={self.num_layers}, FOV={self.vertical_fov_deg} deg"
        )

    def scan_callback(self, scan: LaserScan):
        points = []

        angle = scan.angle_min
        for r in scan.ranges:
            if np.isinf(r) or np.isnan(r) or r <= 0.01:
                angle += scan.angle_increment
                continue

            # базовые координаты 2D скана
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # создаём N копий этого луча по вертикали
            for v_ang in self.layer_angles:
                # НИКАКИХ ТОЧЕК НАЗАД:
                X = x * math.cos(v_ang)        # forward component ALWAYS positive
                Z = x * math.sin(v_ang)
                Y = y                          # 그대로

                # фильтрация от случайных отрицательных X
                if X <= 0:
                    continue

                points.append((X, Y, Z))

            angle += scan.angle_increment

        if not points:
            return

        # Создаём PointCloud2
        cloud = self.points_to_cloud(points, scan.header.stamp)
        self.publisher.publish(cloud)

    def points_to_cloud(self, pts, stamp):
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        data = []
        for x, y, z in pts:
            data.append(struct.pack('fff', x, y, z))
        data = b''.join(data)

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = data

        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = LidarTo3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()