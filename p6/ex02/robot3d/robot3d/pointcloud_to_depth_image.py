#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import struct


class CloudToDepth(Node):
    def __init__(self):
        super().__init__("pointcloud_to_depth_image")

        # Camera parameters
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("fx", 200.0)
        self.declare_parameter("fy", 200.0)
        self.declare_parameter("cx", 160.0)
        self.declare_parameter("cy", 120.0)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)

        # Fixed depth clipping
        self.near = 0.2
        self.far = 5.0

        # Publishers
        self.depth_pub = self.create_publisher(Image, "/robot/virtual_depth/image", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/robot/virtual_depth/camera_info", 10)

        # Subscribe to point cloud from virtual 3D lidar
        self.create_subscription(
            PointCloud2,
            "/robot/virtual_depth/points",
            self.cloud_callback,
            10,
        )

        self.get_logger().info("Depth image generator started")

    def pointcloud2_to_xyz(self, cloud):
        pts = []

        x_offset = cloud.fields[0].offset
        y_offset = cloud.fields[1].offset
        z_offset = cloud.fields[2].offset

        point_step = cloud.point_step
        data = cloud.data

        for i in range(cloud.width):
            base = i * point_step
            try:
                x = struct.unpack_from('f', data, base + x_offset)[0]
                y = struct.unpack_from('f', data, base + y_offset)[0]
                z = struct.unpack_from('f', data, base + z_offset)[0]
                pts.append((x, y, z))
            except struct.error:
                continue

        return pts

    def cloud_callback(self, cloud_msg):

        points = self.pointcloud2_to_xyz(cloud_msg)
        if not points:
            return

        # Empty depth image
        depth = np.full((self.height, self.width), np.inf, dtype=np.float32)

        for x, y, z in points:

            # Convert lidar → camera frame
            # lidar: +X forward, +Y left, +Z up
            # camera: +Z forward, +X right, +Y down
            cx = -y
            cy = -z
            cz = x

            if cz <= 0:  # behind camera → skip
                continue

            # Project to pixel coords
            u = int(self.fx * (cx / cz) + self.cx)
            v = int(self.fy * (cy / cz) + self.cy)

            if 0 <= u < self.width and 0 <= v < self.height:
                if cz < depth[v, u]:
                    depth[v, u] = cz

        # Replace missing values
        depth[np.isinf(depth)] = 0.0

        depth_clamped = np.clip(depth, self.near, self.far)

        # YOU WANT: farther = brighter, nearer = darker
        depth_norm = (depth_clamped - self.near) / (self.far - self.near)

        depth_uint8 = (depth_norm * 255).astype(np.uint8)

        # Publish depth image
        image_msg = Image()
        image_msg.header = cloud_msg.header
        image_msg.height = self.height
        image_msg.width = self.width
        image_msg.encoding = "mono8"
        image_msg.is_bigendian = False
        image_msg.step = self.width
        image_msg.data = depth_uint8.tobytes()
        self.depth_pub.publish(image_msg)

        # Camera info
        info = CameraInfo()
        info.header = image_msg.header
        info.width = self.width
        info.height = self.height
        info.k = [
            float(self.fx), 0.0, float(self.cx),
            0.0, float(self.fy), float(self.cy),
            0.0, 0.0, 1.0
        ]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
