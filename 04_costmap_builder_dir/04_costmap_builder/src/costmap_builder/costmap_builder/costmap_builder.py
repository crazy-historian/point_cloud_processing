import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import open3d as o3d

from .utils import ros_point_cloud_to_numpy_point_cloud, transformation_stamped_to_transformation_matrix


class CostmapBuilder(Node):
    COSTMAP_RANGE = 10  # meters
    COSTMAP_RESOLUTION = (300, 300)  # (w,h) pixels

    def __init__(self):
        super().__init__("convert_node")
        self.get_logger().info("Starting work")
        self.get_logger().info(f'Numpy version {np.version.version}')
        self.subscriber = self.create_subscription(PointCloud2, "/velodyne_points", self.listener_callback, 2)
        self.publisher = self.create_publisher(Image, "/costmap", 10)

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _create_costmap_from_points(self, pcd):
        costmap = np.zeros((self.COSTMAP_RESOLUTION[1], self.COSTMAP_RESOLUTION[0]), dtype=np.uint8)

        distances = np.sqrt(pcd[:, 0] ** 2 + pcd[:, 1] ** 2)
        close_points = pcd[distances < self.COSTMAP_RANGE]

        costmap_shape = np.array(self.COSTMAP_RESOLUTION)
        pixel_scale = (costmap_shape // 2) / self.COSTMAP_RANGE

        costmap_coords = (close_points[:, :2] * pixel_scale).astype(np.int32)
        costmap_coords = costmap_coords + costmap_shape // 2

        costmap[costmap_coords[:, 0], costmap_coords[:, 1]] = 255

        return costmap

    def to_homogeneous(self, pcd: np.array):
        new_shape = (pcd.shape[0], pcd.shape[1] + 1)
        hom_pcd = np.ones(shape=new_shape)
        hom_pcd[:, :-1] = pcd

        return hom_pcd

    def to_cartesian(self, pcd: np.array):
        return np.delete(pcd, 3, 1)

    def _transform_pcd(self, pcd: np.ndarray, transform: TransformStamped):
        M = transformation_stamped_to_transformation_matrix(transform)
        self.get_logger().info("Transform: %s" % (str(M)))

        pcd = self.to_homogeneous(pcd)
        pcd = np.dot(M, pcd.T)
        pcd = self.to_cartesian(pcd.T)

        return pcd

    def _filter_pcd(self, pcd: np.ndarray):

        z_min, z_max = 0.3, 2
        self.get_logger().info(f'Shape before : {pcd.shape}')
        pcd = np.delete(pcd, np.where((pcd[:, 2] >= z_max) | (pcd[:, 2] <= z_min))[0], axis=0)
        self.get_logger().info(f'Shape after height correction: {pcd.shape}')

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(pcd)
        cl, ind = o3d_pcd.remove_radius_outlier(nb_points=4, radius=1.0)
        inlier_cloud = o3d_pcd.select_by_index(ind)
        pcd = np.asarray(inlier_cloud.points)
        self.get_logger().info(f'Shape after snow removing: {pcd.shape}')

        return pcd

    def listener_callback(self, msg: PointCloud2):
        try:
            # get transformation from lidar to base_footprint (robot base)
            transform = self.tf_buffer.lookup_transform(
                "base_footprint",
                "base_scan",
                rclpy.time.Time())

            self.get_logger().info("Transformation: %s" % (str(transform)))
        except TransformException as ex:
            return

        # convert to numpy
        pcd = ros_point_cloud_to_numpy_point_cloud(msg)[:, :3]
        self.get_logger().info("pcd shape: %s" % (str(pcd.shape)))

        # transform pcd to base_footprint
        transformed_pcd = self._transform_pcd(pcd, transform)

        # filter points
        filtered_pcd = self._filter_pcd(transformed_pcd)

        # build costmap
        img = self._create_costmap_from_points(filtered_pcd)

        costmap_msg = self.bridge.cv2_to_imgmsg(img, "mono8")
        costmap_msg.header = msg.header

        self.publisher.publish(costmap_msg)


def main(args=None):
    rclpy.init(args=args)

    print("Subscriber created")
    convert_node = CostmapBuilder()
    rclpy.spin(convert_node)
    print("Spin")
    convert_node.destroy_node()
    print("Destroied")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
