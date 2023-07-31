import sys, getopt
import numpy as np
import cv2
import os
import copy
from gelsight import gsdevice
from gelsight import gs3drecon
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg


class PCDPublisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_node')
        self.declare_parameter('device_name', 'GelSight Mini')
        self.declare_parameter('image_frequency', 5.0)

        # Set flags
        GPU = False

        # Set the camera resolution
        # mmpp = 0.0887  # for 240x320 img size
        # mmpp = 0.1778  # for 160x120 img size from R1
        # mmpp = 0.0446  # for 640x480 img size R1
        # mmpp = 0.029 # for 1032x772 img size from R1
        mmpp = 0.081  # r2d2 gel 18x24mm at 240x320
        self.mpp = mmpp / 1000.

        finger = gsdevice.Finger.MINI
        cam_id = gsdevice.get_camera_id(self.get_parameter('device_name').value)
        self.dev = gsdevice.Camera(finger, cam_id)
        self.dev.connect()

        ''' Load neural network '''
        script_file_path = os.path.dirname(__file__)
        net_file_path = 'nnmini.pt'  # TODO: move the .pt to install
        net_path = os.path.join(script_file_path, net_file_path)
        print('net path = ', net_path)

        if GPU:
            gpuorcpu = "cuda"
        else:
            gpuorcpu = "cpu"
        self.nn = gs3drecon.Reconstruction3D(gs3drecon.Finger.R15, self.dev)
        net = self.nn.load_nn(net_path, gpuorcpu)
        if net is None:
            print('Failed to load neural network')
            sys.exit(0)

        figure0 = self.dev.get_raw_image()
        print('image size = ', figure0.shape[1], figure0.shape[0])
        self.roi = (0, 0, figure0.shape[1], figure0.shape[0])
        print("roi = ", self.roi)
        print("start publish gelsight pointcloud...")

        ''' point array to store point cloud data points '''
        x = np.arange(self.dev.imgh) * self.mpp
        y = np.arange(self.dev.imgw) * self.mpp
        X, Y = np.meshgrid(x, y)
        self.points = np.zeros([self.dev.imgw * self.dev.imgh, 3])
        self.points[:, 0] = np.ndarray.flatten(X)
        self.points[:, 1] = np.ndarray.flatten(Y)
        Z = np.zeros((self.dev.imgh, self.dev.imgw))  # initialize points array with zero depth values
        self.points[:, 2] = np.ndarray.flatten(Z)

        self.pcd_publisher = self.create_publisher(PointCloud2, '/gsmini/pcd', 10)
        timer_period = 1 / self.get_parameter('image_frequency').value
        self.timer = self.create_timer(timer_period, self.publisher_pc)

    def publisher_pc(self):

        # get the roi image
        f1 = self.dev.get_image(self.roi)

        # compute the depth map
        dm = self.nn.get_depthmap(f1, False)

        dm_ros = copy.deepcopy(dm) * self.mpp
        self.points[:, 2] = np.ndarray.flatten(dm_ros)

        # Here I use the point_cloud() function to convert the numpy array
        # into a sensor_msgs.PointCloud2 object. The second argument is the
        # name of the frame the point cloud will be represented in. The default
        # (fixed) frame in RViz is called 'map'
        self.pcd = point_cloud(self.points, 'map')
        # Then I publish the PointCloud2 object
        self.pcd_publisher.publish(self.pcd)


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.
    data = points.astype(dtype).tobytes()
    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]
    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = std_msgs.msg.Header(frame_id=parent_frame)
    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = PCDPublisher()
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
