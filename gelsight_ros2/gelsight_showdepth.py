import sys, getopt
import numpy as np
import cv2
import os
import copy
from gelsight import gsdevice
from gelsight import gs3drecon
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DepthPublisher(Node):

    def __init__(self):
        super().__init__('gelsight_showpcd')
        self.declare_parameter('device_name', 'GelSight Mini')
        self.declare_parameter('image_frequency', 5.0)

        # Set flags
        GPU = True

        # Set the camera resolution
        mmpp = 0.0634
        self.mpp = mmpp / 1000.

        self.dev = gsdevice.Camera(self.get_parameter('device_name').value)
        self.dev.connect()

        ''' Load neural network '''
        script_file_path = os.path.dirname(__file__)
        net_file_path = 'nnmini.pt'
        net_path = os.path.join(script_file_path, net_file_path)
        print('net path = ', net_path)

        if GPU:
            gpuorcpu = "cuda"
        else:
            gpuorcpu = "cpu"
        self.nn = gs3drecon.Reconstruction3D(self.dev)
        net = self.nn.load_nn(net_path, gpuorcpu)
        if net is None:
            print('Failed to load neural network')
            sys.exit(0)

        figure0 = self.dev.get_raw_image()
        print('image size = ', figure0.shape[1], figure0.shape[0])
        self.roi = (0, 0, figure0.shape[1], figure0.shape[0])
        print("roi = ", self.roi)
        print("start publish gelsight pointcloud...")

        self.depth_publisher = self.create_publisher(Image, '/gsmini/depth', 10)
        timer_period = 1 / self.get_parameter('image_frequency').value
        self.timer = self.create_timer(timer_period, self.publisher_depth)
        self.cvbridge = CvBridge()

    def publisher_depth(self):
        f1 = self.dev.get_image(self.roi)
        dm = self.nn.get_depthmap(f1, False)
        scaled_data = (dm / 30) * 255.0  # 可以根据实际情况调整dm幅度
        rounded_data = np.clip(np.round(scaled_data), 0, 255)
        dm_uint8 = rounded_data.astype(np.uint8)
        msg = self.cvbridge.cv2_to_imgmsg(dm_uint8, encoding="8UC1")
        self.depth_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pcd_publisher = DepthPublisher()
    rclpy.spin(pcd_publisher)
    pcd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
