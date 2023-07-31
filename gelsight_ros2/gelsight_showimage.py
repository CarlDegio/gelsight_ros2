import cv2
from PIL import Image
from threading import Thread, Lock
import rclpy
from rclpy.node import Node  # Enables the use of rclpy's Node class
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gelsight import gsdevice


class CameraPublisher(Node):

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('gelsight_showimage')
        self.declare_parameter('device_name', 'GelSight Mini')
        self.declare_parameter('image_frequency', 20.0)

        finger = gsdevice.Finger.MINI
        cam_id = gsdevice.get_camera_id(self.get_parameter('device_name').value)
        self.dev = gsdevice.Camera(finger, cam_id)
        self.dev.connect()
        figure0 = self.dev.get_raw_image()
        print('image size = ', figure0.shape[1], figure0.shape[0])
        self.roi = (0, 0, figure0.shape[1], figure0.shape[0])
        print("roi = ", self.roi)
        print("start publish gelsight raw image...")

        # Maximum queue size of 10.
        self.publisher_cam = self.create_publisher(Image, '/gsmini/image', 10)
        timer_period = 1 / self.get_parameter('image_frequency').value
        self.timer = self.create_timer(timer_period, self.publish_image)

        self.cvbridge = CvBridge()

    def publish_image(self):
        img = self.dev.get_image(self.roi)
        msg = self.cvbridge.cv2_to_imgmsg(img, encoding="passthrough")  # Store the x and y coordinates of the object
        self.publisher_cam.publish(msg)  # Publish the position to the topic


def main(args=None):
    imgh = 240
    imgw = 320

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
