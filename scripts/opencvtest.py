import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.bridge = CvBridge()

        self.done = False

        self.subscription = self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.subscription

    def listener_callback(self, msg):
        if self.done:
            return
        
        cv_img = self.bridge.imgmsg_to_cv(msg, desired_encoding='bgr8')
        cv2.imwrite('frame.png', cv_img)
        self.get_logger().info(f"Saved frame with shape: {csv_image.shape}")
        self.done =True
            

def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)


    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()