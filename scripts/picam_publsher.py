import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera', 10)
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)
        self.camera = Picamera2()
        self.camera_config = self.camera.create_preview_configuration(main={"size": (1920, 1080)})
        self.camera.configure(self.camera_config)
        self.camera.start()

    def timer_callback(self):
        frame = self.camera.capture_array()
        image_message = Image()
        image_message.height = 1080
        image_message.width = 1920
        image_message.encoding = 'rgb8'
        image_message.step = 1920 * 3
        image_message.data = np.array(frame).tobytes()
        self.publisher.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
