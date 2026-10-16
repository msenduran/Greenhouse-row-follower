"""Detect ripe (red) fruit in the side camera image.

Subscribes:  /camera/image_raw (sensor_msgs/Image)
Publishes:   /fruit_detector/fruit (geometry_msgs/PointStamped), only when a fruit is seen:
                 point.x = horizontal offset of the fruit from the image center, -1 (left) .. +1 (right)
                 point.y = vertical offset, -1 (top) .. +1 (bottom)
                 point.z = fraction of the image covered by the fruit
             /fruit_detector/debug_image (sensor_msgs/Image): detection drawn on the camera image
"""

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from midterm_behavior.fruit_detection import detect_fruit, draw_debug, HsvThresholds


class FruitDetector(Node):

    def __init__(self):
        super().__init__('fruit_detector')
        self.thresholds = HsvThresholds(
            hue_low_max=self.declare_parameter('hue_low_max', 10).value,
            hue_high_min=self.declare_parameter('hue_high_min', 170).value,
            sat_min=self.declare_parameter('sat_min', 120).value,
            val_min=self.declare_parameter('val_min', 50).value,
        )
        self.min_area = float(self.declare_parameter('min_area', 150.0).value)
        self.bridge = CvBridge()
        self.fruit_pub = self.create_publisher(PointStamped, 'fruit_detector/fruit', 10)
        self.debug_pub = self.create_publisher(Image, 'fruit_detector/debug_image', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.seen = False

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detection, mask = detect_fruit(bgr, self.thresholds, self.min_area)

        if detection is not None:
            out = PointStamped()
            out.header = msg.header
            out.point.x = detection.offset_x
            out.point.y = detection.offset_y
            out.point.z = detection.area_ratio
            self.fruit_pub.publish(out)
        if (detection is not None) != self.seen:
            self.seen = detection is not None
            self.get_logger().info('ripe fruit in view' if self.seen else 'ripe fruit out of view')

        if self.debug_pub.get_subscription_count() > 0:
            debug = self.bridge.cv2_to_imgmsg(draw_debug(bgr, detection, mask), 'bgr8')
            debug.header = msg.header
            self.debug_pub.publish(debug)


def main():
    rclpy.init()
    node = FruitDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
