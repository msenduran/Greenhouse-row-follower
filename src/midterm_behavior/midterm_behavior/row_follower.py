"""Greenhouse row follower: sensor-based behavior node (state machine).

Inputs
  /scan                  sensor_msgs/LaserScan       row distance/angle (right sector), end of row, obstacles (front)
  /imu                   sensor_msgs/Imu             yaw change during the U-turn
  /fruit_detector/fruit  geometry_msgs/PointStamped  ripe fruit position in the side camera image
  /odom                  nav_msgs/Odometry           distance driven (short advance before the U-turn), marker positions
Outputs
  /cmd_vel               geometry_msgs/Twist         motion command
  /behavior/state        std_msgs/String             current state (for monitoring)
  /behavior/markers      visualization_msgs/MarkerArray  state text above the robot + marked fruit (RViz)

States
  ROW_FOLLOW     drive forward, P-control on the distance/angle to the row on the right (LiDAR)
  FRUIT_STOP     a ripe fruit is centered in the camera image: stop, wait, mark it, continue
  U_TURN         right side lost (end of row): advance a little, then arc right until the IMU yaw is
                 180 deg from the heading recorded while following the row
  OBSTACLE_STOP  something in front closer than obstacle_distance (any state): stop until clear
  DONE           rows_to_cover rows finished
"""

import math

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from midterm_behavior.row_geometry import (
    fit_row, sector_min, sector_points, wrap_angle, yaw_from_quaternion)

ROW_FOLLOW = 'ROW_FOLLOW'
FRUIT_STOP = 'FRUIT_STOP'
U_TURN = 'U_TURN'
OBSTACLE_STOP = 'OBSTACLE_STOP'
DONE = 'DONE'

DEG = math.pi / 180.0


class RowFollower(Node):

    def __init__(self):
        super().__init__('row_follower')
        p = self.declare_parameter
        # Row following
        self.forward_speed = p('forward_speed', 0.25).value
        self.target_distance = p('target_distance', 0.45).value   # LiDAR to row face [m]
        self.kp_distance = p('kp_distance', 2.5).value             # rad/s per m of distance error
        self.kp_angle = p('kp_angle', 1.5).value                   # rad/s per rad of row angle
        self.max_angular = p('max_angular', 0.6).value
        self.row_max_range = p('row_max_range', 1.2).value         # row points farther than this are ignored
        self.row_lost_range = p('row_lost_range', 1.0).value       # nothing closer on the right = end of row
        # U-turn
        self.turn_radius = p('turn_radius', 0.65).value            # target distance + half row width
        self.turn_speed = p('turn_speed', 0.2).value
        self.turn_advance = p('turn_advance', 0.15).value          # drive past the row end before turning [m]
        self.rows_to_cover = p('rows_to_cover', 2).value           # 0 = keep going forever
        # Fruit
        self.fruit_center_tolerance = p('fruit_center_tolerance', 0.15).value
        self.fruit_stop_time = p('fruit_stop_time', 3.0).value
        self.fruit_timeout = 0.5                                   # s without detections = fruit left the view
        # Obstacles
        self.obstacle_distance = p('obstacle_distance', 0.45).value  # from the LiDAR (front face is 0.175 m ahead)
        self.obstacle_half_angle = p('obstacle_half_angle', 20.0).value * DEG

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'behavior/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'behavior/markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointStamped, 'fruit_detector/fruit', self.on_fruit, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        # Latest sensor-derived values
        self.row = None                # (distance, angle) of the row on the right, or None
        self.right_clear = True        # nothing beside the robot on the right
        self.front_min = math.inf
        self.yaw = None
        self.fruit_offset = None
        self.fruit_stamp = None
        self.pose = None               # (x, y, yaw) from odom

        # State machine
        self.state = ROW_FOLLOW
        self.state_start = self.now()
        self.resume_state = ROW_FOLLOW
        self.row_acquired = False      # end-of-row check only after the row has been seen
        self.fruit_handled = False     # current fruit already marked: wait until it leaves the view
        self.row_heading = None        # IMU yaw while driving parallel to the row
        self.turn_target_yaw = None
        self.turn_start_pose = None
        self.rows_done = 0
        self.fruit_count = 0
        self.fruit_markers = []

        self.create_timer(0.05, self.step)
        self.get_logger().info('row follower started')

    # ---------------------------------------------------------------- sensor callbacks

    def on_scan(self, msg):
        args = (msg.ranges, msg.angle_min, msg.angle_increment)
        self.row = fit_row(sector_points(*args, -135 * DEG, -45 * DEG, self.row_max_range))
        self.right_clear = len(sector_points(*args, -100 * DEG, -80 * DEG, self.row_lost_range)) == 0
        self.front_min = sector_min(*args, -self.obstacle_half_angle, self.obstacle_half_angle)

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_fruit(self, msg):
        self.fruit_offset = msg.point.x
        self.fruit_stamp = self.now()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # ---------------------------------------------------------------- helpers

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state
            self.state_start = self.now()

    def fruit_visible(self):
        return self.fruit_stamp is not None and self.now() - self.fruit_stamp < self.fruit_timeout

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # ---------------------------------------------------------------- state machine

    def step(self):
        if self.yaw is None or self.pose is None:
            return  # wait for sensors

        # Safety first: obstacle in front stops the robot in any state
        if self.state not in (OBSTACLE_STOP, DONE) and self.front_min < self.obstacle_distance:
            self.resume_state = self.state
            self.set_state(OBSTACLE_STOP)

        handler = {
            ROW_FOLLOW: self.row_follow,
            FRUIT_STOP: self.fruit_stop,
            U_TURN: self.u_turn,
            OBSTACLE_STOP: self.obstacle_stop,
            DONE: self.done,
        }[self.state]
        handler()

        self.state_pub.publish(String(data=self.state))
        self.publish_markers()

    def row_follow(self):
        if not self.fruit_visible():
            self.fruit_handled = False

        if self.row_acquired and self.right_clear:
            # End of the row
            self.rows_done += 1
            self.get_logger().info(f'end of row {self.rows_done}')
            if self.rows_to_cover and self.rows_done >= self.rows_to_cover:
                self.set_state(DONE)
            else:
                self.turn_start_pose = self.pose
                self.turn_target_yaw = None
                self.set_state(U_TURN)
            return

        if self.row_acquired and not self.fruit_handled and self.fruit_visible() \
                and abs(self.fruit_offset) < self.fruit_center_tolerance:
            self.set_state(FRUIT_STOP)
            return

        if not self.right_clear:
            self.row_acquired = True
        if not self.row_acquired or self.row is None:
            # Row not beside the robot yet (start, or just after a U-turn), or no clean line
            # (row end face in view): drive straight
            self.publish_cmd(self.forward_speed * 0.8, 0.0)
            return

        distance, angle = self.row
        if abs(angle) < 3 * DEG and abs(self.target_distance - distance) < 0.05:
            # Driving parallel to the row: remember this heading as the U-turn reference
            self.row_heading = self.yaw
        angular = self.kp_distance * (self.target_distance - distance) + self.kp_angle * angle
        angular = max(-self.max_angular, min(self.max_angular, angular))
        self.publish_cmd(self.forward_speed, angular)

    def fruit_stop(self):
        self.publish_cmd(0.0, 0.0)
        if self.now() - self.state_start >= self.fruit_stop_time:
            self.fruit_count += 1
            self.fruit_handled = True
            self.mark_fruit()
            self.get_logger().info(f'ripe fruit #{self.fruit_count} marked '
                                   f'(robot at x={self.pose[0]:.2f} y={self.pose[1]:.2f})')
            self.set_state(ROW_FOLLOW)

    def u_turn(self):
        if self.turn_target_yaw is None:
            # Phase 1: drive a little past the row end
            x0, y0, _ = self.turn_start_pose
            if math.hypot(self.pose[0] - x0, self.pose[1] - y0) < self.turn_advance:
                self.publish_cmd(self.turn_speed, 0.0)
                return
            # Target: opposite of the heading along the row (not of the current yaw, which may
            # already be off if the robot drifted at the row end)
            reference = self.row_heading if self.row_heading is not None else self.yaw
            self.turn_target_yaw = wrap_angle(reference + math.pi)
            self.get_logger().info(f'U-turn: row heading {math.degrees(reference):.0f} deg, '
                                   f'target {math.degrees(self.turn_target_yaw):.0f} deg')
        # Phase 2: arc to the right around the row end until the target yaw is reached
        # Angle still to turn (clockwise), in [0, 2*pi): ~pi at the start, decreasing to 0.
        # Values above 1.5*pi mean the target was just passed.
        remaining = (self.yaw - self.turn_target_yaw) % (2 * math.pi)
        if 3 * DEG < remaining < 1.5 * math.pi:
            self.publish_cmd(self.turn_speed, -self.turn_speed / self.turn_radius)
            return
        # Turn finished: follow the other face of the same row
        self.row_acquired = False
        self.publish_cmd(0.0, 0.0)
        self.set_state(ROW_FOLLOW)

    def obstacle_stop(self):
        self.publish_cmd(0.0, 0.0)
        if self.front_min >= self.obstacle_distance + 0.05:   # small hysteresis
            if self.now() - self.state_start > 1.0:
                self.set_state(self.resume_state)
        else:
            self.state_start = self.now()

    def done(self):
        self.publish_cmd(0.0, 0.0)

    # ---------------------------------------------------------------- RViz markers

    def mark_fruit(self):
        # The camera looks to the right (-y of the robot) at the row ~target_distance away
        x, y, yaw = self.pose
        d = self.target_distance + 0.1
        self.fruit_markers.append((x + d * math.sin(yaw), y - d * math.cos(yaw)))

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        text = Marker()
        text.header.frame_id = 'base_footprint'
        text.header.stamp = stamp
        text.ns, text.id = 'state', 0
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.z = 0.8
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'{self.state}  |  ripe fruit: {self.fruit_count}'
        markers.markers.append(text)

        for i, (fx, fy) in enumerate(self.fruit_markers):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns, m.id = 'fruit', i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = fx, fy, 0.45
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.9
            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = RowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
