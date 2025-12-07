import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped


class WallAvoider(Node):
    # States
    STATE_FORWARD = 0
    STATE_TURN_LEFT = 1     # turn left (wall on right)
    STATE_TURN_RIGHT = 2    # turn right (wall on left)

    def __init__(self):
        super().__init__('wall_avoider')

        self.declare_parameter('left_sensor_topic', '/ps6')
        self.declare_parameter('right_sensor_topic', '/ps7')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # I had to tune the distance threshold multiple times, to get the right threshold so that it wouldn't touch the wall.
        self.declare_parameter('threshold', 0.15) 
        self.declare_parameter('soft_distance', 0.22)
        self.declare_parameter('escape_threshold', 0.10)

        self.declare_parameter('forward_speed', 0.045)
        self.declare_parameter('turn_speed', 1.6)

        self.declare_parameter('control_period', 0.032)

        self.declare_parameter('turn_angle_deg', 110.0)
        self.declare_parameter('corner_turn_angle_deg', 150.0)

        # Read parameter values
        self.left_topic = self.get_parameter(
            'left_sensor_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter(
            'right_sensor_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value

        # Get the different speeds and threshold parameters
        self.threshold = float(
            self.get_parameter('threshold').get_parameter_value().double_value)
        self.soft_distance = float(
            self.get_parameter('soft_distance').get_parameter_value().double_value)
        self.escape_threshold = float(
            self.get_parameter('escape_threshold').get_parameter_value().double_value)

        self.forward_speed = float(
            self.get_parameter('forward_speed').get_parameter_value().double_value)
        self.turn_speed = float(
            self.get_parameter('turn_speed').get_parameter_value().double_value)
        self.control_period = float(
            self.get_parameter('control_period').get_parameter_value().double_value)

        self.turn_angle_deg = float(
            self.get_parameter('turn_angle_deg').get_parameter_value().double_value)
        self.corner_turn_angle_deg = float(
            self.get_parameter('corner_turn_angle_deg').get_parameter_value().double_value)

        if self.soft_distance < self.threshold:
            self.soft_distance = self.threshold * 1.3

        angle_rad = math.radians(self.turn_angle_deg)
        steps_for_angle = angle_rad / (self.turn_speed * self.control_period)
        self.turn_steps_default = max(5, int(round(steps_for_angle)))  # min 5 steps

        corner_angle_rad = math.radians(self.corner_turn_angle_deg)
        corner_steps_for_angle = corner_angle_rad / (self.turn_speed * self.control_period)
        self.corner_turn_steps = max(
            self.turn_steps_default + 2,
            int(round(corner_steps_for_angle))
        )

        self.get_logger().info(
            f'WallAvoider params: threshold={self.threshold:.3f}, soft={self.soft_distance:.3f}, '
            f'escape={self.escape_threshold:.3f}, forward_speed={self.forward_speed:.3f}, '
            f'turn_speed={self.turn_speed:.3f}, '
            f'normal_turn={self.turn_angle_deg:.1f} deg -> {self.turn_steps_default} steps, '
            f'corner_turn={self.corner_turn_angle_deg:.1f} deg -> {self.corner_turn_steps} steps'
        )

        # Publisher
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)

        # Subscriptions
        self.left_range = None
        self.right_range = None
        self.left_filt = None
        self.right_filt = None
        self.seen_sensors = False

        self.state = self.STATE_FORWARD
        self.turn_steps_remaining = 0

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            f'WallAvoider ROS2 node started. '
            f'left={self.left_topic}, right={self.right_topic}, cmd_vel={cmd_vel_topic}'
        )

        self.create_subscription(Range, self.left_topic, self.left_callback, 10)
        self.create_subscription(Range, self.right_topic, self.right_callback, 10)

    def left_callback(self, msg: Range):
        self.left_range = msg.range
        self._filter_update()
        self._log_first_sensor()

    def right_callback(self, msg: Range):
        self.right_range = msg.range
        self._filter_update()
        self._log_first_sensor()

    def _filter_update(self):
        if self.left_range is None or self.right_range is None:
            return

        alpha = 0.3
        if self.left_filt is None:
            self.left_filt = self.left_range
            self.right_filt = self.right_range
        else:
            self.left_filt = alpha * self.left_range + (1.0 - alpha) * self.left_filt
            self.right_filt = alpha * self.right_range + (1.0 - alpha) * self.right_filt

    def _log_first_sensor(self):
        if (not self.seen_sensors and
                self.left_filt is not None and
                self.right_filt is not None):
            self.seen_sensors = True
            self.get_logger().info(
                f'First filtered readings: left={self.left_filt:.3f} m, '
                f'right={self.right_filt:.3f} m'
            )

    def control_loop(self):
        if self.left_filt is None or self.right_filt is None:
            self._publish_cmd(self.forward_speed * 0.4, 0.0)
            return

        left_value = self.left_filt
        right_value = self.right_filt

        if self.state == self.STATE_FORWARD:
            danger_left = left_value < self.threshold
            danger_right = right_value < self.threshold

            very_close_left = left_value < self.escape_threshold
            very_close_right = right_value < self.escape_threshold
            in_corner = very_close_left and very_close_right

            if in_corner:
                if left_value < right_value:
                    self._start_turn_right(corner=True, reason="corner, LEFT closer")
                else:
                    self._start_turn_left(corner=True, reason="corner, RIGHT closer")

            elif danger_left or danger_right:
                if danger_left and not danger_right:
                    self._start_turn_right(corner=False, reason="wall on LEFT")
                elif danger_right and not danger_left:
                    self._start_turn_left(corner=False, reason="wall on RIGHT")
                else:
                    if left_value < right_value:
                        self._start_turn_right(corner=False, reason="both danger, LEFT closer")
                    else:
                        self._start_turn_left(corner=False, reason="both danger, RIGHT closer")

        elif self.state in (self.STATE_TURN_LEFT, self.STATE_TURN_RIGHT):
            self.turn_steps_remaining -= 1
            if self.turn_steps_remaining <= 0:
                self.state = self.STATE_FORWARD
                self.get_logger().info('STATE_FORWARD (finished timed turn)')

        lin_x = 0.0
        ang_z = 0.0

        if self.state == self.STATE_FORWARD:
            # forward speed
            lin_x = self.forward_speed
            ang_z = 0.0

            # arc away before getting near threshold
            soft_left = left_value < self.soft_distance
            soft_right = right_value < self.soft_distance

            if soft_left or soft_right:
                lin_x = self.forward_speed * 0.7

                diff = right_value - left_value
                diff_deadband = 0.012  # reduce jitter

                if abs(diff) > diff_deadband:
                    if diff < 0:
                        # left is closer than right -> curve right
                        ang_z = -self.turn_speed * 0.4
                    else:
                        # right is closer than left -> curve left
                        ang_z = self.turn_speed * 0.4

        elif self.state == self.STATE_TURN_LEFT:
            lin_x = 0.0
            ang_z = self.turn_speed

        elif self.state == self.STATE_TURN_RIGHT:
            lin_x = 0.0
            ang_z = -self.turn_speed

        self._publish_cmd(lin_x, ang_z)

    # Helpers
    def _start_turn_left(self, corner: bool, reason: str = ""):
        self.state = self.STATE_TURN_LEFT
        self.turn_steps_remaining = self.corner_turn_steps if corner else self.turn_steps_default
        self.get_logger().info(
            f'STATE_TURN_LEFT ({reason}), steps={self.turn_steps_remaining}'
        )

    def _start_turn_right(self, corner: bool, reason: str = ""):
        self.state = self.STATE_TURN_RIGHT
        self.turn_steps_remaining = self.corner_turn_steps if corner else self.turn_steps_default
        self.get_logger().info(
            f'STATE_TURN_RIGHT ({reason}), steps={self.turn_steps_remaining}'
        )

    def _publish_cmd(self, lin_x: float, ang_z: float):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist.linear.x = lin_x
        stamped.twist.angular.z = ang_z
        self.cmd_pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = WallAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()