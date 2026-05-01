#!/usr/bin/env python3

import logging
import math
import argparse
import colorlog
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from pynput import keyboard


class KeyboardPublisher(Node):
    """
    ROS2 node that reads WASD keyboard input and publishes
    geometry_msgs/Twist messages to the 'vehicle_cmd' topic.

    Topic contract (matches LowLevelController):
      linear.x  : speed          [-1.0 ... +1.0]
      angular.z : steering angle [-pi/2 ... +pi/2] rad
    """

    PUBLISH_RATE_HZ = 10     # Hz

    MAX_SPEED   =  1.0
    MIN_SPEED   = -1.0
    DECEL_STEP  =  0.05      # speed step applied each tick when no W/S held
    MAX_STEER   =  math.pi / 2   # ~1.5708 rad  (full right)
    MIN_STEER   = -math.pi / 2   # ~-1.5708 rad (full left)

    def __init__(self, verbose: bool = True):
        super().__init__('keyboard_publisher')
        self.get_logger().set_level(LoggingSeverity.FATAL)
        self._setup_logger(verbose)

        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._timer = self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._timer_callback)

        # Key state
        self._press_w = False
        self._press_a = False
        self._press_s = False
        self._press_d = False

        # Current command values
        self._speed   = 0.0
        self._steer   = 0.0   # radians

        # Start keyboard listener in a background thread
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

        self.logger.info(
            'KeyboardPublisher started — publishing to "cmd_vel"\n'
            '  W / S : forward / reverse\n'
            '  A / D : steer left / right\n'
            '  ESC   : quit'
        )

    # ------------------------------------------------------------------
    # Logger
    # ------------------------------------------------------------------

    def _setup_logger(self, verbose: bool = True):
        self.logger = logging.getLogger(self.get_name())
        self.logger.setLevel(logging.DEBUG)
        self.logger.handlers.clear()

        if not verbose:
            self.logger.addHandler(logging.NullHandler())
            return

        handler = logging.StreamHandler()
        handler.setFormatter(colorlog.ColoredFormatter(
            fmt='%(log_color)s%(levelname)-8s%(reset)s '
                '%(blue)s[%(asctime)s]%(reset)s '
                '%(cyan)s[%(name)s]%(reset)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            log_colors={
                'DEBUG':    'cyan',
                'INFO':     'green',
                'WARNING':  'yellow',
                'ERROR':    'red',
                'CRITICAL': 'red,bg_white',
            },
        ))
        self.logger.addHandler(handler)

    # ------------------------------------------------------------------
    # Keyboard callbacks  (run in pynput thread)
    # ------------------------------------------------------------------

    def _on_press(self, key):
        try:
            ch = key.char
            if ch == 'w': self._press_w = True
            if ch == 'a': self._press_a = True
            if ch == 's': self._press_s = True
            if ch == 'd': self._press_d = True
        except AttributeError:
            pass  # special key — ignore

    def _on_release(self, key):
        if key == keyboard.Key.esc:
            self.logger.warning('ESC pressed — shutting down.')
            # Publish stop BEFORE invalidating the context
            stop_msg = Twist()
            self._publisher.publish(stop_msg)
            rclpy.shutdown()
            return False  # stop listener
        try:
            ch = key.char
            if ch == 'w': self._press_w = False
            if ch == 'a': self._press_a = False
            if ch == 's': self._press_s = False
            if ch == 'd': self._press_d = False
        except AttributeError:
            pass

    # ------------------------------------------------------------------
    # Timer callback  (runs in ROS2 executor thread)
    # ------------------------------------------------------------------

    def _timer_callback(self):
        self._update_state()
        self._publish()

    def _update_state(self):
        # Speed
        if self._press_w:
            self._speed = self.MAX_SPEED
        elif self._press_s:
            self._speed = self.MIN_SPEED
        else:
            # Decelerate toward zero
            if self._speed > 0.0:
                self._speed = max(0.0, self._speed - self.DECEL_STEP)
            elif self._speed < 0.0:
                self._speed = min(0.0, self._speed + self.DECEL_STEP)

        # Steering
        if self._press_a and not self._press_d:
            self._steer = self.MIN_STEER
        elif self._press_d and not self._press_a:
            self._steer = self.MAX_STEER
        else:
            self._steer = 0.0

    def _publish(self):
        msg = Twist()
        msg.linear.x  = self._speed
        msg.angular.z = self._steer
        self._publisher.publish(msg)
        self.logger.debug(
            f'Published → speed: {self._speed:+.3f}  |  steer: {self._steer:+.4f} rad'
        )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._listener.stop()
        # Send a zero command before exiting so the car stops.
        # Guard with try/except: on Ctrl+C, rclpy's SIGINT handler shuts down
        # the context before this runs, making publish() raise RCLError.
        try:
            stop_msg = Twist()
            self._publisher.publish(stop_msg)
            self.logger.info('KeyboardPublisher shutting down — zero command sent.')
        except Exception:
            self.logger.warning('KeyboardPublisher shutting down — context already invalid, stop command not sent.')
        super().destroy_node()


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    parser = argparse.ArgumentParser(
        description='Keyboard publisher node — controls the car via WASD keys over ROS2.'
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='Suppress all log output'
    )
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else args)
    node = KeyboardPublisher(verbose=not parsed.quiet)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
