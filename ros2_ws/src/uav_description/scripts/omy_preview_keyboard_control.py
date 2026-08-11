#!/usr/bin/env python3

import atexit
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class OmyPreviewKeyboardControl(Node):
    """Terminal keyboard controls for the RViz preview follower."""

    def __init__(self):
        super().__init__('omy_preview_keyboard_control')
        self.declare_parameter('set_enabled_service', '/omy_preview_tracking/set_enabled')
        self.declare_parameter('reset_service', '/omy_preview_tracking/reset')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('poll_rate', 20.0)

        set_enabled_service = self.get_parameter('set_enabled_service').value
        reset_service = self.get_parameter('reset_service').value
        poll_rate = float(self.get_parameter('poll_rate').value)
        self.tracking_enabled = bool(self.get_parameter('start_enabled').value)

        self.set_enabled_client = self.create_client(SetBool, set_enabled_service)
        self.reset_client = self.create_client(Trigger, reset_service)
        self.terminal_settings = None
        self.terminal_ready = False
        self.input_stream = None
        self.tty_file = None

        self.open_terminal_input()
        self.create_timer(1.0 / poll_rate, self.poll_keyboard)

    def open_terminal_input(self):
        if sys.stdin.isatty():
            self.input_stream = sys.stdin
            source = 'stdin'
        else:
            try:
                self.tty_file = open('/dev/tty', 'r', buffering=1)
                self.input_stream = self.tty_file
                source = '/dev/tty'
            except OSError as exc:
                self.get_logger().warn(
                    f'No terminal input available ({exc}); use /omy_preview_tracking services instead.'
                )
                return

        self.terminal_settings = termios.tcgetattr(self.input_stream)
        tty.setcbreak(self.input_stream.fileno())
        atexit.register(self.restore_terminal)
        self.terminal_ready = True
        self.get_logger().info(
            f'Keyboard ready on {source}: SPACE toggles tracking, r resets OMY to initial pose.'
        )

    def restore_terminal(self):
        if self.terminal_settings is not None and self.input_stream is not None:
            termios.tcsetattr(self.input_stream, termios.TCSADRAIN, self.terminal_settings)
            self.terminal_settings = None
        if self.tty_file is not None:
            self.tty_file.close()
            self.tty_file = None

    def poll_keyboard(self):
        if not self.terminal_ready or self.input_stream is None:
            return
        ready, _, _ = select.select([self.input_stream], [], [], 0.0)
        if not ready:
            return

        key = self.input_stream.read(1)
        if key == ' ':
            self.tracking_enabled = not self.tracking_enabled
            self.call_set_enabled(self.tracking_enabled)
        elif key.lower() == 'r':
            self.tracking_enabled = False
            self.call_reset()

    def call_set_enabled(self, enabled):
        if not self.set_enabled_client.service_is_ready():
            self.get_logger().warn('Tracking service is not ready yet.')
            return
        request = SetBool.Request()
        request.data = enabled
        future = self.set_enabled_client.call_async(request)
        future.add_done_callback(lambda done: self.log_response(done, 'tracking'))

    def call_reset(self):
        if not self.reset_client.service_is_ready():
            self.get_logger().warn('Reset service is not ready yet.')
            return
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self.log_response(done, 'reset'))

    def log_response(self, future, action):
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging for ROS service errors.
            self.get_logger().error(f'{action} service call failed: {exc}')
            return
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)


def main(args=None):
    rclpy.init(args=args)
    node = OmyPreviewKeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
