"""ROS glue shared by ol_calibrate and velocity_control."""
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray

from robot_control.ol_primitive import PrimitiveRunner


def init(args=None):
    # Keep the context alive on Ctrl-C so we can still publish the stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)


class MotorNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

    def param(self, name, default):
        # dynamic typing so -p x:=1 and -p x:=1.0 (or [2, 4]) both work
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value

    def float_param(self, name, default):
        return float(self.param(name, default))

    def floats_param(self, name, default):
        v = self.param(name, default)
        return [float(x) for x in v] if hasattr(v, '__iter__') else [float(v)]

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.pub.publish(msg)

    def runner(self, settle_s):
        return PrimitiveRunner(self.send, settle_s)


def shutdown(node):
    if node is not None:
        try:
            node.runner(0.0).stop()
        except Exception:
            pass  # motor_control's watchdog stops anyway
        node.destroy_node()
    rclpy.try_shutdown()
