import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
from .odrive_controller import MotorController
import math
import sys


class VehicleInterfaceNode(Node):
    def __init__(self):
        super().__init__("vehicle_interface")  # register the node

        # load parameters from yaml (passed via launch file)
        self.whl_rad = self.get_parameter_or("whl_rad", 0.1)  # wheel diameter [m]
        self.whl_sep = self.get_parameter_or(
            "whl_sep", 0.4
        )  # wheel separation betwee L\R [m]
        self.gear_ratio = self.get_parameter_or("gear_ratio", 10.0)  # gear ratio
        self.max_whl_rps = self.get_parameter_or(
            "max_whl_rps", 4.0
        )  # max wheel velocity [rps]

        # get odrive config
        self.odrv_usb_port = self.get_parameter_or("odrv_usb_port", "/dev/ttyACM0")
        self.odrv_baud_rate = self.get_parameter_or("odrv_baud_rate", 115200)

        # odrive axis
        self.mtr_axis_l = self.get_parameter_or("mtr_axis_l", 0)
        self.mtr_axis_r = self.get_parameter_or("mtr_axis_r", 1)

        # get topic name that will be used
        self.cmd_vel_topic = self.get_parameter_or("cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_joy_topic = self.get_parameter_or(
            "cmd_vel_joy_topic", "/cmd_vel_joy"
        )
        self.mtr_output_topic = self.get_parameter_or("mtr_output_topic", "/motor_cmd")
        self.emergency_stop_topic = self.get_parameter_or(
            "emergency_stop_topic", "/emergency_stop"
        )

        # connect to odrive
        self.get_logger().info("connecting to ODrive...")
        self.left_motor = MotorController(self.mtr_axis_l)
        self.right_motor = MotorController(self.mtr_axis_r)
        self.get_logger().info("ODrive connected!")

        # subscriber config
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(
            Twist, self.cmd_vel_joy_topic, self.cmd_vel_callback, 10
        )
        self.create_subscription(
            Bool, self.emergency_stop_topic, self.emergency_stop_callback, 10
        )

        # publihser config
        self.motor_cmd_pub = self.create_publisher(
            Float32MultiArray, self.mtr_output_topic, 10
        )

    def cmd_vel_callback(self, msg):
        """convert cmd_vel to motor speed and send to ODrive"""
        lin_x = msg.linear.x  # velocity (forward/backward)
        ang_z = msg.angular.z  # velocity (turning)
        whl_diam = self.whl_rad * 2 * math.pi  # wheel diameter

        # compute rps of L/R wheels
        v_left = (lin_x - ang_z * self.whl_sep / 2) / whl_diam
        v_right = (lin_x + ang_z * self.whl_sep / 2) / whl_diam

        # cap by max speed
        v_left = max(min(v_left, self.max_whl_rps), -self.max_whl_rps)
        v_right = max(min(v_right, self.max_whl_rps), -self.max_whl_rps)

        # convert to motor rps
        mtr_left_rps = v_left * self.gear_ratio
        mtr_right_rps = v_right * self.gear_ratio

        # send command to ODrive
        self.left_motor.set_velocity(mtr_left_rps)
        self.right_motor.set_velocity(mtr_right_rps)

        # publish /motor_cmd
        msg_out = Float32MultiArray()
        msg_out.data = [mtr_left_rps, mtr_right_rps]
        self.motor_cmd_pub.publish(msg_out)

    def emergency_stop_callback(self, msg):
        """emefgency stop: stop motors immediately"""
        if msg.data:
            self.left_motor.set_idle()
            self.right_motor.set_idle()
            self.get_logger().warn("Emergency STOP activated!")


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = VehicleInterfaceNode()  # Create node instance

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")  # Log clean shutdown
    finally:
        if rclpy.ok():  # Prevent multiple shutdown calls
            node.destroy_node()  # Destroy node properly
            rclpy.shutdown()  # Shutdown ROS2 cleanly
        sys.exit(0)  # Exit without error


if __name__ == "__main__":
    main()
