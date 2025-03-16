import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    AXIS_STATE_IDLE,
    INPUT_MODE_VEL_RAMP,
)
import time
import threading
import math
import sys


# def initialize_odrive(queue):
#     """ ODrive の初期化を `rclpy.init()` の前に独立プロセスで実行 """
#     print("[ODrive Init] Connecting to ODrive...")
#     odrv0 = odrive.find_any(timeout=3)  # ODrive を探す
#     if odrv0 is None:
#         print("[ODrive Init] ODrive connection failed!")
#         queue.put(None)
#         return

#     odrv0.clear_errors()  # エラーをクリア
#     print("[ODrive Init] ODrive connected!")

#     # ODrive 設定
#     odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
#     odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
#     odrv0.axis0.controller.config.vel_ramp_rate = 5
#     odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
#     odrv0.axis0.controller.config.pos_gain = 20
#     odrv0.axis0.controller.config.vel_gain = 0.3
#     odrv0.axis0.controller.config.vel_integrator_gain = 0.32
#     odrv0.axis0.controller.config.vel_integrator_limit = 1
#     print(odrv0.axis0.controller.config.vel_integrator_limit)

#     odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
#     odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
#     odrv0.axis1.controller.config.vel_ramp_rate = 5
#     odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
#     odrv0.axis1.controller.config.pos_gain = 20
#     odrv0.axis1.controller.config.vel_gain = 0.3
#     odrv0.axis1.controller.config.vel_integrator_gain = 0.32
#     odrv0.axis1.controller.config.vel_integrator_limit = 1
#     print(odrv0.axis1.controller.config.vel_integrator_limit)

#     # 初期化が成功した ODrive インスタンスをキューに渡す
#     queue.put(odrv0)


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
        self.odrv_usb_port = self.get_parameter_or(
            "odrv_usb_port", "/dev/ttyACM0"
        )
        self.odrv_baud_rate = self.get_parameter_or("odrv_baud_rate", 115200)

        # odrive axis
        self.mtr_axis_l = self.get_parameter_or("mtr_axis_l", 0)
        self.mtr_axis_r = self.get_parameter_or("mtr_axis_r", 1)

        # get topic name that will be used
        self.cmd_vel_topic = self.get_parameter_or("cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_joy_topic = self.get_parameter_or(
            "cmd_vel_joy_topic", "/cmd_vel_joy"
        )
        self.mtr_output_topic = self.get_parameter_or(
            "mtr_output_topic", "/motor_cmd"
        )
        self.emergency_stop_topic = self.get_parameter_or(
            "emergency_stop_topic", "/emergency_stop"
        )

        # ODrive の初期化フラグ
        self.odrive_ready = threading.Event()
        # ODrive ステータス監視用のタイマー
        self.timer = self.create_timer(0.5, self.check_odrive_status)

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


    def initialize_odrive(self):
        """ ODrive の初期化をバックグラウンドスレッドで非同期実行 """
        self.get_logger().info("Connecting to ODrive...")
        odrv0 = odrive.find_any(timeout=5)  # ODrive を探す

        if odrv0 is None:
            self.get_logger().error("ODrive connection failed! Exiting...")
            sys.exit(1)

        odrv0.clear_errors()
        self.get_logger().info("ODrive connected!")

        self.left_motor = odrv0.axis0
        self.right_motor = odrv0.axis1

        # ODrive の設定を別スレッドで確実に実行
        self.set_control_thread = threading.Thread(target=self.set_control_mode, daemon=True)
        self.set_control_thread.start()
        self.get_logger().info("ODrive configured!")

    
    def check_odrive_status(self):
        """ ODrive の状態を監視する (0.5秒ごとに実行) """
        if self.odrive_ready:
            self.get_logger().info("ODrive is operational.")


    def set_control_mode(self):
        # set left motor to ramped velocity control mode
        self.left_motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.left_motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.left_motor.controller.config.vel_ramp_rate = 5
        self.left_motor.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.left_motor.controller.config.pos_gain = 20
        self.left_motor.controller.config.vel_gain = 0.3
        self.left_motor.controller.config.vel_integrator_gain = 0.32
        print(self.left_motor.controller.config.vel_integrator_limit)
        self.left_motor.controller.config.vel_integrator_limit = 2
        print(self.left_motor.controller.config.vel_integrator_limit)
        # **設定完了フラグをセット**
        self.odrive_ready.set()

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
    """ ODrive 初期化後に ROS2 を起動 """

    rclpy.init(args=args)  # **ROS2 の初期化**
    node = VehicleInterfaceNode()  # **ROS2 ノードを起動**
    executor = SingleThreadedExecutor()  # **シングルスレッドのエグゼキュータを使用**
    executor.add_node(node)

    # **ODrive の初期化を待機しながら spin_once() を実行**
    node.get_logger().info("Waiting for ODrive initialization...")

    timeout = 10  # 最大待機時間 (秒)
    start_time = time.time()

    while not node.odrive_ready.is_set():
        executor.spin_once(timeout_sec=0.1)  # **ROS2のイベント処理を適宜回しながら待機**
        
        if time.time() - start_time > timeout:
            node.get_logger().error("ODrive initialization timed out! Exiting...")
            sys.exit(1)

    node.get_logger().info("ODrive is ready. Starting ROS2 loop...")

    try:
        executor.spin()  # **通常のイベントループを回す**
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()
