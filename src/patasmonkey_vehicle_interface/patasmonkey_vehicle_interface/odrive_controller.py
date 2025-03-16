from .odrive_utils import ODriveUtils
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    AXIS_STATE_IDLE,
    INPUT_MODE_VEL_RAMP,
)
import multiprocessing


class MotorController:
    def __init__(self, axis_index=0):
        """
        ODrive motor controller class.
        :param axis_index: 0 (left motor) or 1 (right motor)
        """
        self.axis_index = axis_index
        self.shared_data = multiprocessing.Manager().dict()  # create shared dictionary

        # run ODrive init in a separate process
        self.init_process = multiprocessing.Process(
            target=self.init_odrive, args=(self.shared_data,)
        )
        self.init_process.start()
        self.init_process.join()  # wait for the process to complete

        # retrieve ODrive instances from shared memory
        self.odrive = self.shared_data.get("odrive", None)
        self.axis = self.shared_data.get("axis", None)
        if self.odrive is None or self.axis is None:
            raise RuntimeError(f"Motor {self.axis_index}: Failed to initialize ODrive!")

        # terminate the process
        self.init_process.terminate()

    def init_odrive(self, shared_data):
        """ODrive initialization process in a separate process"""
        print(f"[Motor {self.axis_index}] searching for ODrive...")
        odrv = ODriveUtils.find_odrive()
        ODriveUtils.clear_odrive_errors(odrv)
        axis = odrv.axis0 if self.axis_index == 0 else odrv.axis1

        # initialize motor as ramped velocity control mode
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.vel_ramp_rate = 5
        axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        axis.controller.config.pos_gain = 20
        axis.controller.config.vel_gain = 0.3
        axis.controller.config.vel_integrator_gain = 0.32
        axis.controller.config.vel_integrator_limit = 1

        # pass ODrive instances to the parent process
        shared_data["odrive"] = odrv
        shared_data["axis"] = axis
        print(f"[Motor {self.axis_index}] initialized in velocity control mode.")

    def select_axis(self):
        """Select the ODrive axis based on the index."""
        if self.axis_index == 0:
            return self.odrive.axis0
        elif self.axis_index == 1:
            return self.odrive.axis1
        else:
            raise ValueError("Invalid axis_index! Use 0 or 1.")

    def init_motor(self):
        """Initialize motor: closed-loop control & ramped velocity mode."""
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.vel_ramp_rate = 5
        self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.axis.controller.config.pos_gain = 20
        self.axis.controller.config.vel_gain = 0.3
        self.axis.controller.config.vel_integrator_gain = 0.32
        self.axis.controller.config.vel_integrator_limit = 1
        print(f"Motor {self.axis_index}: Initialized in velocity control mode.")

    def get_velocity(self):
        """Get the current motor velocity [rps]."""
        vel = self.axis.encoder.vel_estimate
        print(f"Motor {self.axis_index}: Current velocity = {vel:.2f} rps")
        return vel

    def set_velocity(self, velocity):
        """Set target velocity [rps]."""
        try:
            self.axis.controller.input_vel = velocity
            print(f"Motor {self.axis_index}: Velocity set to {velocity:.2f} rps.")
        except Exception as e:
            print(f"Error setting velocity: {e}")

    def stop(self):
        """Emergency stop: Set velocity to zero."""
        try:
            self.set_velocity(0.0)
            print(f"Motor {self.axis_index}: Emergency stop activated.")
        except Exception as e:
            print(f"Error during emergency stop: {e}")

    def set_idle(self):
        """Set the motor to idle (disable control)."""
        try:
            self.axis.requested_state = AXIS_STATE_IDLE
            print(f"Motor {self.axis_index}: Set to idle mode.")
        except Exception as e:
            print(f"Error setting idle mode: {e}")

    def check_errors(self):
        """Check and print ODrive error status."""
        ODriveUtils.check_odrive_errors(self.odrive)

    def reboot(self):
        """Reboot the ODrive device."""
        ODriveUtils.reboot_odrive(self.odrive)
