import os

# Fail-safe: se non è stato scelto esplicitamente altro,
# ROS 2 comunica solamente sulla macchina locale.
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

# print(
#     "ROS 2 network mode:",
#     "LOCALHOST" if os.environ["ROS_LOCALHOST_ONLY"] == "1" else "NETWORK",
# )

import sys
import shlex
import subprocess
from pathlib import Path

dir_path = Path(__file__).resolve().parent
sys.path.append(str(dir_path / ".."))

ros_ws = dir_path / "ros2_ws"
setup_bash = ros_ws / "install" / "setup.bash"

if not setup_bash.exists():
    # print("Building the msgs first...")
    subprocess.run(["colcon", "build"], cwd=ros_ws, check=True)

if os.environ.get("BASIC_LOCOMOTION_ROS2_SOURCED") != "1":
    # print("Sourcing ROS2 workspace and restarting script...")
    cmd = (
        f"source {shlex.quote(str(setup_bash))} && "
        "export BASIC_LOCOMOTION_ROS2_SOURCED=1 && "
        f"exec {shlex.quote(sys.executable)} "
        + " ".join(shlex.quote(arg) for arg in [str(Path(__file__).resolve()), *sys.argv[1:]])
    )
    os.execv("/bin/bash", ["bash", "-c", cmd])


# ROS 2 imports
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
from dls2_interface.msg import BaseState, BlindState, Imu, ControlSignal

# Python imports
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QVBoxLayout, QHBoxLayout
from PySide6.QtCore import Qt, QTimer




class AliengoDashboard(Node):
    def __init__(self):
        super().__init__('AliengoDashboard')

        self.subscription_base_state = self.create_subscription(BaseState,"/base_state", self.get_base_state_callback, 1)
        self.subscription_blind_state = self.create_subscription(BlindState,"/blind_state_legged", self.get_blind_state_callback, 1)
        self.subscription_imu = self.create_subscription(Imu,"/imu", self.get_imu_callback, 1)

        self.subscriber_control_signal = self.create_subscription(ControlSignal,"/control_signal_legged", self.get_control_signal_callback, 1)
        self.sequence_id = 0

        # Base State
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Blind State
        self.legs_joints_position = np.zeros(12)
        self.legs_joints_velocity = np.zeros(12)

        # IMU
        self.imu_linear_acceleration = np.zeros(3)
        self.imu_angular_velocity = np.zeros(3)
        self.imu_orientation = np.zeros(4)

        # Control Signal
        # --Desired PD
        self.desired_joints_position = np.zeros(12)
        self.desired_joints_velocity = np.zeros(12)

        # --Desired gains
        self.Kp = 0
        self.Kd = 0

        # self.print_timer = self.create_timer(0.1, self.print_data)


    def get_base_state_callback(self, msg):
        self.position = np.array(msg.pose.position) #world frame
        # For the quaternion, the order is [x, y, z, w] on DLS2 but here we want [w, x, y, z] (mujoco convention)
        self.orientation = np.roll(np.array(msg.pose.orientation), 1) #world frame
        self.linear_velocity = np.array(msg.velocity.linear) #world frame
        self.angular_velocity = np.array(msg.velocity.angular) #base frame


    def get_blind_state_callback(self, msg):
        self.legs_joints_position = np.array(msg.joints_position)
        self.legs_joints_velocity = np.array(msg.joints_velocity)
     
        
    def get_imu_callback(self, msg):
        self.imu_linear_acceleration = np.array(msg.linear_acceleration) 
        self.imu_angular_velocity = np.array(msg.angular_velocity) 
        # For the quaternion, the order is [x, y, z, w] on DLS2 but here we want [w, x, y, z] (mujoco convention)
        self.imu_orientation = np.roll(np.array(msg.orientation), 1) 

    def get_control_signal_callback(self, msg):

        self.desired_joints_position = np.array(msg.joints_position)

        self.Kp = np.array(msg.kp)[0]
        self.Kd = np.array(msg.kd)[0]

    def getData(self):
        data = {
            "BaseState": {  
                "position":self.position,
                "orientation":self.orientation,
                "linear_velocity":self.linear_velocity,
                "angular_velocity":self.angular_velocity
            },
            "BlindState": {
                "jointpositions":self.legs_joints_position,
                "jointvelocities":self.legs_joints_velocity
            },
            "IMU": {
                "linear_acceleration":self.imu_linear_acceleration,
                "angular_velocity":self.imu_angular_velocity,
                "orientation":self.imu_orientation
            }
        }
        return data

    def print_data(self):
        data = f'''
        BaseState:
            Position: {self.position}
            Orientation: {self.orientation}
            Linear velocity: {self.linear_velocity}
            Angular velocity: {self.angular_velocity}
        BlindState:
            Legs joint positions: {self.legs_joints_position}
            Legs joint velocity: {self.legs_joints_velocity}
        IMU:
            IMU linear acceleration: {self.imu_linear_acceleration}
            IMU angular velocity: {self.imu_angular_velocity}
            IMU orientation: {self.imu_orientation}
        '''
        print(f'{data}', end="", flush=True)
        print('\033[14F', end='')

        
class AliengoUI(QMainWindow):
    def __init__(self, rosnode):
        super().__init__()

        self.rosnode = rosnode


        self.setWindowTitle("Aliengo Dashboard")

        self.container = QWidget()
        self.setCentralWidget(self.container)

        self.layout = QVBoxLayout(self.container)

        self.BaseStateLabel = QLabel()
        self.BaseStateLabel.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.BaseStateLabel)

        self.BlindStateLayout = QHBoxLayout()
        self.layout.addLayout(self.BlindStateLayout)

        self.BlindStateLabel = QLabel()
        self.BlindStateLabel.setAlignment(Qt.AlignCenter)
        self.BlindStateLayout.addWidget(self.BlindStateLabel)

        self.IMULabel = QLabel()
        self.IMULabel.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.IMULabel)

        self.UItimer = QTimer()
        self.UItimer.timeout.connect(self.updateUI)
        self.UItimer.start(100)

        self.jointnames = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
        self.jointpositions = {}
        self.jointvelocities = {}
        x = 0
        for i in self.rosnode.getData()["BlindState"]["jointpositions"]:
            self.jointpositions[self.jointnames[x]] = i
            x+=1
        print(self.jointpositions)
        x=0
        for i in self.rosnode.getData()["BlindState"]["jointvelocities"]:
            self.jointvelocities[self.jointnames[x]] = i
            x+=1
        print(self.jointvelocities)
            
            


        


    def updateUI(self):
        data = self.rosnode.getData()
        
        #['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']


#         BaseStateText = f"""
# BaseState
# Position: {data["BaseState"]["position"]}
# Orientation: {data["BaseState"]["orientation"]}
# Linear velocity: {data["BaseState"]["linear_velocity"]}
# Angular velocity: {data["BaseState"]["angular_velocity"]}

# BlindState
# Joint positions: {data["BlindState"]["jointpositions"]}
# Joint velocities: {data["BlindState"]["jointvelocities"]}

# IMU
# Linear acceleration: {data["IMU"]["linear_acceleration"]}
# Angular velocity: {data["IMU"]["angular_velocity"]}
# Orientation: {data["IMU"]["orientation"]}
# """
        BaseStateText = f"""
BaseState
Position: {data["BaseState"]["position"]}
Orientation: {data["BaseState"]["orientation"]}
Linear velocity: {data["BaseState"]["linear_velocity"]}
Angular velocity: {data["BaseState"]["angular_velocity"]}
"""
        self.BaseStateLabel.setText(BaseStateText)

        BlindStateText

        



def main():
    print('aliengo dashboard node initialized!1!11!')
    rclpy.init()

    aliengo_dashboard_node = AliengoDashboard()

    app = QApplication(sys.argv)
    dashboard = AliengoUI(aliengo_dashboard_node)
    dashboard.show()

    rosspin = QTimer()
    rosspin.timeout.connect(lambda: rclpy.spin_once(aliengo_dashboard_node, timeout_sec=0))
    rosspin.start(10)
    try:
        exit_code = app.exec()
    finally:
        aliengo_dashboard_node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()


