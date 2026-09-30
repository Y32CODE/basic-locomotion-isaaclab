import os
import sys
import shlex
import subprocess
from pathlib import Path

# ============================================================
# ROS 2 ENVIRONMENT
# ============================================================

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

dir_path = Path(__file__).resolve().parent
sys.path.append(str(dir_path / ".."))

ros_ws = dir_path / "ros2_ws"
setup_bash = ros_ws / "install" / "setup.bash"

if not setup_bash.exists():
    subprocess.run(
        ["colcon", "build"],
        cwd=ros_ws,
        check=True
    )

if os.environ.get("BASIC_LOCOMOTION_ROS2_SOURCED") != "1":
    cmd = (
        f"source {shlex.quote(str(setup_bash))} && "
        "export BASIC_LOCOMOTION_ROS2_SOURCED=1 && "
        f"exec {shlex.quote(sys.executable)} "
        + " ".join(
            shlex.quote(arg)
            for arg in [
                str(Path(__file__).resolve()),
                *sys.argv[1:]
            ]
        )
    )

    os.execv(
        "/bin/bash",
        ["bash", "-c", cmd]
    )


# ============================================================
# ROS 2 IMPORTS
# ============================================================

import rclpy
from rclpy.node import Node

from dls2_interface.msg import (
    BaseState,
    BlindState,
    Imu,
    ControlSignal,
)


# ============================================================
# PYTHON IMPORTS
# ============================================================

import numpy as np

np.set_printoptions(
    precision=3,
    suppress=True
)


# ============================================================
# QT IMPORTS
# ============================================================

from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QFrame,
    QSizePolicy,
)

from PySide6.QtCore import (
    Qt,
    QTimer,
)


# ============================================================
# ROS NODE
# ============================================================

class AliengoDashboard(Node):

    def __init__(self):
        super().__init__("AliengoDashboard")

        # ----------------------------------------------------
        # ROS SUBSCRIPTIONS
        # ----------------------------------------------------

        self.subscription_base_state = self.create_subscription(
            BaseState,
            "/base_state",
            self.get_base_state_callback,
            1
        )

        self.subscription_blind_state = self.create_subscription(
            BlindState,
            "/blind_state_legged",
            self.get_blind_state_callback,
            1
        )

        self.subscription_imu = self.create_subscription(
            Imu,
            "/imu",
            self.get_imu_callback,
            1
        )

        self.subscription_control_signal = self.create_subscription(
            ControlSignal,
            "/control_signal_legged",
            self.get_control_signal_callback,
            1
        )

        # ----------------------------------------------------
        # BASE STATE
        # ----------------------------------------------------

        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # ----------------------------------------------------
        # BLIND STATE
        # ----------------------------------------------------

        self.legs_joints_position = np.zeros(12)
        self.legs_joints_velocity = np.zeros(12)

        # ----------------------------------------------------
        # IMU
        # ----------------------------------------------------

        self.imu_linear_acceleration = np.zeros(3)
        self.imu_angular_velocity = np.zeros(3)
        self.imu_orientation = np.zeros(4)

        # ----------------------------------------------------
        # CONTROL SIGNAL
        # ----------------------------------------------------

        self.desired_joints_position = np.zeros(12)
        self.desired_joints_velocity = np.zeros(12)

        self.Kp = 0
        self.Kd = 0

    # ========================================================
    # CALLBACKS
    # ========================================================

    def get_base_state_callback(self, msg):

        self.position = np.array(
            msg.pose.position
        )

        # DLS2: [x, y, z, w]
        # Dashboard: [w, x, y, z]

        self.orientation = np.roll(
            np.array(msg.pose.orientation),
            1
        )

        self.linear_velocity = np.array(
            msg.velocity.linear
        )

        self.angular_velocity = np.array(
            msg.velocity.angular
        )

    # --------------------------------------------------------

    def get_blind_state_callback(self, msg):

        self.legs_joints_position = np.array(
            msg.joints_position
        )

        self.legs_joints_velocity = np.array(
            msg.joints_velocity
        )

    # --------------------------------------------------------

    def get_imu_callback(self, msg):

        self.imu_linear_acceleration = np.array(
            msg.linear_acceleration
        )

        self.imu_angular_velocity = np.array(
            msg.angular_velocity
        )

        self.imu_orientation = np.roll(
            np.array(msg.orientation),
            1
        )

    # --------------------------------------------------------

    def get_control_signal_callback(self, msg):

        self.desired_joints_position = np.array(
            msg.joints_position
        )

        self.Kp = np.array(msg.kp)[0]
        self.Kd = np.array(msg.kd)[0]

    # ========================================================
    # DATA
    # ========================================================

    def getData(self):

        return {
            "BaseState": {
                "position": self.position,
                "orientation": self.orientation,
                "linear_velocity": self.linear_velocity,
                "angular_velocity": self.angular_velocity,
            },

            "BlindState": {
                "jointpositions": self.legs_joints_position,
                "jointvelocities": self.legs_joints_velocity,
            },

            "IMU": {
                "linear_acceleration": self.imu_linear_acceleration,
                "angular_velocity": self.imu_angular_velocity,
                "orientation": self.imu_orientation,
            },

            "ControlSignal": {
                "desired_joints_position": self.desired_joints_position,
                "desired_joints_velocity": self.desired_joints_velocity,
                "Kp": self.Kp,
                "Kd": self.Kd,
            }
        }


# ============================================================
# ALIENGO UI
# ============================================================

class AliengoUI(QMainWindow):

    def __init__(self, rosnode):
        super().__init__()

        self.rosnode = rosnode

        self.setWindowTitle(
            "Aliengo Dashboard"
        )

        self.resize(
            1400,
            850
        )

        # ----------------------------------------------------
        # JOINT NAMES
        # ----------------------------------------------------

        self.jointnames = [
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",

            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",

            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",

            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ]

        self.setupUI()

        # ----------------------------------------------------
        # UI TIMER
        # ----------------------------------------------------

        self.UItimer = QTimer()

        self.UItimer.timeout.connect(
            self.updateUI
        )

        self.UItimer.start(100)

    # ========================================================
    # SETUP UI
    # ========================================================

    def setupUI(self):

        self.setStyleSheet("""
            QMainWindow {
                background-color: #101216;
            }

            QLabel {
                color: #E8EAED;
                font-family: "Inter", "Segoe UI";
            }

            QLabel#Title {
                font-size: 26px;
                font-weight: 700;
            }

            QLabel#SectionTitle {
                font-size: 15px;
                font-weight: 700;
                color: #9AA4B2;
            }

            QLabel#Status {
                font-size: 13px;
                font-weight: 600;
                color: #65D68A;
            }

            QFrame#Card {
                background-color: #181B21;
                border: 1px solid #292E38;
                border-radius: 12px;
            }

            QLabel#Value {
                font-size: 14px;
                color: #D7DBE2;
            }

            QLabel#JointName {
                font-size: 13px;
                font-weight: 600;
                color: #AAB2C0;
            }

            QLabel#JointValue {
                font-size: 14px;
                font-weight: 500;
                color: #F1F3F5;
            }
        """)

        # ====================================================
        # CENTRAL WIDGET
        # ====================================================

        self.container = QWidget()

        self.setCentralWidget(
            self.container
        )

        self.mainLayout = QVBoxLayout(
            self.container
        )

        self.mainLayout.setContentsMargins(
            24,
            20,
            24,
            20
        )

        self.mainLayout.setSpacing(16)

        # ====================================================
        # HEADER
        # ====================================================

        header = QHBoxLayout()

        title = QLabel("ALIENGO")
        title.setObjectName("Title")

        subtitle = QLabel("ROBOT DASHBOARD")

        subtitle.setStyleSheet("""
            color: #697383;
            font-size: 12px;
            font-weight: 600;
        """)

        titleLayout = QVBoxLayout()
        titleLayout.setSpacing(0)

        titleLayout.addWidget(title)
        titleLayout.addWidget(subtitle)

        self.statusLabel = QLabel(
            "●  ROS CONNECTED"
        )

        self.statusLabel.setObjectName(
            "Status"
        )

        header.addLayout(titleLayout)
        header.addStretch()
        header.addWidget(self.statusLabel)

        self.mainLayout.addLayout(header)

        # ====================================================
        # TOP SECTION
        # ====================================================

        topLayout = QHBoxLayout()
        topLayout.setSpacing(16)

        # ====================================================
        # ROBOT CARD
        # ====================================================

        robotCard = self.createCard()

        robotLayout = QVBoxLayout(
            robotCard
        )

        robotTitle = QLabel("ROBOT")
        robotTitle.setObjectName("SectionTitle")

        robotLayout.addWidget(robotTitle)

        self.robotLabel = QLabel("ALIENGO")

        self.robotLabel.setAlignment(
            Qt.AlignCenter
        )

        self.robotLabel.setStyleSheet("""
            font-size: 42px;
            font-weight: 700;
            color: #E8EAED;
        """)

        robotLayout.addWidget(
            self.robotLabel
        )

        robotLayout.addStretch()

        topLayout.addWidget(
            robotCard,
            1
        )

        # ====================================================
        # BASE STATE CARD
        # ====================================================

        baseCard = self.createCard()

        baseLayout = QVBoxLayout(
            baseCard
        )

        baseTitle = QLabel("BASE STATE")
        baseTitle.setObjectName("SectionTitle")

        baseLayout.addWidget(baseTitle)

        self.basePosition = self.createValueLabel()
        self.baseOrientation = self.createValueLabel()
        self.baseLinearVelocity = self.createValueLabel()
        self.baseAngularVelocity = self.createValueLabel()

        baseLayout.addLayout(
            self.createRow(
                "Position",
                self.basePosition
            )
        )

        baseLayout.addLayout(
            self.createRow(
                "Orientation",
                self.baseOrientation
            )
        )

        baseLayout.addLayout(
            self.createRow(
                "Linear Velocity",
                self.baseLinearVelocity
            )
        )

        baseLayout.addLayout(
            self.createRow(
                "Angular Velocity",
                self.baseAngularVelocity
            )
        )

        topLayout.addWidget(
            baseCard,
            2
        )

        self.mainLayout.addLayout(
            topLayout
        )

        # ====================================================
        # JOINT STATE CARD
        # ====================================================

        jointCard = self.createCard()

        jointLayout = QVBoxLayout(
            jointCard
        )

        jointTitle = QLabel("JOINT STATE")
        jointTitle.setObjectName("SectionTitle")

        jointLayout.addWidget(jointTitle)

        self.jointGrid = QGridLayout()

        self.jointGrid.setHorizontalSpacing(12)
        self.jointGrid.setVerticalSpacing(8)

        headers = [
            "JOINT",
            "POSITION",
            "VELOCITY",
        ]

        for column, text in enumerate(headers):

            label = QLabel(text)

            label.setStyleSheet("""
                color: #697383;
                font-size: 11px;
                font-weight: 700;
            """)

            self.jointGrid.addWidget(
                label,
                0,
                column
            )

        self.jointLabels = {}

        for row, jointName in enumerate(
            self.jointnames,
            start=1
        ):

            name = QLabel(jointName)
            name.setObjectName("JointName")

            position = QLabel("0.000")
            position.setObjectName("JointValue")

            velocity = QLabel("0.000")
            velocity.setObjectName("JointValue")

            self.jointGrid.addWidget(
                name,
                row,
                0
            )

            self.jointGrid.addWidget(
                position,
                row,
                1
            )

            self.jointGrid.addWidget(
                velocity,
                row,
                2
            )

            self.jointLabels[jointName] = {
                "position": position,
                "velocity": velocity,
            }

        jointLayout.addLayout(
            self.jointGrid
        )

        self.mainLayout.addWidget(
            jointCard
        )

        # ====================================================
        # IMU CARD
        # ====================================================

        imuCard = self.createCard()

        imuLayout = QVBoxLayout(
            imuCard
        )

        imuTitle = QLabel("IMU")
        imuTitle.setObjectName("SectionTitle")

        imuLayout.addWidget(imuTitle)

        imuGrid = QGridLayout()

        imuGrid.setHorizontalSpacing(20)

        self.imuAcceleration = (
            self.createValueLabel()
        )

        self.imuAngularVelocity = (
            self.createValueLabel()
        )

        self.imuOrientation = (
            self.createValueLabel()
        )

        imuGrid.addWidget(
            self.createMetric(
                "LINEAR ACCELERATION",
                self.imuAcceleration
            ),
            0,
            0
        )

        imuGrid.addWidget(
            self.createMetric(
                "ANGULAR VELOCITY",
                self.imuAngularVelocity
            ),
            0,
            1
        )

        imuGrid.addWidget(
            self.createMetric(
                "ORIENTATION",
                self.imuOrientation
            ),
            0,
            2
        )

        imuLayout.addLayout(imuGrid)

        self.mainLayout.addWidget(imuCard)

        self.mainLayout.addStretch()

    # ========================================================
    # CREATE CARD
    # ========================================================

    def createCard(self):

        card = QFrame()

        card.setObjectName("Card")

        card.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Preferred
        )

        return card

    # ========================================================
    # CREATE VALUE LABEL
    # ========================================================

    def createValueLabel(self):

        label = QLabel(
            "0.000, 0.000, 0.000"
        )

        label.setObjectName("Value")

        return label

    # ========================================================
    # CREATE ROW
    # ========================================================

    def createRow(self, name, value):

        layout = QHBoxLayout()

        title = QLabel(name)

        title.setStyleSheet("""
            color: #697383;
            font-size: 12px;
            font-weight: 600;
        """)

        layout.addWidget(title)
        layout.addStretch()
        layout.addWidget(value)

        return layout

    # ========================================================
    # CREATE METRIC
    # ========================================================

    def createMetric(self, title, value):

        widget = QWidget()

        layout = QVBoxLayout(widget)

        layout.setContentsMargins(
            0,
            0,
            0,
            0
        )

        label = QLabel(title)

        label.setStyleSheet("""
            color: #697383;
            font-size: 10px;
            font-weight: 700;
        """)

        layout.addWidget(label)
        layout.addWidget(value)

        return widget

    # ========================================================
    # FORMAT VECTOR
    # ========================================================

    def formatVector(self, vector):

        return ", ".join(
            f"{value:.3f}"
            for value in vector
        )

    # ========================================================
    # UPDATE UI
    # ========================================================

    def updateUI(self):

        data = self.rosnode.getData()

        # ----------------------------------------------------
        # BASE STATE
        # ----------------------------------------------------

        base = data["BaseState"]

        self.basePosition.setText(
            self.formatVector(
                base["position"]
            )
        )

        self.baseOrientation.setText(
            self.formatVector(
                base["orientation"]
            )
        )

        self.baseLinearVelocity.setText(
            self.formatVector(
                base["linear_velocity"]
            )
        )

        self.baseAngularVelocity.setText(
            self.formatVector(
                base["angular_velocity"]
            )
        )

        # ----------------------------------------------------
        # JOINT STATE
        # ----------------------------------------------------

        blind = data["BlindState"]

        positions = blind["jointpositions"]
        velocities = blind["jointvelocities"]

        for i, jointName in enumerate(
            self.jointnames
        ):

            self.jointLabels[jointName]["position"].setText(
                f"{positions[i]:.3f}"
            )

            self.jointLabels[jointName]["velocity"].setText(
                f"{velocities[i]:.3f}"
            )

        # ----------------------------------------------------
        # IMU
        # ----------------------------------------------------

        imu = data["IMU"]

        self.imuAcceleration.setText(
            self.formatVector(
                imu["linear_acceleration"]
            )
        )

        self.imuAngularVelocity.setText(
            self.formatVector(
                imu["angular_velocity"]
            )
        )

        self.imuOrientation.setText(
            self.formatVector(
                imu["orientation"]
            )
        )


# ============================================================
# MAIN
# ============================================================

def main():

    print(
        "aliengo dashboard node initialized!"
    )

    rclpy.init()

    aliengo_dashboard_node = AliengoDashboard()

    app = QApplication(sys.argv)

    dashboard = AliengoUI(
        aliengo_dashboard_node
    )

    dashboard.show()

    # --------------------------------------------------------
    # ROS SPIN TIMER
    # --------------------------------------------------------

    rosspin = QTimer()

    rosspin.timeout.connect(
        lambda: rclpy.spin_once(
            aliengo_dashboard_node,
            timeout_sec=0
        )
    )

    rosspin.start(10)

    try:
        exit_code = app.exec()

    finally:
        aliengo_dashboard_node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()