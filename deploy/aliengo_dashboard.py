import os
import sys
import threading
from pathlib import Path


# ============================================================
# PATHS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
CONDA_PREFIX = Path(sys.prefix)

PYTHON_VERSION = f"python{sys.version_info.major}.{sys.version_info.minor}"


# ============================================================
# PySide6 / Qt paths
# ============================================================

PYSIDE6_QT = (
    CONDA_PREFIX
    / "lib"
    / PYTHON_VERSION
    / "site-packages"
    / "PySide6"
    / "Qt"
)

PYSIDE6_QML = PYSIDE6_QT / "qml"
PYSIDE6_PLUGINS = PYSIDE6_QT / "plugins"
PYSIDE6_LIB = PYSIDE6_QT / "lib"


# IMPORTANT:
# Set these BEFORE importing PySide6.

os.environ["QT_PLUGIN_PATH"] = str(PYSIDE6_PLUGINS)
os.environ["QML2_IMPORT_PATH"] = str(PYSIDE6_QML)
os.environ["QML_IMPORT_PATH"] = str(PYSIDE6_QML)


# ============================================================
# ROS 2
# ============================================================

os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

ROS_WS = BASE_DIR / "ros2_ws"


# ============================================================
# DLS2 paths
# ============================================================

DLS2_PYTHON = (
    ROS_WS
    / "install"
    / "dls2_interface"
    / "lib"
    / PYTHON_VERSION
    / "site-packages"
)

DLS2_LIB = (
    ROS_WS
    / "install"
    / "dls2_interface"
    / "lib"
)


# ============================================================
# Python path
# ============================================================

if DLS2_PYTHON.exists():
    sys.path.insert(0, str(DLS2_PYTHON))

    os.environ["PYTHONPATH"] = (
        str(DLS2_PYTHON)
        + os.pathsep
        + os.environ.get("PYTHONPATH", "")
    )


# ============================================================
# Shared library path
# ============================================================

ld_paths = []

if DLS2_LIB.exists():
    ld_paths.append(str(DLS2_LIB))

if PYSIDE6_LIB.exists():
    ld_paths.append(str(PYSIDE6_LIB))

existing_ld = os.environ.get("LD_LIBRARY_PATH", "")

if existing_ld:
    ld_paths.append(existing_ld)

os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(ld_paths)


# ============================================================
# Debug information
# ============================================================

print("Python:", sys.executable)
print("Python version:", sys.version.split()[0])
print("PySide6 Qt:", PYSIDE6_QT)
print("QML path:", PYSIDE6_QML)
print("Qt plugin path:", PYSIDE6_PLUGINS)
print("Qt library path:", PYSIDE6_LIB)
print("DLS2 Python:", DLS2_PYTHON)
print("DLS2 lib:", DLS2_LIB)


# ============================================================
# ROS 2 imports
# ============================================================

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState


# ============================================================
# Qt imports
# ============================================================

from PySide6.QtCore import QObject, Signal

from PySide6.QtGui import QGuiApplication

from PySide6.QtQml import QQmlApplicationEngine


# ============================================================
# Joint mapping
# ============================================================

JOINT_NAMES = [
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


# ============================================================
# Robot state
# ============================================================

class RobotState(QObject):

    jointPositionsChanged = Signal("QVariantList")
    jointVelocitiesChanged = Signal("QVariantList")
    jointEffortsChanged = Signal("QVariantList")

    connectedChanged = Signal(bool)

    def __init__(self):

        super().__init__()

        self._positions = [0.0] * 12
        self._velocities = [0.0] * 12
        self._efforts = [0.0] * 12

        self._connected = False

    @property
    def connected(self):

        return self._connected

    def update_joint_state(self, msg):

        positions = [0.0] * 12
        velocities = [0.0] * 12
        efforts = [0.0] * 12

        name_to_index = {
            name: i
            for i, name in enumerate(JOINT_NAMES)
        }

        for i, name in enumerate(msg.name):

            if name not in name_to_index:
                continue

            joint_index = name_to_index[name]

            if i < len(msg.position):
                positions[joint_index] = msg.position[i]

            if i < len(msg.velocity):
                velocities[joint_index] = msg.velocity[i]

            if i < len(msg.effort):
                efforts[joint_index] = msg.effort[i]

        self._positions = positions
        self._velocities = velocities
        self._efforts = efforts

        self.jointPositionsChanged.emit(positions)
        self.jointVelocitiesChanged.emit(velocities)
        self.jointEffortsChanged.emit(efforts)

        if not self._connected:

            self._connected = True

            self.connectedChanged.emit(True)


# ============================================================
# ROS 2 node
# ============================================================

class AlienGoROSNode(Node):

    def __init__(self, state):

        super().__init__("aliengo_qt_dashboard")

        self.state = state

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states_legged",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info(
            "Subscribed to /joint_states_legged"
        )

    def joint_state_callback(self, msg):

        self.state.update_joint_state(msg)


# ============================================================
# ROS thread
# ============================================================

def ros_thread(state):

    rclpy.init()

    node = AlienGoROSNode(state)

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# Main
# ============================================================

def main():

    app = QGuiApplication(sys.argv)

    robot_state = RobotState()

    thread = threading.Thread(
        target=ros_thread,
        args=(robot_state,),
        daemon=True,
    )

    thread.start()

    engine = QQmlApplicationEngine()

    engine.addImportPath(
        str(PYSIDE6_QML)
    )

    print(
        "QML import path:",
        PYSIDE6_QML
    )

    engine.rootContext().setContextProperty(
        "robotState",
        robot_state,
    )

    qml_path = BASE_DIR / "main.qml"

    if not qml_path.exists():

        print(
            "ERROR: main.qml not found:",
            qml_path
        )

        sys.exit(1)

    engine.load(
        str(qml_path)
    )

    if not engine.rootObjects():

        print(
            "ERROR: QML failed to load."
        )

        sys.exit(1)

    sys.exit(
        app.exec()
    )


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":

    main()

