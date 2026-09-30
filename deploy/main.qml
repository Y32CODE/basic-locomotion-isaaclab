import QtQuick
import QtQuick.Controls
import QtQuick.Window
import QtQuick3D
import QtQuick3D.AssetUtils

Window {
    id: window

    width: 1500
    height: 900
    visible: true

    title: "AlienGo — ROS2 Dashboard"
    color: "#101216"

    // ========================================================
    // ROS2 JOINT STATE
    // ========================================================

    property var jointPositions: [
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    ]

    property var jointVelocities: [
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    ]

    property var jointEfforts: [
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    ]

    Connections {
        target: robotState

        function onJointPositionsChanged(values) {
            window.jointPositions = values
        }

        function onJointVelocitiesChanged(values) {
            window.jointVelocities = values
        }

        function onJointEffortsChanged(values) {
            window.jointEfforts = values
        }
    }

    // ========================================================
    // 3D VIEW
    // ========================================================

    View3D {
        id: view

        anchors {
            left: parent.left
            top: parent.top
            bottom: parent.bottom
            right: telemetryPanel.left
        }

        camera: camera

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#101216"
        }

        // ----------------------------------------------------
        // CAMERA
        // ----------------------------------------------------

        PerspectiveCamera {
            id: camera

            position: Qt.vector3d(
                1.8,
                1.2,
                2.6
            )

            eulerRotation.x: -15
            eulerRotation.y: 35

            clipNear: 0.01
            clipFar: 1000
        }

        // ----------------------------------------------------
        // LIGHTING
        // ----------------------------------------------------

        DirectionalLight {
            eulerRotation.x: -45
            eulerRotation.y: -35

            brightness: 1.5
        }

        DirectionalLight {
            eulerRotation.x: 45
            eulerRotation.y: 135

            brightness: 0.6
        }

        // ----------------------------------------------------
        // ALIENGO GLB
        // ----------------------------------------------------

        RuntimeLoader {
            id: aliengo

            source: "file:///home/yazn/Downloads/ImageToStl.com_aliengo/aliengo.glb"

            scale: Qt.vector3d(
                1,
                1,
                1
            )

            onStatusChanged: {
                if (status === RuntimeLoader.Success) {
                    console.log(
                        "AlienGo GLB loaded successfully"
                    )
                }

                if (status === RuntimeLoader.Error) {
                    console.log(
                        "AlienGo GLB error:",
                        errorString
                    )
                }
            }
        }
    }

    // ========================================================
    // TOP BAR
    // ========================================================

    Rectangle {
        id: topBar

        anchors {
            left: view.left
            right: view.right
            top: parent.top
        }

        height: 65

        color: "#181b22"

        Text {
            anchors {
                left: parent.left
                leftMargin: 25
                verticalCenter: parent.verticalCenter
            }

            text: "ALIENGO"

            color: "white"

            font.pixelSize: 25
            font.bold: true
        }

        Rectangle {
            anchors {
                right: parent.right
                rightMargin: 25
                verticalCenter: parent.verticalCenter
            }

            width: 125
            height: 32

            radius: 16

            color: robotState.connected
                   ? "#18351d"
                   : "#352718"

            Text {
                anchors.centerIn: parent

                text: robotState.connected
                      ? "● CONNECTED"
                      : "● WAITING"

                color: robotState.connected
                       ? "#6ee56e"
                       : "#e6a84f"

                font.pixelSize: 12
                font.bold: true
            }
        }
    }

    // ========================================================
    // TELEMETRY PANEL
    // ========================================================

    Rectangle {
        id: telemetryPanel

        anchors {
            right: parent.right
            top: parent.top
            bottom: parent.bottom
        }

        width: 360

        color: "#15181e"

        // ----------------------------------------------------
        // TITLE
        // ----------------------------------------------------

        Text {
            x: 25
            y: 90

            text: "Robot State"

            color: "white"

            font.pixelSize: 22
            font.bold: true
        }

        Text {
            x: 25
            y: 125

            text: "Live ROS2 joint telemetry"

            color: "#858c98"

            font.pixelSize: 13
        }

        // ----------------------------------------------------
        // JOINT TELEMETRY
        // ----------------------------------------------------

        Column {
            x: 25
            y: 175

            width: parent.width - 50

            spacing: 8

            Repeater {
                model: 12

                delegate: Rectangle {
                    width: parent.width
                    height: 45

                    radius: 6

                    color: index >= 9
                           ? "#1b1f27"
                           : "#191c23"

                    Text {
                        anchors {
                            left: parent.left
                            leftMargin: 10
                            verticalCenter: parent.verticalCenter
                        }

                        text: {
                            var names = [
                                "FL HIP",
                                "FL THIGH",
                                "FL CALF",

                                "FR HIP",
                                "FR THIGH",
                                "FR CALF",

                                "RL HIP",
                                "RL THIGH",
                                "RL CALF",

                                "RR HIP",
                                "RR THIGH",
                                "RR CALF"
                            ]

                            return names[index]
                        }

                        color: "white"

                        font.pixelSize: 12
                        font.bold: true
                    }

                    Text {
                        anchors {
                            right: parent.right
                            rightMargin: 10
                            verticalCenter: parent.verticalCenter
                        }

                        text:
                            window.jointPositions[index].toFixed(3)
                            + " rad"

                        color: "#aeb5c0"

                        font.pixelSize: 12
                    }
                }
            }
        }

        // ----------------------------------------------------
        // RR CALF
        // ----------------------------------------------------

        Rectangle {
            x: 25
            y: 760

            width: parent.width - 50
            height: 90

            radius: 8

            color: "#1d2028"

            Column {
                anchors {
                    left: parent.left
                    leftMargin: 15
                    verticalCenter: parent.verticalCenter
                }

                spacing: 6

                Text {
                    text: "RR CALF"

                    color: "white"

                    font.bold: true
                    font.pixelSize: 13
                }

                Text {
                    text:
                        "Position: "
                        + window.jointPositions[11].toFixed(3)
                        + " rad"

                    color: "#aeb5c0"

                    font.pixelSize: 12
                }

                Text {
                    text:
                        "Effort: "
                        + window.jointEfforts[11].toFixed(2)
                        + " Nm"

                    color: "#aeb5c0"

                    font.pixelSize: 12
                }
            }
        }
    }
}

