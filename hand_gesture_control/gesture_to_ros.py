#!/usr/bin/env python3

import cv2
import mediapipe as mp
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    'base_joint',
    'shoulder_joint',
    'elbow_joint',
    'wrist_joint1',
    'wrist_joint2',
    'wrist_joint3',
    'right_finger',
    'left_finger'
]

JOINT_LIMITS = [
    (-3.14, 3.14),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-3.14, 3.14),
    (-1.57, 1.57),
    (-3.14, 3.14),
    (0.0, 0.7),
    (0.0, 0.7)
]

ARM_SPEED = 0.5
FINGER_SPEED = 0.3
UPDATE_RATE = 20.0

MOVE_THRESHOLD = 0.04
DEPTH_THRESHOLD = 0.15
TILT_THRESHOLD = 0.15


class GestureRobotController(Node):

    def __init__(self):
        super().__init__('gesture_robot_controller')

        self.joint_positions = {}
        self.targets = None
        self.last_update = time.monotonic()

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / UPDATE_RATE,
            self.update_robot
        )

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

        if self.targets is None and all(
            name in self.joint_positions for name in JOINT_NAMES
        ):
            self.targets = [
                self.joint_positions[name] for name in JOINT_NAMES
            ]

    def command_joint(self, index, direction):
        if self.targets is None:
            return

        lower, upper = JOINT_LIMITS[index]
        speed = FINGER_SPEED if index >= 6 else ARM_SPEED

        dt = min(time.monotonic() - self.last_update, 0.05)
        self.targets[index] = max(
            lower,
            min(upper, self.targets[index] + direction * speed * dt)
        )

    def set_gripper(self, opened):
        if self.targets is None:
            return

        # Assumes 0.0 = closed and 0.7 = open.
        value = 0.7 if opened else 0.0
        self.targets[6] = value
        self.targets[7] = value

    def update_robot(self):
        if self.targets is None:
            return

        self.last_update = time.monotonic()

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = list(self.targets)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 150000000

        trajectory.points.append(point)
        self.publisher.publish(trajectory)

    def stop_motion(self):
    # Hold all joints at their current positions
        if self.targets is None:
            return

        for i, name in enumerate(JOINT_NAMES):
            if name in self.joint_positions:
                self.targets[i] = self.joint_positions[name]

def main():
    rclpy.init()
    node = GestureRobotController()

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("ERROR: Webcam could not be opened.")
        node.destroy_node()
        rclpy.shutdown()
        return

    ref_x = None
    ref_y = None
    ref_size = None
    previous_gesture = None
    last_gripper_command = 0.0

    try:
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.7
        ) as hands:

            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.001)

                success, frame = cap.read()
                if not success:
                    print("Webcam frame unavailable.")
                    break

                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = hands.process(rgb)

                movement = "NONE"
                gesture = "NO HAND"

                if results.multi_hand_landmarks:
                    hand = results.multi_hand_landmarks[0]
                    
                    landmarks = hand.landmark

                    mp_draw.draw_landmarks(
                        frame, hand, mp_hands.HAND_CONNECTIONS
                    )

                    wrist = landmarks[mp_hands.HandLandmark.WRIST]
                    index_mcp = landmarks[
                        mp_hands.HandLandmark.INDEX_FINGER_MCP
                    ]
                    pinky_mcp = landmarks[
                        mp_hands.HandLandmark.PINKY_MCP
                    ]
                    middle_mcp = landmarks[
                        mp_hands.HandLandmark.MIDDLE_FINGER_MCP
                    ]

                    x, y = wrist.x, wrist.y

                    palm_width = math.hypot(
                        index_mcp.x - pinky_mcp.x,
                        index_mcp.y - pinky_mcp.y
                    )
                    palm_length = math.hypot(
                        middle_mcp.x - wrist.x,
                        middle_mcp.y - wrist.y
                    )
                    hand_size = (palm_width + palm_length) / 2

                    tips = [
                        mp_hands.HandLandmark.INDEX_FINGER_TIP,
                        mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                        mp_hands.HandLandmark.RING_FINGER_TIP,
                        mp_hands.HandLandmark.PINKY_TIP
                    ]
                    joints = [
                        mp_hands.HandLandmark.INDEX_FINGER_PIP,
                        mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
                        mp_hands.HandLandmark.RING_FINGER_PIP,
                        mp_hands.HandLandmark.PINKY_PIP
                    ]

                    extended = sum(
                        landmarks[t].y < landmarks[j].y
                        for t, j in zip(tips, joints)
                    )

                    if extended >= 3:
                        gesture = "OPEN PALM"
                    elif extended <= 1:
                        gesture = "FIST"
                    else:
                        gesture = "HAND"

                    # Establish neutral position
                    if ref_x is None:
                        ref_x, ref_y, ref_size = x, y, hand_size
                        

                    dx = x - ref_x
                    dy = y - ref_y
                    ratio = hand_size / ref_size if ref_size > 0 else 1.0

                    # Only one arm joint is controlled at a time.
                    # Releasing the reference dead zone stops new motion.
                        
                    if ratio > 1.0 + DEPTH_THRESHOLD:
                        movement = "TOWARD CAMERA"
                        node.command_joint(2, -1)

                    elif ratio < 1.0 - DEPTH_THRESHOLD:
                        movement = "AWAY FROM CAMERA"
                        node.command_joint(2, 1)

                    elif abs(dx) > MOVE_THRESHOLD and abs(dx) > abs(dy):
                        if dx > 0:
                            movement = "RIGHT"
                            node.command_joint(0, 1)
                        else:
                            movement = "LEFT"
                            node.command_joint(0, -1)

                    elif abs(dy) > MOVE_THRESHOLD:
                        if dy > 0:
                            movement = "DOWN"
                            node.command_joint(1, -1)
                        else:
                            movement = "UP"
                            node.command_joint(1, 1)

                    # Update gripper only when gesture changes,
                    # preventing continuous repeated gripper commands.
                    now = time.monotonic()
                    if gesture in ("OPEN PALM", "FIST"):
                        if gesture != previous_gesture and now - last_gripper_command > 0.4:
                            node.set_gripper(gesture == "OPEN PALM")
                            last_gripper_command = now

                    previous_gesture = gesture

                    cv2.putText(
                        frame, "Gesture: " + gesture, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2
                    )
                    cv2.putText(
                        frame, "Movement: " + movement, (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2
                    )
                    cv2.putText(
                        frame, "R: STOP AND RESET | Q: QUIT",
                        (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
                    )

                else:
                    # Require the hand to be reacquired and calibrated.
                    ref_x = ref_y = ref_size = None
                    previous_gesture = None

                cv2.imshow("Gesture Robot Control - Press Q to quit", frame)

                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break

                elif key == ord('r'):
                    node.stop_motion()

                    if results.multi_hand_landmarks:
                        hand = results.multi_hand_landmarks[0]
                        wrist = hand.landmark[mp_hands.HandLandmark.WRIST]
                        index_mcp = hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                        pinky_mcp = hand.landmark[mp_hands.HandLandmark.PINKY_MCP]
                        middle_mcp = hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

                        ref_x = wrist.x
                        ref_y = wrist.y

                        palm_width = math.hypot(
                            index_mcp.x - pinky_mcp.x,
                            index_mcp.y - pinky_mcp.y
                        )
                        palm_length = math.hypot(
                            middle_mcp.x - wrist.x,
                            middle_mcp.y - wrist.y
                        )
                        ref_size = (palm_width + palm_length) / 2

                    previous_gesture = None
                    print("Motion stopped. Reference reset.")

    except KeyboardInterrupt:
        pass

    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
