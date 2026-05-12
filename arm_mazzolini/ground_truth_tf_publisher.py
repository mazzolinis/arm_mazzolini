#!/usr/bin/env python3
"""
Publishes world -> odom as a dynamic TF using Gazebo ground truth.

The Gazebo pose bridge gives the robot's true pose (agricultural_world -> weeder_robot,
which equals world -> base_link since both static offsets are identity).
The diff drive controller gives odom -> base_link.
This node computes: world_T_odom = world_T_base * inv(odom_T_base)
so the full chain world -> odom -> base_link -> ... -> laser is ground-truth accurate.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf2_ros


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[0, 0] = 1 - 2*(y*y + z*z)
    T[0, 1] = 2*(x*y - w*z)
    T[0, 2] = 2*(x*z + w*y)
    T[1, 0] = 2*(x*y + w*z)
    T[1, 1] = 1 - 2*(x*x + z*z)
    T[1, 2] = 2*(y*z - w*x)
    T[2, 0] = 2*(x*z - w*y)
    T[2, 1] = 2*(y*z + w*x)
    T[2, 2] = 1 - 2*(x*x + y*y)
    return T


def _make_T(translation, rotation) -> np.ndarray:
    T = _quat_to_matrix(rotation)
    T[0, 3] = translation.x
    T[1, 3] = translation.y
    T[2, 3] = translation.z
    return T


def _matrix_to_quat(R: np.ndarray):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class GroundTruthTFPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_publisher')

        self.declare_parameter('gz_model_name', 'weeder_robot')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')

        model_name = self.get_parameter('gz_model_name').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self._model_name = model_name
        self._broadcaster = tf2_ros.TransformBroadcaster(self)

        self._world_T_base: np.ndarray | None = None
        self._odom_T_base: np.ndarray | None = None
        self._last_stamp = None

        self.create_subscription(TFMessage, '/model/weeder_robot/pose',
                                 self._gt_callback, 10)
        self.create_subscription(Odometry, odom_topic,
                                 self._odom_callback, 10)

        self.get_logger().info(
            f'Ground truth TF publisher started. '
            f'Waiting for /model/weeder_robot/pose and {odom_topic}...'
        )

    def _gt_callback(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.child_frame_id == self._model_name:
                self._world_T_base = _make_T(tf.transform.translation,
                                             tf.transform.rotation)
                self._last_stamp = tf.header.stamp
                self._publish()
                break

    def _odom_callback(self, msg: Odometry):
        self._odom_T_base = _make_T(msg.pose.pose.position,
                                    msg.pose.pose.orientation)
        if self._last_stamp is None:
            self._last_stamp = msg.header.stamp
        self._publish()

    def _publish(self):
        if self._world_T_base is None or self._odom_T_base is None:
            return

        world_T_odom = self._world_T_base @ np.linalg.inv(self._odom_T_base)

        t = TransformStamped()
        t.header.stamp = self._last_stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = world_T_odom[0, 3]
        t.transform.translation.y = world_T_odom[1, 3]
        t.transform.translation.z = world_T_odom[2, 3]
        qx, qy, qz, qw = _matrix_to_quat(world_T_odom[:3, :3])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
