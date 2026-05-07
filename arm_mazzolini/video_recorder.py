#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

# This node records a video from a ROS image topic
# using OpenCV's VideoWriter. 
# The output file format is determined by the codec and the output filename extension.

# # custom path / topic
# to use it run:
# ros2 run arm_mazzolini video_recorder.py \
#   --ros-args \
#   -p topic:=/simulated_D435/rgb \
#   -p output:=/tmp/run_001.mkv \
#   -p fps:=30.0
#   -p codec:=X264


# Frames buffered before initialising the VideoWriter so the actual
# wall-clock incoming rate can be measured (fixes short-video issue when
# the simulation RTF is well below 1.0).
_WARMUP = 8


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        self.declare_parameter('topic',  '/simulated_D435/rgb')
        self.declare_parameter('output', 'recording.mkv')
        self.declare_parameter('codec',  'XVID')

        topic:  str = str(self.get_parameter('topic').value)
        output: str = str(self.get_parameter('output').value)
        codec:  str = str(self.get_parameter('codec').value)

        self._bridge = CvBridge()
        self._writer = None
        self._fourcc = cv2.VideoWriter_fourcc(*codec)
        self._output: str = output
        self._warmup: list = []   # list of (frame, wall_time)

        self._sub = self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(f'Recording {topic} -> {os.path.abspath(output)}')

    def _cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        now   = time.monotonic()

        if self._writer is None:
            self._warmup.append((frame, now))
            if len(self._warmup) < _WARMUP:
                return
            # Compute actual wall-clock fps from the warmup window
            elapsed = self._warmup[-1][1] - self._warmup[0][1]
            fps = (_WARMUP - 1) / elapsed if elapsed > 0 else 1.0
            h, w = frame.shape[:2]
            self._writer = cv2.VideoWriter(self._output, self._fourcc, fps, (w, h))
            self.get_logger().info(
                f'Opened writer {w}x{h} @ {fps:.2f} fps (measured over {_WARMUP} frames)')
            for f, _ in self._warmup:
                self._writer.write(f)
            self._warmup = []
            return

        self._writer.write(frame)

    def destroy_node(self):
        if self._writer:
            self._writer.release()
            self.get_logger().info(f'Saved {self._output}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
