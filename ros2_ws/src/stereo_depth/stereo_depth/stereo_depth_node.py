#!/usr/bin/env python3

from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from stereo_depth.algorithms.sgbm import SGBMDepthEstimator


class StereoDepthNode(Node):
    """
    Compute a metric depth map from rectified stereo grayscale images.

    The node uses generic internal topic names and relies on launch-file
    remapping to connect to dataset-specific ZED topics.
    """

    def __init__(self):
        super().__init__("stereo_depth_node")

        # Internal topic names. Remapped in saver.launch.py.
        self.left_image_topic = "left/image"
        self.right_image_topic = "right/image"
        self.left_info_topic = "left/camera_info"
        self.right_info_topic = "right/camera_info"
        self.depth_topic = "depth/image"
        self.depth_info_topic = "depth/camera_info"

        # Algorithm / tuning parameters.
        self.declare_parameter("algorithm", "sgbm")
        self.declare_parameter("scale", 0.5)

        self.algorithm = str(self.get_parameter("algorithm").value).lower()
        self.scale = float(self.get_parameter("scale").value)

        if self.algorithm == "sgbm":
            self.depth_estimator = SGBMDepthEstimator()
        elif self.algorithm == "foundation_stereo":
            self.depth_estimator = FoundationStereoDepthEstimator()
        else:
            raise ValueError(f"Unsupported algorithm: {self.algorithm}")

        self.bridge = CvBridge()
        self.left_info: Optional[CameraInfo] = None
        self.right_info: Optional[CameraInfo] = None

        self.left_info_sub = self.create_subscription(
            CameraInfo, self.left_info_topic, self.left_info_callback, 10
        )
        self.right_info_sub = self.create_subscription(
            CameraInfo, self.right_info_topic, self.right_info_callback, 10
        )

        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, self.depth_info_topic, 10)

        # Synchronize left/right image frames approximately.
        self.left_sub = message_filters.Subscriber(self, Image, self.left_image_topic)
        self.right_sub = message_filters.Subscriber(self, Image, self.right_image_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.05,
        )
        self.sync.registerCallback(self.stereo_callback)

        self.frame_count = 0

        self.get_logger().info(
            f"StereoDepthNode started: {self.left_image_topic} + "
            f"{self.right_image_topic} -> {self.depth_topic}"
        )

    def left_info_callback(self, msg: CameraInfo):
        self.left_info = msg

    def right_info_callback(self, msg: CameraInfo):
        self.right_info = msg

    def get_baseline_from_camera_info(self) -> Optional[float]:
        """
        For rectified stereo, right CameraInfo.P[3] is -fx * baseline.
        baseline = abs(P[3] / P[0])
        """
        if self.right_info is None:
            return None

        fx_right = float(self.right_info.p[0])
        tx = float(self.right_info.p[3])

        if abs(fx_right) < 1e-6:
            return None

        baseline = abs(tx / fx_right)
        return baseline

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        if self.left_info is None or self.right_info is None:
            self.get_logger().warn("Waiting for camera_info messages...", throttle_duration_sec=5.0)
            return

        baseline = self.get_baseline_from_camera_info()
        if baseline is None or baseline <= 0.0:
            self.get_logger().error("Invalid stereo baseline from right CameraInfo.P")
            return

        fx = float(self.left_info.k[0])
        if fx <= 0:
            self.get_logger().error("Invalid focal length from left CameraInfo.K")
            return

        try:
            left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="mono8")
            right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        original_h, original_w = left.shape[:2]

        # Downscale for speed. Use scaled focal length consistently.
        if self.scale != 1.0:
            proc_w = int(original_w * self.scale)
            proc_h = int(original_h * self.scale)
            left_proc = cv2.resize(left, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            right_proc = cv2.resize(right, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            fx_proc = fx * self.scale
        else:
            left_proc = left
            right_proc = right
            fx_proc = fx

        depth = self.depth_estimator.compute(left_proc, right_proc, fx_proc, baseline)

        # Resize depth back to original image size if needed.
        if self.scale != 1.0:
            depth = cv2.resize(depth, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

        depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding="32FC1")
        depth_msg.header.stamp = left_msg.header.stamp
        depth_msg.header.frame_id = left_msg.header.frame_id

        info_msg = CameraInfo()
        info_msg.header.stamp = left_msg.header.stamp
        info_msg.header.frame_id = left_msg.header.frame_id
        info_msg.height = self.left_info.height
        info_msg.width = self.left_info.width
        info_msg.distortion_model = self.left_info.distortion_model
        info_msg.d = list(self.left_info.d)
        info_msg.k = list(self.left_info.k)
        info_msg.r = list(self.left_info.r)
        info_msg.p = list(self.left_info.p)
        info_msg.binning_x = self.left_info.binning_x
        info_msg.binning_y = self.left_info.binning_y
        info_msg.roi = self.left_info.roi

        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(info_msg)

        self.frame_count += 1
        if self.frame_count % 50 == 0:
            finite = np.isfinite(depth)
            valid_ratio = float(np.count_nonzero(finite)) / float(depth.size)
            mean_depth = float(np.nanmean(depth)) if np.any(finite) else float("nan")
            self.get_logger().info(
                f"Published frame {self.frame_count}: "
                f"baseline={baseline:.4f} m, fx={fx:.2f}, "
                f"valid={valid_ratio:.2%}, mean_depth={mean_depth:.2f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()