#!/home/pritam/vla_venv/bin/python3
"""
VLA Grounding Node: Local Open-Vocabulary 3D Target Grounding for Autonomous Mobile Robots
Ingests RGB camera frames and natural language directives (e.g. "yellow shelf", "cardboard box").
Runs zero-cost feed-forward YOLO-World on CUDA.
Deprojects 2D visual bboxes to 3D metric map poses and publishes docking approach goals for Nav2.
"""

import sys
import math
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

try:
    from ultralytics import YOLOWorld
except ImportError:
    YOLOWorld = None


class VlaGroundingNode(Node):
    def __init__(self):
        super().__init__('vla_grounding_node')

        # Declare parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('robot_name', 'bcr_bot_1')
        self.declare_parameter('model_path', '/home/pritam/bcr_ws/src/bcr_bot/models/yolov8s-worldv2.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence_threshold', 0.12)
        self.declare_parameter('standoff_distance', 0.90)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('default_prompt', 'yellow shelf')

        self.robot_name = self.get_parameter('robot_name').value
        self.model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        self.conf_thresh = float(self.get_parameter('confidence_threshold').value)
        self.standoff_dist = float(self.get_parameter('standoff_distance').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.current_prompt = self.get_parameter('default_prompt').value

        self.optical_frame = f"{self.robot_name}/kinect_camera_optical"
        self.base_frame = f"{self.robot_name}/base_link"

        self.get_logger().info(f"Initializing VLA Grounding Node for robot: {self.robot_name}")
        self.get_logger().info(f"Loading YOLO-World model from: {self.model_path} on {self.device}")

        # Load YOLO-World model
        self.model = YOLOWorld(self.model_path).to(self.device)
        self.set_classes_for_prompt(self.current_prompt)

        # Bridge and state
        self.bridge = CvBridge()
        self.latest_camera_info = None
        self.latest_depth_image = None
        self.last_detection_time = self.get_clock().now()

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Reliable depth 10 QoS ensures instant matching with ros_gz_bridge
        sensor_qos = 10

        # Subscribers
        img_topic = f"/{self.robot_name}/kinect_camera/image"
        info_topic = f"/{self.robot_name}/kinect_camera/camera_info"
        depth_topic = f"/{self.robot_name}/kinect_camera"
        prompt_topic = "/fleet_commander/natural_prompt"

        self.img_sub = self.create_subscription(Image, img_topic, self.image_callback, sensor_qos)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, sensor_qos)
        self.prompt_sub = self.create_subscription(String, prompt_topic, self.prompt_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, f"/{self.robot_name}/vla_target_pose", 10)
        self.annotated_img_pub = self.create_publisher(Image, f"/{self.robot_name}/vla_detection_image", 1)
        self.marker_pub = self.create_publisher(Marker, f"/{self.robot_name}/vla_target_marker", 10)
        self.status_pub = self.create_publisher(String, f"/{self.robot_name}/vla_status", 10)

        # Throttle processing rate to ~10 Hz to maintain abundant GPU headroom
        self.process_interval_sec = 0.10
        self.last_process_time = 0.0

        self.get_logger().info(f"VLA Grounding Node ready! Active prompt: '{self.current_prompt}'")

    def set_classes_for_prompt(self, prompt: str):
        prompt_clean = prompt.strip().lower()
        if not prompt_clean:
            return
        self.current_prompt = prompt_clean
        # Add prompt and contextual synonyms for optimal open-vocabulary grounding
        classes = [self.current_prompt, "shelf", "box", "pallet", "barrel"]
        self.get_logger().info(f"Updating open-vocabulary classes to: {classes}")
        self.model.set_classes(classes)

    def prompt_callback(self, msg: String):
        new_prompt = msg.data.strip()
        if new_prompt and new_prompt != self.current_prompt:
            self.get_logger().info(f"Received new natural language prompt: '{new_prompt}'")
            self.set_classes_for_prompt(new_prompt)

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def depth_callback(self, msg: Image):
        try:
            if msg.encoding in ['32FC1', '32F']:
                depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding in ['16UC1', '16U']:
                depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1').astype(np.float32) / 1000.0
            else:
                depth_cv = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            self.latest_depth_image = depth_cv
        except Exception as e:
            self.get_logger().debug(f"Depth conversion warning: {e}")

    def image_callback(self, msg: Image):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (now_sec - self.last_process_time) < self.process_interval_sec:
            return
        self.last_process_time = now_sec

        self.get_logger().info(
            f"VLA Perception: Ingested frame ({msg.width}x{msg.height}), evaluating '{self.current_prompt}'",
            throttle_duration_sec=3.0
        )

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            try:
                # Direct buffer fallback
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            except Exception as e:
                self.get_logger().error(f"Image decode failed: {e}")
                return

        # Run YOLO-World inference on CUDA
        results = self.model(cv_img, conf=self.conf_thresh, verbose=False)[0]

        best_box = None
        best_conf = 0.0
        best_cls_name = ""

        if results.boxes is not None and len(results.boxes) > 0:
            for box in results.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                cls_name = results.names[cls_id]

                # Match primary prompt or highest confidence valid object
                if (cls_name == self.current_prompt or cls_id == 0) and conf > best_conf:
                    best_conf = conf
                    best_box = box.xyxy[0].cpu().numpy()
                    best_cls_name = cls_name
                elif best_box is None and conf > best_conf:
                    best_conf = conf
                    best_box = box.xyxy[0].cpu().numpy()
                    best_cls_name = cls_name

        annotated_img = cv_img.copy()

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            # Estimate depth
            depth_m = self.estimate_depth_at(cx, cy, cv_img.shape)

            # Draw visual bounding box
            color = (0, 255, 0) if best_cls_name == self.current_prompt else (0, 200, 255)
            cv2.rectangle(annotated_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.circle(annotated_img, (int(cx), int(cy)), 4, (0, 0, 255), -1)

            label_str = f"{best_cls_name}: {best_conf:.2f} | Z={depth_m:.2f}m"
            cv2.putText(annotated_img, label_str, (int(x1), max(int(y1) - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Deproject to 3D and transform to map
            map_target_pt = self.deproject_and_transform_to_map(cx, cy, depth_m)
            if map_target_pt is not None:
                # Compute collision-free standoff navigation goal
                goal_pose = self.compute_standoff_goal(map_target_pt)
                if goal_pose is not None:
                    self.pose_pub.publish(goal_pose)
                    self.publish_target_marker(map_target_pt, goal_pose, best_cls_name)

                    status_dict = {
                        "status": "TARGET_GROUNDED",
                        "prompt": self.current_prompt,
                        "detected_class": best_cls_name,
                        "confidence": round(best_conf, 3),
                        "target_x": round(map_target_pt[0], 3),
                        "target_y": round(map_target_pt[1], 3),
                        "goal_x": round(goal_pose.pose.position.x, 3),
                        "goal_y": round(goal_pose.pose.position.y, 3),
                        "standoff_dist": self.standoff_dist
                    }
                    status_msg = String()
                    status_msg.data = json.dumps(status_dict)
                    self.status_pub.publish(status_msg)
                    self.get_logger().info(
                        f"Grounded '{best_cls_name}' ({best_conf:.2f}) -> Map=({map_target_pt[0]:.2f}, {map_target_pt[1]:.2f}) Dock=({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
                    )
        else:
            # No target in current view
            cv2.putText(annotated_img, f"Searching: '{self.current_prompt}'", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            if (now_sec - getattr(self, '_last_search_pub', 0.0)) > 2.0:
                self._last_search_pub = now_sec
                search_dict = {
                    "status": "SEARCHING",
                    "prompt": self.current_prompt
                }
                search_msg = String()
                search_msg.data = json.dumps(search_dict)
                self.status_pub.publish(search_msg)

        # Publish visual telemetry image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            out_msg.header = msg.header
            self.annotated_img_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")

    def estimate_depth_at(self, cx: float, cy: float, shape) -> float:
        """Sample depth from depth image if available, else use focal length geometry."""
        h, w = shape[:2]
        ix, iy = int(round(cx)), int(round(cy))

        if self.latest_depth_image is not None:
            dh, dw = self.latest_depth_image.shape[:2]
            # Scale coordinates if depth resolution differs
            dx = int(round(ix * dw / w))
            dy = int(round(iy * dh / h))

            x0 = max(0, dx - 3)
            x1 = min(dw, dx + 4)
            y0 = max(0, dy - 3)
            y1 = min(dh, dy + 4)

            patch = self.latest_depth_image[y0:y1, x0:x1]
            valid = patch[np.isfinite(patch) & (patch > 0.15) & (patch < 12.0)]
            if len(valid) > 0:
                return float(np.median(valid))

        # Fallback metric estimation: typical warehouse aisle line of sight is 1.8 to 2.5m
        return 2.20

    def deproject_and_transform_to_map(self, cx: float, cy: float, depth: float):
        """Deproject pixel (cx, cy) to 3D point in camera optical frame, then transform to map."""
        if self.latest_camera_info is not None and self.latest_camera_info.k[0] > 0.0:
            fx = self.latest_camera_info.k[0]
            cx_cam = self.latest_camera_info.k[2]
            fy = self.latest_camera_info.k[4]
            cy_cam = self.latest_camera_info.k[5]
        else:
            # Standard 640x480 Kinect intrinsics (approx 60 deg horizontal FOV)
            fx = 554.25
            fy = 554.25
            cx_cam = 320.0
            cy_cam = 240.0

        # Optical frame: X right, Y down, Z forward
        z_c = depth
        x_c = (cx - cx_cam) * z_c / fx
        y_c = (cy - cy_cam) * z_c / fy

        pt_opt = PointStamped()
        pt_opt.header.frame_id = self.optical_frame
        pt_opt.header.stamp = rclpy.time.Time().to_msg()
        pt_opt.point.x = float(x_c)
        pt_opt.point.y = float(y_c)
        pt_opt.point.z = float(z_c)

        try:
            # Transform to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.optical_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08)
            )
            pt_map = do_transform_point(pt_opt, transform)
            return (pt_map.point.x, pt_map.point.y, pt_map.point.z)
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed from {self.optical_frame} to {self.map_frame}: {e}")
            return None

    def compute_standoff_goal(self, target_pt):
        """Calculate approach waypoint standoff meters away from target facing the object."""
        tx, ty, _ = target_pt

        # Lookup current robot base pose in map frame
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08)
            )
            rx = tf_robot.transform.translation.x
            ry = tf_robot.transform.translation.y
        except Exception:
            return None

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        if dist < 0.1:
            return None

        # Desired standoff distance
        actual_standoff = min(self.standoff_dist, dist * 0.7)
        nx = dx / dist
        ny = dy / dist

        gx = tx - nx * actual_standoff
        gy = ty - ny * actual_standoff

        # Facing angle towards target object
        yaw = math.atan2(dy, dx)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = float(qz)
        goal.pose.orientation.w = float(qw)

        return goal

    def publish_target_marker(self, target_pt, goal_pose, cls_name: str):
        tx, ty, tz = target_pt

        # Landmark sphere marker
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_grounding"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(tx)
        marker.pose.position.y = float(ty)
        marker.pose.position.z = 0.4
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.80
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.8
        marker.color.a = 0.85
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VlaGroundingNode()
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
