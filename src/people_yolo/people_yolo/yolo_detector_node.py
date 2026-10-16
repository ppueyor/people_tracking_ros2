#!/usr/bin/env python3
"""
RGB-D people detector using a YOLOv4-tiny CNN via OpenCV's DNN module.

Drop-in replacement for people_rgbd's HOG detector: it detects people in the
RGB image, reads the depth image to get each person's 3D position, transforms
that into the configured frame, and publishes people_msgs/People tagged "RGBD"
on /people_detections (plus visualization markers). Because it uses the same
topic and tag, the arco_people_tracker fuser treats it identically - just run
this node instead of people_rgbd_node.
"""
import os
import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from people_msgs.msg import People, Person

import tf2_ros
from tf2_geometry_msgs import do_transform_point

# COCO class index for "person"
PERSON_CLASS_ID = 0


class YoloPeopleDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Parameters
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.rate = self.declare_parameter('rate', 5.0).value
        self.conf_threshold = self.declare_parameter('confidence_threshold', 0.4).value
        self.nms_threshold = self.declare_parameter('nms_threshold', 0.4).value
        self.depth_min = self.declare_parameter('depth_min', 0.3).value
        self.depth_max = self.declare_parameter('depth_max', 8.0).value
        self.input_size = int(self.declare_parameter('input_size', 416).value)
        # If set, the back-projected point is stamped with this frame instead of
        # the RGB image's header frame_id. Use it when the camera driver labels
        # images with a non-optical frame (e.g. base_footprint) but a proper
        # optical frame exists in the TF tree.
        self.camera_optical_frame = self.declare_parameter('camera_optical_frame', '').value
        model_config = self.declare_parameter('model_config', '').value
        model_weights = self.declare_parameter('model_weights', '').value

        if not model_config or not model_weights:
            self.get_logger().fatal('model_config and model_weights parameters are required')
            raise SystemExit(2)
        if not os.path.isfile(model_config) or not os.path.isfile(model_weights):
            self.get_logger().fatal(f'model files not found: {model_config} / {model_weights}')
            raise SystemExit(2)

        # Load the YOLO network (CPU backend)
        self.net = cv2.dnn.readNetFromDarknet(model_config, model_weights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        ln = self.net.getLayerNames()
        self.out_layers = [ln[i - 1] for i in self.net.getUnconnectedOutLayers().flatten()]
        self.get_logger().info(f'Loaded YOLO model, output layers: {self.out_layers}')

        # Camera intrinsics (filled by camera_info callback)
        self.have_calib = False
        self.fx = self.fy = self.cx = self.cy = 0.0

        self.bridge = CvBridge()
        self.last_process_time = self.get_clock().now()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.people_pub = self.create_publisher(People, '/people_detections', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/people_detections/markers', 1)

        # Subscriptions: RGB + depth time-synchronized, camera_info separate
        self.info_sub = self.create_subscription(CameraInfo, 'camera_info', self.info_cb, 1)
        rgb_sub = message_filters.Subscriber(self, Image, 'image')
        depth_sub = message_filters.Subscriber(self, Image, 'depth')
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        self.sync.registerCallback(self.image_cb)

        self.get_logger().info('YOLO people detector ready.')

    def info_cb(self, msg):
        if not self.have_calib:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.have_calib = True
            self.get_logger().info(
                f'Got camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def image_cb(self, rgb_msg, depth_msg):
        if not self.have_calib:
            self.get_logger().warn('No camera calibration yet, skipping frame', throttle_duration_sec=5.0)
            return

        # Throttle to the configured rate
        now = self.get_clock().now()
        if (now - self.last_process_time).nanoseconds < 1e9 / max(self.rate, 0.1):
            return
        self.last_process_time = now

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Depth to meters (float32 assumed meters; uint16 assumed millimeters)
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        boxes = self.detect_people(rgb)
        positions = self.boxes_to_positions(boxes, depth_m, depth_msg.header)

        self.publish_people(positions)
        self.publish_markers(positions)

    def detect_people(self, bgr):
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1 / 255.0, (self.input_size, self.input_size),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.out_layers)

        boxes, confidences = [], []
        for out in outs:
            for det in out:
                scores = det[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if class_id == PERSON_CLASS_ID and confidence > self.conf_threshold:
                    cx_n, cy_n, bw_n, bh_n = det[0], det[1], det[2], det[3]
                    bw = int(bw_n * w)
                    bh = int(bh_n * h)
                    x = int(cx_n * w - bw / 2)
                    y = int(cy_n * h - bh / 2)
                    boxes.append([x, y, bw, bh])
                    confidences.append(confidence)

        # Non-maximum suppression
        keep = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        result = []
        if len(keep) > 0:
            for i in keep.flatten():
                x, y, bw, bh = boxes[i]
                # Clamp to image bounds
                x0, y0 = max(0, x), max(0, y)
                x1, y1 = min(w, x + bw), min(h, y + bh)
                if x1 > x0 and y1 > y0:
                    result.append((x0, y0, x1, y1))
        return result

    def boxes_to_positions(self, boxes, depth_m, depth_header):
        positions = []
        src_frame = self.camera_optical_frame if self.camera_optical_frame else depth_header.frame_id
        for (x0, y0, x1, y1) in boxes:
            # Robust depth for the box: median of valid depths in the central region
            cx_px = (x0 + x1) // 2
            cy_px = (y0 + y1) // 2
            # sample a central patch to avoid background pixels at box edges
            px0 = max(x0, cx_px - (x1 - x0) // 4)
            px1 = min(x1, cx_px + (x1 - x0) // 4)
            py0 = max(y0, cy_px - (y1 - y0) // 4)
            py1 = min(y1, cy_px + (y1 - y0) // 4)
            patch = depth_m[py0:py1, px0:px1]
            valid = patch[(patch > self.depth_min) & (patch < self.depth_max) & np.isfinite(patch)]
            if valid.size == 0:
                continue
            depth = float(np.median(valid))

            # Back-project the box-center pixel to a 3D point in the camera
            # optical frame (z forward, x right, y down).
            pt = PointStamped()
            pt.header.frame_id = src_frame
            pt.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()  # latest transform
            pt.point.z = depth
            pt.point.x = (cx_px - self.cx) * depth / self.fx
            pt.point.y = (cy_px - self.cy) * depth / self.fy

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_id, src_frame, Time(seconds=0, nanoseconds=0))
                tp = do_transform_point(pt, tf)
                positions.append(tp)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'TF {src_frame}->{self.frame_id} failed: {e}',
                                       throttle_duration_sec=5.0)
        return positions

    def publish_people(self, positions):
        if not positions:
            return
        people = People()
        people.header.frame_id = self.frame_id
        people.header.stamp = self.get_clock().now().to_msg()
        for i, tp in enumerate(positions):
            person = Person()
            person.name = ''
            person.position.x = tp.point.x
            person.position.y = tp.point.y
            person.position.z = 0.0
            person.reliability = 0.8
            person.tagnames = ['RGBD']
            person.tags = [str(i)]
            people.people.append(person)
        self.people_pub.publish(people)

    def publish_markers(self, positions):
        arr = MarkerArray()
        for i, tp in enumerate(positions):
            for j, (mtype, z, scale_z) in enumerate(
                    [(Marker.CYLINDER, 0.75, 1.5), (Marker.SPHERE, 1.7, 0.4)]):
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'people_yolo'
                m.id = 10 * i + j
                m.type = mtype
                m.action = Marker.ADD
                m.pose.position.x = tp.point.x
                m.pose.position.y = tp.point.y
                m.pose.position.z = z
                m.pose.orientation.w = 1.0
                m.scale.x = 0.4
                m.scale.y = 0.4
                m.scale.z = scale_z
                m.color.a = 1.0
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                arr.markers.append(m)
        if arr.markers:
            self.markers_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPeopleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
