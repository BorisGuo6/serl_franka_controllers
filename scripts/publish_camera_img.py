#!/usr/bin/env python3
import glob
import os

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

DEFAULT_CAMERA_ID = 8
DEFAULT_SERIAL = "f1181690"
DEFAULT_RESOLUTIONS = [
    (640, 480),
]
DEFAULT_TOPIC = "/camera2/image_raw"
DEFAULT_FRAME_ID = "camera"
DEFAULT_RATE = 25


def _resolve_device():
    camera_device = rospy.get_param("~camera_device", "")
    if camera_device:
        return camera_device

    serial = rospy.get_param("~serial", DEFAULT_SERIAL)
    prefer_rgb = bool(rospy.get_param("~prefer_rgb", True))
    video_index = str(rospy.get_param("~video_index", "0"))
    by_id_dir = rospy.get_param("~by_id_dir", "/dev/v4l/by-id")

    if serial and os.path.isdir(by_id_dir):
        pattern = os.path.join(by_id_dir, f"*{serial}*")
        candidates = sorted(glob.glob(pattern))
        if candidates:
            filtered = candidates
            if video_index:
                filtered = [c for c in filtered if f"video-index{video_index}" in c]
            if prefer_rgb:
                rgb = [c for c in filtered if "RGB" in c or "rgb" in c]
                if rgb:
                    return rgb[0]
            if filtered:
                return filtered[0]
            return candidates[0]

    return int(rospy.get_param("~camera_id", DEFAULT_CAMERA_ID))


def _get_resolutions():
    resolutions = rospy.get_param("~resolutions", DEFAULT_RESOLUTIONS)
    if isinstance(resolutions, (list, tuple)) and resolutions:
        parsed = []
        for entry in resolutions:
            if isinstance(entry, (list, tuple)) and len(entry) == 2:
                try:
                    parsed.append((int(entry[0]), int(entry[1])))
                except (TypeError, ValueError):
                    continue
        if parsed:
            return parsed
    return list(DEFAULT_RESOLUTIONS)


def _open_camera(device, resolutions):
    candidates = [
        (device, cv2.CAP_V4L2),
        (device, cv2.CAP_ANY),
    ]

    for dev, backend in candidates:
        cap = cv2.VideoCapture(dev, backend)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        for width, height in resolutions:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            rospy.sleep(0.2)

            ret, frame = cap.read()
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            if ret and frame is not None:
                rospy.loginfo(
                    "Camera %s opened with backend %s. Requested: %dx%d, Actual: %dx%d",
                    dev,
                    backend,
                    width,
                    height,
                    actual_width,
                    actual_height,
                )
                return cap

            rospy.logwarn(
                "Camera %s opened with backend %s but no frame read at %dx%d (Actual: %dx%d).",
                dev,
                backend,
                width,
                height,
                actual_width,
                actual_height,
            )

        cap.release()

    rospy.logerr("Failed to open camera device: %s", device)
    return None


def main():
    rospy.init_node("camera_publisher", anonymous=True)

    device = _resolve_device()
    resolutions = _get_resolutions()
    topic_name = rospy.get_param("~topic", DEFAULT_TOPIC)
    frame_id = rospy.get_param("~frame_id", DEFAULT_FRAME_ID)
    publish_rate = float(rospy.get_param("~publish_rate", DEFAULT_RATE))

    if publish_rate <= 0.0:
        rospy.logwarn("publish_rate must be > 0.0, using %d", DEFAULT_RATE)
        publish_rate = float(DEFAULT_RATE)

    camera_cap = _open_camera(device, resolutions)
    if camera_cap is None:
        rospy.logerr("Camera not available. Shutting down.")
        return

    publisher = rospy.Publisher(topic_name, Image, queue_size=10)
    rospy.loginfo("Publishing frames to %s at %.2f Hz", topic_name, publish_rate)

    bridge = CvBridge()
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        ret, frame = camera_cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame from camera %s. Reconnecting...", device)
            camera_cap.release()
            rospy.sleep(0.2)
            device = _resolve_device()
            camera_cap = _open_camera(device, resolutions)
            if camera_cap is None:
                rospy.logerr("Camera re-initialization failed. Shutting down.")
                break
            continue

        try:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = frame_id
            publisher.publish(ros_image)
        except CvBridgeError as exc:
            rospy.logerr("CvBridge error for camera %s: %s", device, exc)

        rate.sleep()

    if camera_cap is not None:
        camera_cap.release()
    rospy.loginfo("Camera node shutting down.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
