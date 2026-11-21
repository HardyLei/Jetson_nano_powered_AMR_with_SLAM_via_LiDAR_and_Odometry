#!/usr/bin/env python3
"""
AMR face detection (CSI camera, Jetson Nano) — FIRST MILESTONE: detection only.

Detects faces from the Raspberry Pi camera (CSI / IMX219) using an
nvarguscamerasrc GStreamer pipeline and OpenCV Haar cascades, then logs each
frame's result and optionally publishes it on a ROS 2 topic.

Run on the JETSON HOST (the ros:humble container has no nvarguscamerasrc):

    python3 scripts/face_recognition.py

Usage:
    python3 face_recognition.py [--width 1280] [--height 720] [--fps 30]
                               [--flip 0] [--sensor 0] [--no-display]
                               [--ros] [--topic /face_detections]

Notes:
  * Use JetPack's system cv2 (built with GStreamer). A pip `opencv-python`
    wheel does NOT include GStreamer, so cv2.VideoCapture(..., CAP_GSTREAMER)
    would fail.
  * No identity recognition yet; this only detects that faces are present.
"""

import argparse
import sys
import time

import cv2

# ROS 2 is optional. Only used when --ros is given and rclpy is importable.
try:
    import rclpy
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720,
                       display_width=1280, display_height=720,
                       framerate=30, flip_method=0):
    """Canonical Jetson Nano CSI (nvarguscamerasrc) GStreamer pipeline."""
    return (
        f"nvarguscamerasrc sensor_id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, "
        f"height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, "
        f"height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--width', type=int, default=1280, help='capture width')
    parser.add_argument('--height', type=int, default=720, help='capture height')
    parser.add_argument('--fps', type=int, default=30, help='capture framerate')
    parser.add_argument('--flip', type=int, default=0,
                        help='nvvidconv flip-method (0 = normal, 2 = upside down)')
    parser.add_argument('--sensor', type=int, default=0, help='CSI sensor_id')
    parser.add_argument('--scale', type=float, default=0.5,
                        help='downscale factor to speed up detection')
    parser.add_argument('--no-display', action='store_true',
                        help='run headless (no on-screen window)')
    parser.add_argument('--ros', action='store_true',
                        help='publish detections over ROS 2 when available')
    parser.add_argument('--topic', default='/face_detections',
                        help='ROS 2 topic to publish on')
    args = parser.parse_args()

    if args.ros and not HAS_ROS:
        print('WARNING: rclpy is not importable; continuing without ROS 2.',
              file=sys.stderr)

    node = None
    publisher = None
    if args.ros and HAS_ROS:
        rclpy.init()
        node = rclpy.create_node('face_detection')
        publisher = node.create_publisher(String, args.topic, 10)

    # Haar cascade ships inside OpenCV; no model download required.
    cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    if cascade.empty():
        print('ERROR: could not load the Haar cascade.', file=sys.stderr)
        return 1

    pipeline = gstreamer_pipeline(
        sensor_id=args.sensor,
        capture_width=args.width,
        capture_height=args.height,
        display_width=int(args.width * args.scale),
        display_height=int(args.height * args.scale),
        framerate=args.fps,
        flip_method=args.flip,
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print('ERROR: failed to open the CSI camera. Run this on the Jetson '
              'host (not inside the ros:humble container).', file=sys.stderr)
        return 1

    print(f'Face detection running. display={not args.no_display} '
          f'ros={args.ros and HAS_ROS}')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('WARNING: dropped/empty frame.', file=sys.stderr)
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            faces = cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            summary = f'{len(faces)} face(s) detected'
            print(summary)
            for (x, y, w, h) in faces:
                print(f'  face x={x} y={y} w={w} h={h}')
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if publisher is not None:
                publisher.publish(String(data=summary))
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.001)
            if not args.no_display:
                cv2.imshow('AMR face detection', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
