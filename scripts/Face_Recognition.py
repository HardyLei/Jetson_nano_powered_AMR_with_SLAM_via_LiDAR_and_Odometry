#!/usr/bin/env python3
"""
AMR face recognition + door control (Jetson Nano, CSI camera).

Detects faces (OpenCV Haar), recognises people against the scripts/known_faces/
folder using `face_recognition`, and triggers the Arduino door servo on A3
when a known person is seen.

The firmware moves the servo from 0 to 180 degrees for five seconds and
returns to 0. It handles the timer even if face recognition stops.

On the Jetson, ./scripts/start_robot.sh starts this script on the host and
forwards its DOOR,OPEN requests through the running Arduino bridge. Place
reference photos in scripts/known_faces/. To skip the camera, start the robot
with AMR_FACE_RECOGNITION=0.

For a standalone test with the Arduino bridge stopped, run on the Jetson host:

    python3 scripts/Face_Recognition.py --arduino-port /dev/amr

Reference photo layout — one image per person, the filename is the label:
    scripts/known_faces/
        ishmam.jpeg
        rafay.jpeg

Dependencies (host):
  * JetPack cv2 (has GStreamer) — do NOT pip install opencv-python.
  * pip3 install face_recognition pyserial
"""

import argparse
import glob
import os
from pathlib import Path
import socket
import sys
import time

import cv2

try:
    import face_recognition
    HAS_RECOG = True
except ImportError:
    HAS_RECOG = False

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


KNOWN_FACES_DIR = Path(__file__).resolve().with_name('known_faces')


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


def load_known_faces(known_dir):
    """Return (encodings, names) for every reference photo in known_dir."""
    encodings, names = [], []
    if not HAS_RECOG:
        return encodings, names
    files = []
    for ext in ('*.jpg', '*.jpeg', '*.png'):
        files += glob.glob(os.path.join(known_dir, ext))
    for path in sorted(files):
        label = os.path.splitext(os.path.basename(path))[0]
        image = face_recognition.load_image_file(path)
        locs = face_recognition.face_locations(image)
        if not locs:
            print(f'WARNING: no face found in {path} — skipping.', file=sys.stderr)
            continue
        # use the largest face in the reference photo
        locs.sort(key=lambda l: (l[2] - l[0]) * (l[1] - l[3]), reverse=True)
        encodings.append(face_recognition.face_encodings(
            image, known_face_locations=[locs[0]])[0])
        names.append(label)
        print(f'Loaded known face: {label}')
    return encodings, names


def recognize(face_enc, known_encodings, known_names, tolerance):
    """Return (name, distance) for the closest known face."""
    if not known_encodings:
        return 'Unknown', None
    distances = face_recognition.face_distance(known_encodings, face_enc)
    best = int(distances.argmin())
    name = known_names[best] if distances[best] <= tolerance else 'Unknown'
    return name, float(distances[best])


def send_door_open(ser, door_socket):
    """Send one open request to the bridge or to a standalone Arduino."""
    try:
        if ser is not None:
            ser.write(b'DOOR,OPEN\n')
        else:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
                client.setblocking(False)
                client.sendto(b'DOOR,OPEN', str(door_socket))
        return True
    except OSError as exc:
        print(f'WARNING: cannot send door request: {exc}', file=sys.stderr)
        return False


def enroll_face(name, args):
    """Capture a face from the camera and save it to known_faces/<name>.jpeg."""
    cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    if cascade.empty():
        print('ERROR: could not load the Haar cascade.', file=sys.stderr)
        return 1

    cap = cv2.VideoCapture(gstreamer_pipeline(
        sensor_id=args.sensor, capture_width=args.width,
        capture_height=args.height, display_width=int(args.width * args.scale),
        display_height=int(args.height * args.scale),
        framerate=args.fps, flip_method=args.flip), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print('ERROR: failed to open the CSI camera. Run on the Jetson host.',
              file=sys.stderr)
        return 1

    os.makedirs(args.known_dir, exist_ok=True)
    out_path = os.path.join(args.known_dir, name + '.jpeg')
    print(f"Enrolling '{name}'. Look straight at the camera.")
    print("Press 's' to save, 'q' to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            boxes = cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
            for (x, y, w, h) in boxes:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.imshow('Enroll ' + name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                if len(boxes) == 0:
                    print('No face detected - get closer / improve light.')
                    continue
                (x, y, w, h) = max(boxes, key=lambda b: b[2] * b[3])
                pad = int(0.3 * w)
                x0 = max(0, x - pad)
                y0 = max(0, y - pad)
                x1 = min(frame.shape[1], x + w + pad)
                y1 = min(frame.shape[0], y + h + pad)
                cv2.imwrite(out_path, frame[y0:y1, x0:x1])
                print(f'Saved {out_path}')
                break
            if key == ord('q'):
                print('Cancelled.')
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


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
    parser.add_argument('--known-dir', default=KNOWN_FACES_DIR,
                        help='reference photo folder (defaults to scripts/known_faces)')
    parser.add_argument('--tolerance', type=float, default=0.6,
                        help='lower = stricter match (0.4-0.6 typical)')
    parser.add_argument('--enroll', metavar='NAME',
                        help='save a reference photo as <NAME>.jpeg in the known faces folder')
    parser.add_argument('--arduino-port',
                        help='direct serial port for standalone use with the bridge stopped')
    parser.add_argument('--arduino-baud', type=int, default=115200)
    parser.add_argument('--no-door', action='store_true',
                        help='recognise only; do not open the serial door control')
    parser.add_argument('--no-display', action='store_true',
                        help='run headless (no on-screen window)')
    args = parser.parse_args()

    if args.enroll:
        return enroll_face(args.enroll, args)

    if not HAS_RECOG:
        print('WARNING: face_recognition not installed — detection only.',
              file=sys.stderr)

    known_encodings, known_names = load_known_faces(args.known_dir)
    if HAS_RECOG and not known_encodings:
        print(f'WARNING: no reference faces loaded from {args.known_dir}.', file=sys.stderr)

    cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    if cascade.empty():
        print('ERROR: could not load the Haar cascade.', file=sys.stderr)
        return 1

    cap = cv2.VideoCapture(gstreamer_pipeline(
        sensor_id=args.sensor, capture_width=args.width,
        capture_height=args.height, display_width=int(args.width * args.scale),
        display_height=int(args.height * args.scale),
        framerate=args.fps, flip_method=args.flip), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print('ERROR: failed to open the CSI camera. Run on the Jetson host.',
              file=sys.stderr)
        return 1

    ser = None
    door_socket = None
    if not args.no_door:
        if args.arduino_port is None:
            door_socket = Path(__file__).resolve().parents[1] / 'ros2_ws/build/amr-door.sock'
        elif not HAS_SERIAL:
            print('WARNING: pyserial not installed; door control disabled.', file=sys.stderr)
        else:
            try:
                ser = serial.Serial(
                    args.arduino_port, args.arduino_baud, write_timeout=0.5, exclusive=True,
                )
            except OSError as exc:
                print(f'WARNING: cannot open {args.arduino_port}: {exc}. '
                      'Door control disabled.', file=sys.stderr)

    door_target = args.arduino_port if ser is not None else door_socket
    triggered = False
    next_door_attempt = 0.0
    previous_status = None
    print(f'Recognition running. known={known_names} display={not args.no_display} '
          f'door={door_target or "off"}')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)
            boxes = cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            labels = []
            for (x, y, w, h) in boxes:
                name = 'Unknown'
                if HAS_RECOG:
                    encs = face_recognition.face_encodings(
                        rgb, known_face_locations=[(y, x + w, y + h, x)])
                    if encs:
                        name, _ = recognize(
                            encs[0], known_encodings, known_names, args.tolerance)
                labels.append((x, y, w, h, name))

            present = sorted({name for *_, name in labels})
            summary = ', '.join(present) if present else 'no faces'
            status = f'{len(labels)} face(s): {summary}'
            if status != previous_status:
                print(status)
                previous_status = status

            # Trigger once while a known face remains in view. The Arduino
            # closes the door independently after five seconds.
            known_present = any(name != 'Unknown' for *_, name in labels)
            now = time.monotonic()
            if not known_present:
                triggered = False
            elif door_target and not triggered and now >= next_door_attempt:
                triggered = send_door_open(ser, door_socket)
                next_door_attempt = now + 1.0
                if triggered:
                    print('DOOR: OPEN requested')

            for (x, y, w, h, name) in labels:
                color = (0, 255, 0) if name != 'Unknown' else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, name, (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if not args.no_display:
                cv2.imshow('AMR face recognition', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        if ser is not None:
            ser.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
