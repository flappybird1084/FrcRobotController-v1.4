import argparse
import cv2
import socket
import json
import time
import os
import sys
import math
import logging

import vision_engine

# --- CONFIGURATION ---
UDP_IP = "10.102.52.2"  # RoboRIO IP
UDP_PORT = 5800  # port at which roborio is listening for vision data
MAP_FILE = "field_map.json"
TAG_SIZE = 0.1651  # Meters (6.5 inches)

# Camera Calibration - Replace with your 2026 calibration results!
FOCAL_LENGTH = 650
#CX, CY = 640, 360
CX, CY = 640, 240

# Pose offsets (meters)
X_OFFSET = 0.533
Y_OFFSET = 0.091
#IMPORTANT: X IS LATERAL, Y IS HEIGHT, Z IS FWD/BACKWD
#chud


def load_field_map(filepath):
    if os.path.exists(filepath):
        with open(filepath, "r") as f:
            return json.load(f)
    print(f"Error: {filepath} not found!")
    return {}


def main():
    parser = argparse.ArgumentParser(description="AprilTag UDP vision streamer")
    parser.add_argument("--debug", action="store_true", help="Print each detection")
    parser.add_argument("--verbose", action="store_true", help="Verbose camera/debug logs")
    args = parser.parse_args()
    debug = args.debug
    verbose = args.verbose

    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    # 1. Load your Tag Data
    field_map = load_field_map(MAP_FILE)

    # 2. Setup UDP & Detector
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logging.info("UDP socket created for %s:%s", UDP_IP, UDP_PORT)
    except PermissionError as exc:
        print(
            "Warning: UDP socket creation failed due to permissions. "
            "Continuing without network output."
        )
        print(f"Details: {exc}")
        logging.warning("UDP socket creation failed: %s", exc)
    vision_engine.set_tag_family("tag36h11")
    vision_engine.configure_detector()
    logging.info("Vision engine configured. Tag family: tag36h11")

    def rotation_matrix_to_quaternion(rot):
        r00, r01, r02 = rot[0]
        r10, r11, r12 = rot[1]
        r20, r21, r22 = rot[2]

        trace = r00 + r11 + r22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r21 - r12) / s
            qy = (r02 - r20) / s
            qz = (r10 - r01) / s
        elif r00 > r11 and r00 > r22:
            s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
            qw = (r21 - r12) / s
            qx = 0.25 * s
            qy = (r01 + r10) / s
            qz = (r02 + r20) / s
        elif r11 > r22:
            s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
            qw = (r02 - r20) / s
            qx = (r01 + r10) / s
            qy = 0.25 * s
            qz = (r12 + r21) / s
        else:
            s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
            qw = (r10 - r01) / s
            qx = (r02 + r20) / s
            qy = (r12 + r21) / s
            qz = 0.25 * s

        return qw, qx, qy, qz

    def open_camera(index=0, retry_seconds=2):
        # Prefer OpenCV's default backend selection; on macOS try AVFoundation first.
        backends = [None, cv2.CAP_V4L2, cv2.CAP_ANY]
        if sys.platform == "darwin":
            backends = [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY]

        if isinstance(index, int):
            indices = [index] + [i for i in range(4) if i != index]
        else:
            indices = list(index) if isinstance(index, (list, tuple)) else [0, 1, 2, 3]
        logging.debug("Camera probe indices=%s backends=%s", indices, backends)
        for idx in indices:
            for backend in backends:
                backend_name = "DEFAULT" if backend is None else str(backend)
                logging.debug("Trying camera index=%s backend=%s", idx, backend_name)
                cap = cv2.VideoCapture(idx) if backend is None else cv2.VideoCapture(idx, backend)
                if cap.isOpened():
                    # 2026 Tip: Set high shutter speed/low exposure to stop motion blur
                    cap.set(cv2.CAP_PROP_EXPOSURE, 5)
                    logging.info("Camera opened index=%s backend=%s", idx, backend_name)
                    return cap
                cap.release()
            if sys.platform.startswith("linux"):
                device_path = f"/dev/video{idx}"
                logging.debug("Trying camera device path=%s", device_path)
                cap = cv2.VideoCapture(device_path)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_EXPOSURE, 5)
                    logging.info("Camera opened device path=%s", device_path)
                    return cap
                cap.release()
        return None

    cap = open_camera(0)
    if cap is None:
        print("Camera not detected. Retrying until available...")
        logging.warning("Camera not detected on initial probe.")

    print(f"Vision System Online. Tracking {len(field_map)} tags.")
    logging.info("Vision System Online. Tracking %s tags.", len(field_map))

    last_log = time.time()
    frame_count = 0

    while True:
        if cap is None:
            time.sleep(2)
            cap = open_camera(0)
            if cap is None:
                continue
            print("Camera detected. Resuming capture.")
            logging.info("Camera detected. Resuming capture.")

        ret, frame = cap.read()
        if not ret:
            cap.release()
            cap = None
            print("Camera read failed. Waiting for camera to return...")
            logging.warning("Camera read failed. Releasing and retrying.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = vision_engine.detect_tags(
            gray, FOCAL_LENGTH, FOCAL_LENGTH, CX, CY, TAG_SIZE
        )
        frame_count += 1
        if time.time() - last_log >= 2.0:
            logging.info("Frames=%s Detections=%s", frame_count, len(detections))
            last_log = time.time()
            frame_count = 0

        packet = []
        for det in detections:
            tag_id = str(det.get("id"))

            # Only process if it's in our JSON map
            if tag_id in field_map:
                pose = det.get("pose")
                if not pose:
                    continue
                rotation = pose.get("rotation")
                translation = pose.get("translation")
                if rotation is None or translation is None:
                    continue

                # Translation: X (right), Y (down), Z (forward) from camera
                t = translation
                # Rotation: Rotation of the tag relative to the camera
                qw, qx, qy, qz = rotation_matrix_to_quaternion(rotation)

                det_payload = {
                    "id": int(tag_id),
                    "x": round(float(t[0] + X_OFFSET), 3),
                    "y": round(float(t[1] + Y_OFFSET), 3),
                    "z": round(float(t[2]), 3),
                    "rot": [
                        round(qw, 3),
                        round(qx, 3),
                        round(qy, 3),
                        round(qz, 3),
                    ],
                }
                packet.append(det_payload)
                if debug:
                    print(det_payload)

        # 3. Stream to RoboRIO
        if packet and sock is not None:
            message = json.dumps(packet).encode()
            try:
                sock.sendto(message, (UDP_IP, UDP_PORT))
                logging.debug("Sent packet with %s detections", len(packet))
            except Exception as e:
                print(f"Network Error: {e}")
                logging.warning("Network error while sending UDP: %s", e)

        # Keep CPU usage in check
        #time.sleep(0.01)


if __name__ == "__main__":
    main()
