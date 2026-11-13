#!/usr/bin/env python3
"""Publish the G1 Innomaker UVC camera in the mapping camera JSON format.

Timestamps are host realtime immediately after frame retrieval, not hardware
exposure timestamps. No intrinsic or LiDAR-to-camera calibration is implied.
"""
import argparse
import base64
import json
import signal
import time

import cv2
import zmq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0')
    parser.add_argument('--port', type=int, default=5555)
    parser.add_argument('--fps', type=float, default=5.0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error('fps must be positive')
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError('cannot open camera: ' + args.device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    context = zmq.Context()
    sock = context.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.bind('tcp://*:%d' % args.port)
    running = [True]
    def stop(signum, frame):
        running[0] = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print('Innomaker camera started on port %d; rotating 90 degrees CCW' % args.port, flush=True)
    next_send = 0.0
    try:
        while running[0]:
            ok, frame = cap.read()
            captured = time.time()
            if not ok:
                raise RuntimeError('camera frame read failed')
            now = time.monotonic()
            if now < next_send:
                continue
            next_send = now + 1.0 / args.fps
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            payload = {'images': {'head_camera': base64.b64encode(encoded).decode('ascii')},
                       'timestamps': {'head_camera': captured}}
            sock.send_string(json.dumps(payload, separators=(',', ':')))
    finally:
        cap.release()
        sock.close()
        context.term()


if __name__ == '__main__':
    main()
