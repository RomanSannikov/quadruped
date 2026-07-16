from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import io
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import json
import queue

from picamera2 import Picamera2
from ultralytics import YOLO

import utils


class Camera:
    def __init__(self):
        self.picam = Picamera2()
        self.picam.configure(self.picam.create_preview_configuration({'size': (320, 320), 'format': 'RGB888'}))
        self.picam.start()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.camera_running = threading.Event()
        self.camera_running.set()
        self.detection_enabled = True
        # Queue to pass frames (numpy arrays) from the capture thread to the detection worker
        self.frame_queue = queue.Queue(maxsize=1)
        # Statistics for FPS
        self.latest_stats = {'fps': 0}
        self.stats_lock = threading.Lock()
        self._frame_counter = 0
        self._last_stats_time = time.time()
        self.detection_count = 0


    def start(self):
        # Start camera thread
        self.camera_thread = threading.Thread(target=self.camera_worker, daemon=True)
        self.camera_thread.start()
        # Start detection thread
        self.detection_thread = threading.Thread(target=self.detection_worker, daemon=True)
        self.detection_thread.start()


    def stop(self):
        self.camera_running.clear()
        self.camera_thread.join(timeout=1.0)
        self.detection_thread.join(timeout=1.0)


    def _record_frame(self):
        with self.stats_lock:
            self._frame_counter += 1
            now = time.time()
            elapsed = now - self._last_stats_time
            if elapsed >= 1.0:
                fps = self._frame_counter / elapsed
                self.latest_stats['fps'] = int(fps)
                self._frame_counter = 0
                self._last_stats_time = now


    def get_latest_frame(self):
        with self.frame_lock:
            return self.latest_frame


    def get_latest_stats(self):
        with self.stats_lock:
            return self.latest_stats.copy()


    def get_detection_state(self):
        return self.detection_enabled


    def set_detection_state(self, state: bool):
        self.detection_enabled = state


    def camera_worker(self):
        while self.camera_running.is_set():
            frame = self.picam.capture_array()
            if frame is None:
                time.sleep(0.01)
                continue
            # BRG to RGB, without it I am a smurf
            frame = frame[..., ::-1]
            im = Image.fromarray(frame)
            # Convert to numpy for the detection worker
            # Enqueue frame for detection; if the queue is full, drop the oldest frame
            if self.frame_queue.full():
                self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(frame)
            # If detection is disabled, produce a preview JPEG for streaming
            if not self.detection_enabled:
                buf = io.BytesIO()
                im.save(buf, format='JPEG')
                data = buf.getvalue()
                with self.frame_lock:
                    self.latest_frame = data
                    self._record_frame()
            time.sleep(0.03)


    def detection_worker(self):
        yolo_model = YOLO('best_ncnn_model', task='detect')
        while self.camera_running.is_set():
            try:
                img_np = self.frame_queue.get(timeout=0.5)
            except Exception:
                continue
            im = Image.fromarray(img_np)
            # Run detection if enabled
            if self.detection_enabled:
                # Detect objects in the frame 
                results = yolo_model(img_np, imgsz=320, conf=0.25, workers=2)
                if len(results) > 0:
                    r = results[0]
                    xyxy = r.boxes.xyxy.numpy()
                    confs = r.boxes.conf.numpy()
                    clss = r.boxes.cls.numpy()
                    if len(xyxy):
                        draw = ImageDraw.Draw(im)
                        font = ImageFont.load_default()
                        for box, conf, cls in zip(xyxy, confs, clss):
                            x1, y1, x2, y2 = map(int, box)
                            class_id = int(cls)
                            label = yolo_model.names[class_id]
                            box_color = 'red' if class_id == 0 else 'blue'
                            label_conf = f"{label} {conf:.2f}"
                            # Measure text size
                            left_t, top_t, right_t, bottom_t = draw.textbbox((0, 0), label_conf, font=font)
                            text_w = int(right_t - left_t)
                            text_h = int(bottom_t - top_t)
                            pad = 2
                            img_w, img_h = im.size
                            # Preferred label position: above the box
                            desired_label_y1 = y1 - text_h - pad
                            # If label would be out of frame above, try shifting the box down minimally
                            if desired_label_y1 < 0:
                                shift_down = -desired_label_y1
                                y1_new = y1 + shift_down
                                # Ensure box doesn't invert or become too small (min height 4)
                                if y1_new + 4 <= y2:
                                    y1 = int(y1_new)
                                    label_y1 = 0
                                else:
                                    # Draw label inside the box near the bottom
                                    label_y1 = max(0, min(img_h - text_h - pad, y2 - text_h - pad))
                            else:
                                label_y1 = int(desired_label_y1)
                            # Clamp box coords to image bounds
                            x1 = max(0, min(int(x1), img_w - 1))
                            y1 = max(0, min(int(y1), img_h - 1))
                            x2 = max(0, min(int(x2), img_w - 1))
                            y2 = max(0, min(int(y2), img_h - 1))
                            # Draw box
                            draw.rectangle([x1, y1, x2, y2], outline=box_color, width=2)
                            # Draw label background and text
                            label_bg_x2 = min(img_w, x1 + text_w + pad)
                            label_bg_y2 = min(img_h, label_y1 + text_h + pad)
                            draw.rectangle([x1, label_y1, label_bg_x2, label_bg_y2], fill=box_color)
                            draw.text((x1 + 1, label_y1 + 1), label_conf, fill='white', font=font)
                        # update detection counter
                        with self.stats_lock:
                            self.detection_count += len(xyxy)
                            self.latest_stats['detections'] = self.detection_count
            # Encode to JPEG and store for streaming
            buf = io.BytesIO()
            im.save(buf, format='JPEG')
            data = buf.getvalue()
            with self.frame_lock:
                self.latest_frame = data
                self._record_frame()


class HTTPServer(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            try:
                with open('index.html', 'r') as f:
                    content = f.read()
                self.wfile.write(content.encode('utf-8'))
            except FileNotFoundError:
                self.send_error(404, "index.html not found")
        elif self.path == '/video_feed':
            # MJPEG stream delivered from the background Picamera2 worker
            boundary = b'--FRAME'
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            try:
                while True:
                    frame = camera.get_latest_frame()
                    if frame is None:
                        continue
                    try:
                        self.wfile.write(boundary + b"\r\n")
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(0.03)
            except Exception:
                return
        elif self.path == '/stats':
            try:
                payload = json.dumps(camera.get_latest_stats()).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except Exception:
                self.send_error(500)
        elif self.path == '/toggle_detection':
            try:
                if camera is None:
                    raise RuntimeError('camera not initialized')
                new_state = not camera.get_detection_state()
                camera.set_detection_state(new_state)
                payload = json.dumps({'detection_enabled': new_state}).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except Exception:
                self.send_error(500)
        else:
            self.send_error(404)


    def do_POST(self):
        if robot and robot.move(self.path):
            self.send_response(200)
            self.end_headers()
        else:
            self.send_error(404)


class Robot:
    def __init__(self):
        # Initialize robot components
        self.kit = utils.init()
        self.calib_map = utils.read_calibration_data()
        self.robot_state = 'IDLE'
        self.last_request_time = time.time()
        self.legs_position = None


    def walk(self, func, *args, **kwargs):
        func(*args, **kwargs)
        self.robot_state = 'STANDING'


    def check_timeout(self):
        while True:
            time.sleep(1)
            current_time = time.time()
            if ((current_time - self.last_request_time) > 3) and self.robot_state == 'STANDING':
                self.robot_state = 'IDLE'
                utils.laydown(self.kit, self.legs_position)
                time.sleep(1)


    def start(self):
        timeout_thread = threading.Thread(target=self.check_timeout, daemon=True)
        timeout_thread.start()


    def move(self, movement) -> bool:
        self.last_request_time = time.time()
        if self.robot_state == 'IDLE':
            self.legs_position = utils.standup(self.kit, self.calib_map, x_start=-20, y_ground=-165)
            self.robot_state = 'STANDING'
        elif self.robot_state == 'MOVING':
            return True
        self.robot_state = 'MOVING'
        x_start, x_end = (-20, 20) if movement == '/moveForward' else (20, -20)
        if movement == '/moveForward' or movement == '/moveBackwards':
            # Run trot in a background thread so server doesn't block
            threading.Thread(target=self.walk, args=(utils.trot, self.kit, self.calib_map, x_start, x_end, -165, 30, 5), daemon=True).start()
        elif movement == '/walkSidewaysLeft' or movement == '/walkSidewaysRight':
            # Run sideways walk in a background thread so server doesn't block
            right = (movement == '/walkSidewaysRight')
            threading.Thread(target=self.walk, args=(utils.turn_in_place, self.kit, self.calib_map, x_start, x_end, -165, 30, 5, not right), daemon=True).start()
        else:
            return False
        return True


if __name__ == '__main__':
    camera = Camera()
    camera.start()

    robot = Robot()
    robot.start()

    server_address = ('', 8000)
    http_server = ThreadingHTTPServer(server_address, HTTPServer)
    http_server.camera = camera
    http_server.robot = robot

    print(f'Server running on port {server_address[1]}...')
    http_server.serve_forever()
