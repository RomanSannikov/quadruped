from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time

import utils


# Initialize robot components
kit = utils.init()
calib_map = utils.read_calibration_data()
robot_state = 'IDLE'
last_request_time = time.time()
legs_position = None


class MyHandler(BaseHTTPRequestHandler):
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
        else:
            self.send_error(404)


    def do_POST(self):
        global last_request_time, robot_state, legs_position

        if self.path == '/moveForward' or self.path == '/moveBackwards':
            last_request_time = time.time()
            if robot_state == 'IDLE':
                legs_position = utils.standup(kit, calib_map, x_start=-20, y_ground=-165)
                robot_state = 'STANDING'
            elif robot_state == 'MOVING':
                print('return')
                return
            robot_state = 'MOVING'
            x_start, x_end = (-20, 20) if self.path == '/moveForward' else (20, -20)
            # Run trot in a background thread so server doesn't block
            threading.Thread(target=run_trot, args=(utils.trot, kit, calib_map, x_start, x_end, -165, 30, 5), daemon=True).start()
            self.send_response(200)
            self.end_headers()
        else:
            self.send_error(404)


def run_trot(func, *args, **kwargs):
    global robot_state

    func(*args, **kwargs)
    robot_state = 'STANDING'


def check_timeout():
    global last_request_time, robot_state, legs_position
    while True:
        time.sleep(1)
        current_time = time.time()
        if ((current_time - last_request_time) > 3) and robot_state == 'STANDING':
            robot_state = 'IDLE'
            utils.laydown(kit, legs_position)
            time.sleep(1)


if __name__ == '__main__':
    server_address = ('', 8000)
    httpd = HTTPServer(server_address, MyHandler)

    timeout_thread = threading.Thread(target=check_timeout, daemon=True)
    timeout_thread.start()

    print('Server running on port 8000...')    
    httpd.serve_forever()
