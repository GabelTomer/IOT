import threading
import struct
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection
import sys
from communication.pose_aggregator import PoseAggregator
from server.flaskServer import server
import random
import json
import math
import select

import requests
import math
# --- General GLOBAL Variables ---
POSE_UPDATE_THRESHOLD = 20000.0
GENERATE_ARUCO_BOARD = False
COMMUNICATION_METHOD = 'wifi'  # change to 'WiFi or i2c' when needed
CAR_IP = "192.168.0.104"
COMMAND_COOLDOWN = 3 # seconds
ANGEL_THRESHOLD = 10 # degrees


# --- GLOBAL Variables and Import specific Libraries for I2C ---
if COMMUNICATION_METHOD == 'i2c':
    import RPi.GPIO as GPIO
    import smbus2

    SLAVE_CONFIG = {
    0x08: 18,  # GPIO pin for Pi2 interrupt
    0x09: 19,  # GPIO pin for Pi3 interrupt
    }
    # GPIO setup and interrupt handling for data ready (per slave)
    GPIO.setmode(GPIO.BCM)
    ready_flags = {}

# --- GLOBAL Variables and Import specific Libraries for WiFi ---
elif COMMUNICATION_METHOD == "wifi":
    import socket
    import queue
    PORT = 6002


# --- GLOBAL Variables and Intialization of 3D Visulaization ---
combined_aruco_ids = set()
aruco_ids_lock = threading.Lock()
_gui_available = None

def is_gui_available():
    """
    Check if the script can display a GUI.
    This is the most reliable method and works on all platforms.
    """
    global _gui_available
    if _gui_available is None:
        try:
            # Create a named window
            cv2.namedWindow("test_window", cv2.WINDOW_NORMAL)
            # Attempt to move it
            cv2.moveWindow("test_window", 100, 100)
            # Destroy it immediately
            cv2.destroyWindow("test_window")
            _gui_available = True
            print("GUI is available.")
        except cv2.error:
            _gui_available = False
            print("GUI is not available (running in a headless environment or as a service).")
    return _gui_available

def make_callback(addr):
    global ready_flags
    def callback(channel):
        ready_flags[addr].set()
    return callback

def plot_updater_thread(stop_event, flaskServer = None):
    global combined_aruco_ids
    print("Started plot update thread")
    while not stop_event.is_set():
        # Update global set
            with aruco_ids_lock:
                # Call your visual update with all seen markers
                #update_pose_visual_and_stats("3D Pose Estimation", pose, aruco_ids)
                flaskServer.updateIds(list(combined_aruco_ids))
                combined_aruco_ids.clear()

            time.sleep(0.4)

def send_command(cmd):
    if not hasattr(send_command, 'last_cmd_time'):
        send_command.last_cmd_time = 0
    if not hasattr(send_command, 'last_cmd'):
        send_command.last_cmd = 'stop'

    if send_command.last_cmd == cmd and not "Short" in cmd:
        print(f"died in none none not short send_command.last_cmd  = {send_command.last_cmd } cmd = {cmd}")
        return None, None
    if abs(time.time()-send_command.last_cmd_time) <= COMMAND_COOLDOWN and send_command.last_cmd == cmd:
        print(f"died in none none time {abs(time.time()-send_command.last_cmd_time)}")
        return None, None
    url = f"http://{CAR_IP}/{cmd}"
    try:
        response = requests.get(url, timeout=0.5)
        if response.status_code == 200:
            print(f"Sent command: {cmd}")
            send_command.last_cmd = cmd
            send_command.last_cmd_time = time.time()
            return cmd, send_command.last_cmd_time
        else:
            print(f"Failed to send command: {cmd}: {response.status_code}")
        return None, None
    except requests.exceptions.RequestException as e:
        print(f"Error sending {cmd}: {e}")
        return None, None

def runServer(flaskServer: server):

    flaskServer.setup_routes()
    flaskServer.run()

def generate_aruco_board():
    # Constants
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    dpi = 300
    a4_width_px = 3508
    a4_height_px = 2480
    a4_width_m = 0.297  # 29.7 cm
    a4_height_m = 0.21  # 21 cm
    meters_to_pixels = a4_width_px / a4_width_m

    marker_size_m = 0.06
    marker_size_px = int(marker_size_m * meters_to_pixels)
    min_spacing_m = 0.03  # relaxed spacing

    canvas = 255 * np.ones((a4_height_px, a4_width_px), dtype=np.uint8)
    marker_ids = [26,27,28,29,30]

    # Safe placement function
    def generate_safe_positions(num_markers, marker_size_m, min_spacing_m):
        positions = [np.array([0.0, 0.0, 0.0])]  # ID 1 at center
        attempts = 0
        margin_x = (a4_width_m - marker_size_m) / 2 - 0.005
        margin_y = (a4_height_m - marker_size_m) / 2 - 0.005

        while len(positions) < num_markers:
            candidate = np.array([
                random.uniform(-margin_x, margin_x),
                random.uniform(-margin_y, margin_y),
                0.0
            ])
            if all(np.linalg.norm(candidate[:2] - p[:2]) >= (marker_size_m + min_spacing_m) for p in positions):
                positions.append(candidate)
            attempts += 1
            if attempts > 10000:
                print("⚠️ Warning: Too many placement attempts. Proceeding with fewer markers.")
                break
        return positions

    # Generate safe marker positions
    positions_list = generate_safe_positions(len(marker_ids), marker_size_m, min_spacing_m)
    if len(positions_list) < len(marker_ids):
        print(f"⚠️ Only {len(positions_list)} out of {len(marker_ids)} markers were placed safely.")
        marker_ids = marker_ids[:len(positions_list)]
    positions_m = {id: pos for id, pos in zip(marker_ids, positions_list)}

    # Center point on canvas
    center_px = (a4_width_px // 2, a4_height_px // 2)

    # Draw all markers
    for marker_id in marker_ids:
        if marker_id not in positions_m:
            print(f"⚠️ Skipping marker ID {marker_id} — no position assigned.")
            continue
        marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)

        x_m, y_m, _ = positions_m[marker_id]
        x_px = int(center_px[0] + x_m * meters_to_pixels)
        y_px = int(center_px[1] - y_m * meters_to_pixels)

        top_left_x = x_px - marker_size_px // 2
        top_left_y = y_px - marker_size_px // 2

        if 0 <= top_left_x < a4_width_px - marker_size_px and 0 <= top_left_y < a4_height_px - marker_size_px:
            canvas[top_left_y:top_left_y+marker_size_px, top_left_x:top_left_x+marker_size_px] = marker_img

    # Save the PNG
    cv2.imwrite("aruco_board_landscape.png", canvas)

    # Print marker coordinates
    print("✅ Marker coordinates (in meters):")
    for marker_id in marker_ids:
        x, y, z = positions_m[marker_id]
        print(f"ID {marker_id}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

def resource_path(relative_path):
    import sys, os
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.abspath(relative_path)

def kalman_filter_config():
    # === Kalman Filter Configuration ===
    kalman = cv2.KalmanFilter(3, 3, 0 , cv2.CV_64F)  # 6 state variables (pos + velocity), 3 measurements (pos only)
    # Transition matrix (state update: x = Ax + Bu + w)
    kalman.transitionMatrix = np.eye(3, dtype=np.double)  # Identity: x = x
    #Measurement matrix (we only measure position)
    kalman.measurementMatrix = np.eye(3, dtype=np.double) # observe only see the position (x, y, z), not velocity.”
    #controls how much random motion you expect in your model.
        #small value (like 1e-4) means: “I trust my motion model — things don’t move randomly.”
        #larger value (like 1e-2) would mean: “The world is noisy — objects might move unpredictably.”
    kalman.processNoiseCov = np.eye(3, dtype=np.double) * 1e-4
    #It controls how trustworthy the measurements are:
        #smaller value → “Measurements are precise and accurate.”
        #A bigger value → “I’m not sure about my measurements (e.g., noisy sensor).”
    kalman.measurementNoiseCov = np.eye(3, dtype=np.double) * 1e-2
    #initial uncertainty in your state estimate. this matrix said how much do I trust my initial guess about position and velocity
    #Using np.eye(6) means moderately uncertain about both position and velocity at the beginning.
    #increase this (e.g. 10 * np.eye(6)) to say: really don’t know where I started.
    kalman.errorCovPost = np.eye(3, dtype=np.double)
    # Initial state (0 position, 0 velocity)
    kalman.statePost = np.zeros((3, 1), dtype=np.double)
    return kalman

def wifi_listener_and_processor(aggregator, flaskServer, stop_event, port = 6002):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    print(f"[UDP] Listening and processing on port {port}")

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(512)
            if data[0:3] == b'\xFA\xF3\x20':
                x, y, z , timestamp = struct.unpack('<3fQ', data[3:23])
                aruco_list = list(data[23:])
                pose = (x, y, z)
                aggregator.update_pose(pose)
                avg_pose = aggregator.get_average_pose()
                time_now = time.time_ns() // 1000
                time_diff = (time_now - timestamp)

                if avg_pose: #and 0 <= time_diff <= POSE_UPDATE_THRESHOLD:
                    x, y, z = avg_pose
                    flaskServer.updatePosition(x, y, z)
                    with aruco_ids_lock:
                        combined_aruco_ids.update(aruco_list)

        except Exception as e:
            print(f"[UDP] Error: {e}")

def receive_from_clients(method, aggregator, flaskServer, stop_event):
    if method == 'wifi':
        threading.Thread(target=wifi_listener_and_processor, args=(aggregator, flaskServer, stop_event, PORT), daemon=True).start()

    elif method == 'i2c':
        # Use a single I2C bus for all slave addresses
        i2c_bus = {0x08: smbus2.SMBus(1), 0x09: smbus2.SMBus(3)}
        bus_map = {addr: i2c_bus for addr in SLAVE_CONFIG.keys()}
        addrs = list(SLAVE_CONFIG.keys())
        buses = [bus_map[addr] for addr in addrs]
        threading.Thread(target=i2c_listener, args=(buses, addrs, aggregator, flaskServer, stop_event), daemon=True).start()

def i2c_listener(buses, addresses, aggregator, flaskServer, stop_event):
    global ready_flags
    last_counters = {addr: 0 for addr in addresses}
    lost_packages = {addr: 0 for addr in addresses}
    packages = {addr: 0 for addr in addresses}
    pose_temp = {}
    all_connected_slaves = set()
    time_diff = 0.0
    while not stop_event.is_set():
        try:
            # Wait for any ready event
            for addr in addresses:
                bus = buses[addresses.index(addr)]
                if addr not in all_connected_slaves and GPIO.input(SLAVE_CONFIG[addr]) == GPIO.HIGH:
                    bus.write_byte(addr, 0xA5)
                    all_connected_slaves.add(addr)
                    ready_flags[addr].clear()

                if ready_flags[addr].is_set():
                    ready_flags[addr].clear()
                    try:
                        data = bus.read_i2c_block_data(addr, 0, 16)
                        if bytes(data[0:2]) == bytes([0xEB,0x90]):
                            packages[addr] += 1
                            print(f"[MASTER addr {hex(addr)}] Received:", bytes(data).hex(), len(data))
                            counter, packet_type = struct.unpack('<BB', bytes(data[2:4]))

                            if packet_type == 0x01:
                                x, y, z, timestamp = struct.unpack('<3HI', bytes(data[4:14]))
                                # Convert the 16-bit raw integers back into float16
                                pose_f16 = np.array([x, y, z], dtype=np.uint16).view(np.float16)
                                # convert to float32 for better precision
                                pose_f32 = pose_f16.astype(np.float32)
                                x, y, z = pose_f32[0], pose_f32[1], pose_f32[2]
                                pose_temp[addr] = {'counter': counter, 'pose': (x, y, z)}
                                if ((pose_temp[addr]['counter'] - last_counters[addr]) % 256) != 1:
                                    lost_packages[addr] += ((pose_temp[addr]['counter'] - last_counters[addr]) % 256)

                                last_counters[addr] = pose_temp[addr]['counter']
                                acc = 100 - ((lost_packages[addr] / packages[addr]) * 100)
                                print(f"The Accuracy of receiving messages are : {acc}%")
                                print(f"[MASTER addr {hex(addr)}] Timestamp received:", timestamp)
                                now = (time.time_ns() // 1000) & 0xFFFFFFFF
                                print(f"[MASTER addr {hex(addr)}] Current time      :", now)
                                time_diff = (now - timestamp) & 0xFFFFFFFF
                                print(f"[MASTER addr {hex(addr)}] Time diff (μs)    :", time_diff)

                                if not any(np.isnan(pose_temp[addr]['pose'])) and 0 <= time_diff <= POSE_UPDATE_THRESHOLD:
                                    aggregator.update_pose((x, y, z))
                                    pose = aggregator.get_average_pose()
                                    if pose:
                                        x, y, z = pose
                                        flaskServer.updatePosition(x, y, z)
                                        print(f"Filtered Camera Position -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")

                            elif packet_type == 0x02:
                                if addr in pose_temp and pose_temp[addr]['counter'] == counter:
                                    num_of_detected_aruco = struct.unpack('<B', bytes(data[4:5]))
                                    if num_of_detected_aruco > 0:
                                        aruco_id_list = struct.unpack(f'<{num_of_detected_aruco}B', bytes(data[5:(5 + num_of_detected_aruco)]))
                                        pose = aggregator.get_average_pose()

                    except Exception as e:
                        print(f"[I2C addr {hex(addr)}] Read error: {e}")

        except Exception as e:
            print(f"[I2C Listener] Fatal error: {e}")

def main():
    is_gui_available()
    global known_markers
    # --- Generate Aruco board for camera calibration ---
    if GENERATE_ARUCO_BOARD:
        try:
            import random
            generate_aruco_board()

        except RuntimeError as e:
            print(f"[ARUCO BOARD ERROR] {e}")

    # --- Camera Calibration ---
    recalibrate = False
    camera = Camera(gui_available=_gui_available)

    while not recalibrate:
        mean_error = camera.calibrate_camera()
        if mean_error < 0.5:
            print("Calibration is GOOD ✅")
            recalibrate = True
        else:
            print("Retrying calibration. Got error:", mean_error)

    detector = Detection(known_markers_path="core/utils/known_markers.json")

    known_markers = detector.known_markers
    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json", detector=detector, left_camera_ip="192.168.0.101", right_camera_ip="192.168.0.102")
    aggregator = PoseAggregator()
    stop_event = threading.Event()
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    threading.Thread(target=plot_updater_thread, args = (stop_event, flaskServer), daemon=True).start()
    server_thread.start()

    if COMMUNICATION_METHOD == "i2c":
        for addr, pin in SLAVE_CONFIG.items():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            ready_flags[addr] = threading.Event()
            GPIO.add_event_detect(pin, GPIO.RISING, callback=make_callback(addr))

    receive_from_clients(COMMUNICATION_METHOD, aggregator, flaskServer, stop_event)

    # --- Open Camera for Video Capturing ---
    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Error: Could not Open Video")
        sys.exit(1)
    last_turn_direction = None
    # for navigating:
    REACHED_THRESHOLD = 0.1 # meters
    smoothed_error =0
    alpha=0.3
    smoothed_heading = 0
    forward_thershold=5
    previous_pos = None
    robot_heading = 0.0
    cmd = None
    cmd_time = None
    # Main thread displays

    # === Kalman Filter Configuration ===
    kalman = kalman_filter_config()
    filtered_pos = 0
    last_rvec = None
    last_tvec = None
    # Main thread displays
    while True:

            ret, frame = video.read()
            if not ret:
                break

            corners, twoDArray, threeDArray, frame, aruco_markers_detected = detector.aruco_detect(frame=frame)
            measured = None
            forward_vector = None
            R = None
            if twoDArray is not None and threeDArray is not None and twoDArray and threeDArray:

                obj_pts = np.vstack(threeDArray)
                img_pts = np.vstack(twoDArray)
                num_points = obj_pts.shape[0]
                if obj_pts.shape[0] != img_pts.shape[0]:
                    print("[WARN] Mismatched 3D and 2D points.")
                    continue

                if num_points >= 4:
                    #flags = cv2.SOLVEPNP_ITERATIVE
                    #success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs, flags=flags)
                    # if success:
                    #     R, _ = cv2.Rodrigues(rvec)
                    #     measured = (-R.T @ tvec).astype(np.float32).reshape(3, 1)
                    success, rvec, tvec, inliners = cv2.solvePnPRansac(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs)
                    if success and inliners is not None and len(inliners) >= 4:
                        last_rvec = rvec
                        last_tvec = tvec
                        R, _ = cv2.Rodrigues(rvec)
                        measured = (-R.T @ tvec).astype(np.double).reshape(3, 1)

                elif num_points == 3:
                    flags = cv2.SOLVEPNP_ITERATIVE
                    if last_tvec is None and last_rvec is None:
                        guess_rvec = np.zeros((3, 1), dtype=np.double)
                        guess_tvec = np.zeros((3, 1), dtype=np.double)
                    else:
                        guess_rvec = last_rvec
                        guess_tvec = last_tvec

                    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs,
                                                    rvec=guess_rvec, tvec=guess_tvec,
                                                    useExtrinsicGuess=True, flags=flags)
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        measured = (-R.T @ tvec).astype(np.double).reshape(3, 1)

                elif num_points == 2:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, camera.MARKER_LENGTH, camera.camera_matrix, camera.dist_coeffs)
                    positions = []
                    for rvec, tvec in zip(rvecs, tvecs):
                        R, _ = cv2.Rodrigues(rvec)
                        pos = -R.T @ tvec.T
                        positions.append(pos)

                    measured = np.mean(positions, axis = 0).astype(np.double).reshape(3, 1)

                elif num_points == 1:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, camera.MARKER_LENGTH, camera.camera_matrix, camera.dist_coeffs)
                    rvec = rvecs[0]
                    tvec = tvecs[0]
                    R, _ = cv2.Rodrigues(rvec)
                    measured = (-R.T @ tvec.T).astype(np.double).reshape(3, 1)

            # Apply Kalman filter only if measured is valid
            if measured is not None and not np.any(np.isnan(measured)):

                #marker_id = aruco_markers_detected[0]  # Use the first detected marke
                #theta_deg = detector.thetas[str(marker_id)]
                #theta_rad = np.radians(theta_deg)

                # R_marker_to_global = np.array([
                #     [ np.cos(theta_rad), 0, np.sin(theta_rad)],
                #     [ 0,                1, 0                ],
                #     [-np.sin(theta_rad), 0, np.cos(theta_rad)]
                # ])

                # measured = R_marker_to_global @ measured


                kalman.correct(measured)
                predicted = kalman.predict()
                filtered_pos = predicted[:3]

                if 'rvec' in locals() and 'tvec' in locals():
                    cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)

                aggregator.update_pose((filtered_pos[0][0], filtered_pos[1][0], filtered_pos[2][0]))
                pose = aggregator.get_average_pose()

                aruco_markers_detected = np.array(aruco_markers_detected).flatten().tolist() if aruco_markers_detected is not None else []
                if aruco_markers_detected:
                    with aruco_ids_lock:
                        combined_aruco_ids.update(aruco_markers_detected)

                if R is not None:
                    forward_vector = R[:, 2]  # Forward vector in camera frame
                    smoothed_heading = (1-alpha)*smoothed_heading +alpha*math.atan2(forward_vector[0], forward_vector[2])  # Z, X
                    robot_heading = smoothed_heading
                if pose:
                    x, y, z = pose
                    # Update server with smoothed average
                    flaskServer.updatePosition(x, y, z, robot_heading)
                    #print Average Camera Position
                    print(f"Filtered Camera Position -> X: {x:.2f}, Y height: {y:.2f}, Z depth: {z:.2f}")

            current_pos = flaskServer.getPos()
            target_pos = flaskServer.get_target()
            if target_pos is not None:
                print(f"Target position: {target_pos}")

                dx = target_pos['x'] - current_pos['x']
                dz = target_pos['z'] - current_pos['z']
                distance = math.hypot(dx, dz)

                if distance < REACHED_THRESHOLD:
                    cmd, cmd_time = send_command("stop")
                    flaskServer.target_position = None

                    print("=== Reached Target ===")
                else:
                    angle_to_target = math.atan2(dx, dz)  # Again, atan2(X, Z) for your system

                    raw_error = angle_to_target - robot_heading
                    raw_error = (raw_error+180)%360 -180 #math.atan2(math.sin(raw_error), math.cos(raw_error))  # Normalize to [-π, π]
                    raw_error = math.degrees(raw_error)
                    smoothed_error = (1-alpha)*smoothed_error + alpha*raw_error

                            # Simple steering logic
                    if abs(smoothed_error) < forward_thershold:
                        cmd, cmd_time = send_command("forward")
                    else:
                        if   smoothed_error > ANGEL_THRESHOLD :

                            cmd, cmd_time = send_command("leftShort")

                        elif smoothed_error < -ANGEL_THRESHOLD :

                            cmd, cmd_time = send_command("rightShort")



                previous_pos = current_pos

            #else:
                #print("[ERROR] twoDArray or threeDArray is None!")
                #cmd, cmd_time = send_command("stop") # Turn around to find markers

            if _gui_available:
                cv2.imshow("Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break
            flaskServer.update_last_command(cmd,cmd_time)

    cv2.destroyAllWindows()

    #Now wait for all threads to end
    server_thread.join()
    #averaging_thread.join()

if __name__ == "__main__":
    main()
