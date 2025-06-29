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
COMMAND_COOLDOWN = 0.5 # seconds 
REACHED_THRESHOLD = 0.2 # meters

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


# --- GLOBAL Variables and Intialization of 3D Visulaization ---
plot_lock = threading.Lock()

combined_aruco_ids = {}
aruco_ids = []
aruco_ids_lock = threading.Lock()
known_markers = {}

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

def plot_updater_thread(aggregator, stop_event, flaskServer = None):
    global aruco_ids,combined_aruco_ids
    print("Started plot update thread")
    while not stop_event.is_set():
        # Update global set
        with aruco_ids_lock:
            aruco_ids = list(combined_aruco_ids)

        # Call your visual update with all seen markers
        #update_pose_visual_and_stats("3D Pose Estimation", pose, aruco_ids)
        flaskServer.updateIds(aruco_ids)
        time.sleep(0.02)

def send_command(cmd):
    if not hasattr(send_command, 'last_cmd_time'):
        send_command.last_cmd_time = 0
    if not hasattr(send_command, 'last_cmd'):
        send_command.last_cmd = cmd

    if send_command.last_cmd == cmd and not "short" in cmd:
        return
    if abs(time.time()-send_command.last_cmd_time) <= COMMAND_COOLDOWN:
        return
    url = f"http://{CAR_IP}/{cmd}"
    try:
        response = requests.get(url, timeout=0.5)
        if response.status_code == 200:
            print(f"Sent command: {cmd}")
            send_command.last_cmd_time = time.time()
        else:
            print(f"Failed to send command: {cmd}: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending {cmd}: {e}")


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
    marker_ids = [1, 2, 3, 4, 5]

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

def wifi_listener_enqueue(pose_queue, port, stop_event):
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    print(f"[UDP] Listening on port {port}")

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(512)
            # Unpack header + pose + timestamp + aruco_list
            if data[0:3] == b'\xFA\xF3\x20':
                x, y, z , timestamp = struct.unpack('<3fQ', data[3:23])

                # Get ArUco list
                aruco_list = list(data[23:])  # remaining bytes are ArUco IDs

                # Push to queue
                pose_queue.put((x, y, z, timestamp, aruco_list))

        except Exception as e:
            print(f"[UDP Listener] Error: {e}")

def resource_path(relative_path):
    import sys, os
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.abspath(relative_path)

def kalman_filter_config():
    # === Kalman Filter Configuration ===
    kalman = cv2.KalmanFilter(6, 3)  # 6 state variables (pos + velocity), 3 measurements (pos only)
    # Transition matrix (state update: x = Ax + Bu + w)
    kalman.transitionMatrix = np.array([
        [1, 0, 0, 1, 0, 0],  # x
        [0, 1, 0, 0, 1, 0],  # y
        [0, 0, 1, 0, 0, 1],  # z
        [0, 0, 0, 1, 0, 0],  # vx
        [0, 0, 0, 0, 1, 0],  # vy
        [0, 0, 0, 0, 0, 1]   # vz
    ], dtype=np.float32)

    #Measurement matrix (we only measure position)
    kalman.measurementMatrix = np.eye(3, 6, dtype=np.float32)
    kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-4
    kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-2
    kalman.errorCovPost = np.eye(6, dtype=np.float32)
    
    # Initial state (0 position, 0 velocity)
    kalman.statePost = np.zeros((6, 1), dtype=np.float32)
    return kalman

def wifi_processor_dequeue(pose_queue, aggregator, flaskServer, stop_event):
    while not stop_event.is_set():
        if not pose_queue.empty():
            x, y, z, timestamp, aruco_list = pose_queue.get()
            aggregator.update_pose((x, y, z))
            pose = aggregator.get_average_pose()
            time_now = time.time_ns() // 1000
            if pose and  0 <= (time_now - timestamp) <= POSE_UPDATE_THRESHOLD:
                x, y, z = pose
                flaskServer.updatePosition(x, y, z)
                with aruco_ids_lock:
                    for marker in aruco_list:
                        combined_aruco_ids[str(marker)] =  (combined_aruco_ids[str(marker)] + 1) % 2

def receive_from_clients(method, aggregator, flaskServer, stop_event):
    if method == 'wifi':
        pose_queue = queue.Queue(maxsize = 5000)
        port = 6002
        threading.Thread(target = wifi_listener_enqueue, args=(pose_queue, port, stop_event), daemon=True).start()
        threading.Thread(target = wifi_processor_dequeue, args=(pose_queue, aggregator, flaskServer, stop_event), daemon=True).start()
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
    for key in detector.known_markers.keys():
        combined_aruco_ids[key] = 0
    known_markers = detector.known_markers
    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json", detector=detector)
    aggregator = PoseAggregator()
    stop_event = threading.Event()
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    threading.Thread(target=plot_updater_thread, args = (aggregator, stop_event, flaskServer), daemon=True).start()
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
    
    # for navigating:
    LOOP_DELAY = 0.2 # seconds 
    REACHED_THRESHOLD = 0.2 # meters
    previous_pos = None
    robot_heading = 0.0

    # Main thread displays
    
    # === Kalman Filter Configuration ===
    kalman = kalman_filter_config()
    filtered_pos = 0
    
    # Main thread displays
    while True:


            ret, frame = video.read()
            if not ret:
                break
            
            corners, twoDArray, threeDArray, threeDCenters, frame, aruco_markers_detected = detector.aruco_detect(frame=frame)
                        
            if twoDArray is not None and threeDArray is not None:
                if twoDArray.shape[0] < 3:
                    if len(twoDArray) == 2 and len(threeDArray) == 2:
                        img_pts = twoDArray.reshape(-1, 2).astype(np.float32)
                        obj_pts = threeDArray.reshape(-1, 3).astype(np.float32)
                        success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                        if success:
                            R, _ = cv2.Rodrigues(rvec)
                            position = -R.T @ tvec
                            measured = np.array(position, dtype=np.float32).reshape(3, 1)
                            kalman.correct(measured)
                            predicted = kalman.predict()
                            filtered_pos = predicted[:3]
                            
                    elif len(twoDArray) == 1:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, camera.MARKER_LENGTH, camera.camera_matrix, camera.dist_coeffs)
                        rvec = rvecs[0]
                        tvec = tvecs[0]
                        R_marker2cam, _ = cv2.Rodrigues(rvec)
                        t_marker2cam = tvec.reshape(3, 1)
                        R_cam2marker = R_marker2cam.T
                        t_cam2marker = -R_marker2cam.T @ t_marker2cam
                        center_avg = np.mean(threeDCenters, axis=0).reshape(3, 1)
                        t_cam2world = R_cam2marker @ t_cam2marker + center_avg


                        # Compute average position with Kalman Filter
                        measured = t_cam2world.reshape(3, 1).astype(np.float32)
                        kalman.correct(measured)
                        predicted = kalman.predict()
                        filtered_pos = predicted[:3]
                
                elif len(twoDArray) >= 3 and len(threeDArray) >= 3 and len(twoDArray) == len(threeDArray):
                    flags = cv2.SOLVEPNP_ITERATIVE if len(twoDArray) > 3 else cv2.SOLVEPNP_P3P
                    img_pts = twoDArray.reshape(-1, 2).astype(np.float32)
                    obj_pts = threeDArray.reshape(-1, 3).astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs, flags)
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        position = -R.T @ tvec
                        measured = np.array(position, dtype=np.float32).reshape(3, 1)
                        kalman.correct(measured)
                        predicted = kalman.predict()
                        filtered_pos = predicted[:3]
                
                aruco_markers_detected = np.array(aruco_markers_detected).flatten().tolist() if aruco_markers_detected is not None else []
                if not aruco_markers_detected:
                    with aruco_ids_lock:
                        for marker in aruco_markers_detected:
                            combined_aruco_ids[marker] =  (combined_aruco_ids[marker] + 1) % 2
                        
                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
                aggregator.update_pose((filtered_pos[0][0],filtered_pos[1][0],filtered_pos[2][0]))
                pose = aggregator.get_average_pose()
                # Camera facing forward = Z-axis; extract forward direction
                forward_vector = R.T @ np.array([0, 0, 1])  # Z-axis in world coordinates
                robot_heading = math.atan2(forward_vector[1], forward_vector[0])  # Y, X
                if pose is not None:
                    x, y, z = pose
                    # Update server with smoothed average
                    flaskServer.updatePosition(x, y, z, robot_heading)
                    #print Average Camera Position
                    print(f"Filtered Camera Position -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")
    
                current_pos = flaskServer.getPos()
                target_pos = flaskServer.get_target()
                if target_pos is not None:
                    print(f"Target position: {target_pos}")

                    dx = target_pos['x'] - current_pos['x']
                    dy = target_pos['y'] - current_pos['y']
                    distance = math.hypot(dx, dy)




                            
                    if distance < REACHED_THRESHOLD:
                        send_command("stop")
                        flaskServer.target_position = None
                        print("=== Reached Target ===")
                    else:
                        angle_to_target = math.atan2(dy, dx)
                        heading_error = angle_to_target - robot_heading
                        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error)) # Normalize to get the shortets rotation 
                        heading_error = math.degrees(heading_error)

                        # Simple steering logic
                        if abs(heading_error) < 5:
                            send_command("forward")
                        elif heading_error > 5:
                            send_command("leftShort")
                        elif heading_error < -5:
                            send_command("rightShort")


                    previous_pos = current_pos

                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
            
            else:
                # print("[ERROR] twoDArray or threeDArray is None!")
                send_command("stop")
                
            if _gui_available:
                cv2.imshow("Detection", frame)
    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break
            time.sleep(0.02)

    cv2.destroyAllWindows()

    #Now wait for all threads to end
    server_thread.join()
    #averaging_thread.join()

if __name__ == "__main__":
    main()
