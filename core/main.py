import threading
import struct
import queue
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection
import sys
from communication.pose_aggregator import PoseAggregator
from server.flaskServer import server
import random
import socket
import json
import smbus2
import select
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import RPi.GPIO as GPIO
import os
import matplotlib.patheffects as pe

POSE_UPDATE_THRESHOLD = 40000.0
GENERATE_ARUCO_BOARD = False

SLAVE_CONFIG = {
    0x08: 18,  # Pi2: address 0x08, interrupt pin 12
    0x09: 19,  # Pi3: address 0x09, interrupt pin 35
}

known_markers = Detection.load_known_markers("core/utils/known_markers.json")

plot_lock = threading.Lock()
fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
poses_log = []
plt.ion()

# GPIO setup and interrupt handling for data ready (per slave)
GPIO.setmode(GPIO.BCM)
ready_flags = {}

def make_callback(addr):
    def callback(channel):
        ready_flags[addr].set()
    return callback

def update_pose_visual_and_stats(title, pose, markers = None, color = 'b', marker = 'o'):
    with plot_lock:
        global fig, ax, poses_log
        x, y, z = pose

        # Keep history for statistics, but don't plot it all
        poses_log.append((x, y, z))

        ax.cla()
        ax.set_title(title)
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(0, 2)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        # Plot only the current pose
        ax.scatter([x], [y], [z], c=color, marker=marker)

        # Draw lines from each detected ArUco marker center to the current pose
        if markers is not None:
            for marker in markers:
                cx, cy, cz = known_markers[str(marker)]
                dx, dy, dz = x - cx, y - cy, z - cz
                ax.quiver(cx, cy, cz, dx, dy, dz, color = 'r', arrow_length_ratio = 0.05)

        # Overlay statistics
        try:
            if len(poses_log) > 1:
                df = pd.DataFrame(poses_log, columns=["X", "Y", "Z"])
                mean = df.mean()
                std = df.std()
                stats_text = (
                    f"Mean: ({mean['X']:.2f}, {mean['Y']:.2f}, {mean['Z']:.2f})\n"
                    f"Std:  ({std['X']:.2f}, {std['Y']:.2f}, {std['Z']:.2f})"
                )
                ax.text2D(0.05, 0.95, stats_text, transform=ax.transAxes, fontsize=8,
                        verticalalignment ='top', bbox=dict(boxstyle = "round", fc = "w"),
                        path_effects=[pe.withStroke(linewidth=1, foreground = "black")])
                
                # Save statistics to CSV
                try:
                    if len(poses_log) > 1:
                        df.tail(1).to_csv("pose_statistics_log.csv", mode = 'a', header = not os.path.exists("pose_statistics_log.csv"), index = False)
                
                except Exception as e:
                    print("[Plot Error] Failed to save stats to CSV:", e)
                
        except Exception as e:
            print("[Plot Error] Failed to compute stats overlay:", e)

        plt.pause(0.001)

def runServer(flaskServer : server):
    
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
            if attempts > 5000:
                print("⚠️ Warning: Too many placement attempts. Returning what was placed.")
                break
        return positions

    # Generate safe marker positions
    positions_list = generate_safe_positions(len(marker_ids), marker_size_m, min_spacing_m)
    if len(positions_list) < len(marker_ids):
        raise RuntimeError("❌ Failed to generate safe positions for all markers.")
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

def wifi_listener_enqueue(pose_queue, ports):
    sockets = []
    for port in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', port))
        sock.listen(1)
        sock.setblocking(False)
        sockets.append(sock)
        print(f"[WIFI] Listening on port {port}")

    inputs = sockets.copy()

    while True:
        readable, _, _ = select.select(inputs, [], [])
        for s in readable:
            if s in sockets:
                conn, addr = s.accept()
                conn.setblocking(False)
                inputs.append(conn)
                print(f"[WIFI] Connected: {addr}")
                
            else:
                try:
                    data = s.recv(1024).decode()
                    if not data:
                        inputs.remove(s)
                        s.close()
                        continue
                    pose_data = json.loads(data)

                    x = pose_data.get("x")
                    y = pose_data.get("y")
                    z = pose_data.get("z")
                    timestamp = pose_data.get("timestamp")
                    aruco_list = pose_data.get("aruco_list", [])

                    if None not in (x, y, z, timestamp):
                        pose_queue.put((x, y, z, timestamp, aruco_list))
                        
                except Exception as e:
                    print(f"[WIFI] Error: {e}")
                    inputs.remove(s)
                    s.close()

def resource_path(relative_path):
    import sys, os
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.abspath(relative_path)

def test_communication():
    inputs = sockets.copy()

    while True:
        readable, _, _ = select.select(inputs, [], [])
        for s in readable:
            if s in sockets:
                conn, addr = s.accept()
                conn.setblocking(False)
                inputs.append(conn)
                print(f"[WIFI] Connected: {addr}")
            else:
                try:
                    data = s.recv(1024).decode()
                    if not data:
                        inputs.remove(s)
                        s.close()
                        continue
                    pose_data = json.loads(data)
                    x = pose_data.get("x")
                    y = pose_data.get("y")
                    z = pose_data.get("z")
                    if None not in (x, y, z):
                        pose_queue.put((x, y, z))
                except Exception as e:
                    print(f"[WIFI] Error: {e}")
                    inputs.remove(s)
                    s.close()

def wifi_processor_dequeue(pose_queue, aggregator, flaskServer):
    while True:
        if not pose_queue.empty():
            x, y, z, timestamp, aruco_list = pose_queue.get()
            aggregator.update_pose((x, y, z))
            pose = aggregator.get_average_pose()
            time_now = time.time_ns() // 1000
            if pose and  0 <= (time_now - timestamp) <= POSE_UPDATE_THRESHOLD:
                x, y, z = pose
                flaskServer.updatePosition(x, y, z)
                update_pose_visual_and_stats("3D Pose Estimation", pose, aruco_list, color = 'orange', marker = 'x')

def receive_from_clients(method, aggregator, flaskServer, stop_event):
    if method == 'wifi':
        pose_queue = queue.Queue(maxsize = 5000)
        ports = [6001, 6002]
        threading.Thread(target=wifi_listener_enqueue, args=(pose_queue, ports), daemon=True).start()
        threading.Thread(target=wifi_processor_dequeue, args=(pose_queue, aggregator, flaskServer), daemon=True).start()
    elif method == 'i2c':
        # Map addresses to bus numbers
        bus_map = {0x08: smbus2.SMBus(1), 0x09: smbus2.SMBus(3)}
        addrs = list(SLAVE_CONFIG.keys())
        buses = [bus_map[addr] for addr in addrs]
        threading.Thread(target=i2c_listener, args=(buses, addrs, aggregator, flaskServer, stop_event), daemon=True).start()

def i2c_listener(buses, addresses, aggregator, flaskServer, stop_event):
    last_counters = {addr: 0 for addr in addresses}
    lost_packages = {addr: 0 for addr in addresses}
    packages = {addr: 0 for addr in addresses}
    pose_temp = {}
    wait_for_all_slaves = set()    
    time_diff = 0.0
    while not stop_event.is_set():
        try:
            # Wait for any ready event
            for addr in addresses:
                bus = buses[addresses.index(addr)]
                if addr not in wait_for_all_slaves and GPIO.input(SLAVE_CONFIG[addr]) == GPIO.HIGH:
                    bus.write_byte(addr, 0xA5)
                    wait_for_all_slaves.add(addr)
                    
                if GPIO.input(SLAVE_CONFIG[addr]) == GPIO.HIGH:
                    ready_flags[addr].set()
                    
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
                                timestamp = struct.unpack('<Q', bytes(data[4:12]))[0]
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
                                        if pose:
                                            update_pose_visual_and_stats("3D Pose Estimation",pose, aruco_id_list)
                                        
                    except Exception as e:
                        print(f"[I2C addr {hex(addr)}] Read error: {e}")
                        
        except Exception as e:
            print(f"[I2C Listener] Fatal error: {e}")
                   
def main():
    
    if GENERATE_ARUCO_BOARD:
        try:
            generate_aruco_board()
        
        except RuntimeError as e:
            print(f"[ARUCO BOARD ERROR] {e}")
        
    recalibrate = False
    camera = Camera()

    while not recalibrate:
        mean_error = camera.calibrate_camera()
        if mean_error < 0.5:
            print("Calibration is GOOD ✅")
            recalibrate = True
        
    detector = Detection(known_markers_path="core/utils/known_markers.json")
    
    flaskServer = server(port = 5000)
    stop_event = threading.Event()
    aggregator = PoseAggregator()
    
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    
    server_thread.start()
    
    for addr, pin in SLAVE_CONFIG.items():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        ready_flags[addr] = threading.Event()
        GPIO.add_event_detect(pin, GPIO.RISING, callback=make_callback(addr))
    
    # Choose communication method: 'wifi' or 'i2c'
    communication_method = 'i2c'  # ← change to 'i2c' when needed
    receive_from_clients(communication_method, aggregator, flaskServer, stop_event)
    
    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Error: Could not Open Video")
        sys.exit(1)
    # Main thread displays
    
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
    filtered_pos = 0

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
                        #R_marker2world = np.eye(3)
                        #angle_rad = np.pi / 2
                        #R_marker2world = cv2.Rodrigues(np.array([angle_rad, 0, 0]))[0]
                        #R_marker2world[:, 1] *= -1  # Flip the y-axis
                        R_marker2cam, _ = cv2.Rodrigues(rvec)
                        t_marker2cam = tvec.reshape(3, 1)
                        R_cam2marker = R_marker2cam.T
                        t_cam2marker = -R_marker2cam.T @ t_marker2cam
                        #R_cam2world = R_marker2world @ R_cam2marker
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

                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
                aggregator.update_pose((filtered_pos[0][0],filtered_pos[1][0],filtered_pos[2][0]))
                pose = aggregator.get_average_pose()
                if pose is not None:
                    x, y, z = pose
                    # Update server with smoothed average
                    flaskServer.updatePosition(x, y, z)
                    update_pose_visual_and_stats("3D Pose Estimation",pose, aruco_markers_detected)
                    #print Average Camera Position
                    print(f"Filtered Camera Position -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")
    
            else:
                print("[ERROR] twoDArray or threeDArray is None!")
                
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break

    cv2.destroyAllWindows()

    #Now wait for all threads to end
    server_thread.join()

if __name__ == "__main__":
    main()
