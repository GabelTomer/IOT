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
from communication.pose_aggregator import PoseAggregator
from server.flaskServer import server
import random
import socket
import json

import smbus2

import math
import select

POSE_UPDATE_THRESHOLD = 10000.0
GENERATE_ARUCO_BOARD = True

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
            if attempts > 10000:
                print("⚠️ Warning: Too many placement attempts. Proceeding with fewer markers.")
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
    total_x = total_y = total_z = 0.0
    count = 0
    while True:
        if not pose_queue.empty():
            x, y, z = pose_queue.get()
            total_x += x
            total_y += y
            total_z += z
            count += 1
            aggregator.update_pose((total_x, total_y, total_z), count)
            pose = aggregator.get_average_pose()
            if pose:
                x, y, z = pose
                flaskServer.updatePosition(x, y, z)

def receive_from_clients(method, aggregator, flaskServer, stop_event):
    if method == 'wifi':
        pose_queue = queue.Queue()
        ports = [6001, 6002]
        threading.Thread(target=wifi_listener_enqueue, args=(pose_queue, ports), daemon=True).start()
        threading.Thread(target=wifi_processor_dequeue, args=(pose_queue, aggregator, flaskServer), daemon=True).start()
    elif method == 'i2c':
        bus_pi2 = smbus2.SMBus(1)
        bus_pi3 = smbus2.SMBus(3)
        addr_pi2 = 0x08
        addr_pi3 = 0x09
        threading.Thread(target=i2c_listener, args=([bus_pi2, bus_pi3], [addr_pi2, addr_pi3], aggregator, flaskServer, stop_event), daemon=True).start()

def i2c_listener(buses, address, aggregator, flaskServer, stop_event):
    last_counter = 0
    try:
        time_diff = 0.0
        total_x = total_y = total_z = 0.0
        count = 0
        while not stop_event.is_set():
            for bus, addr in zip(buses, address):
                try:
                    data = bus.read_i2c_block_data(addr, 0, 24)
                    if data[0:2] == 0xFAF320:
                        print("[MASTER] Received:", bytes(data).hex(), len(data))
                        counter, x, y, z, timestamp = struct.unpack('<BfffQ', bytes(data[3:]))
                        if (counter + 1 - last_counter) % 256 == 1:
                            print("[MASTER] : No missing message")
                            last_counter = counter
                            
                        print("[MASTER] Timestamp received:", timestamp)
                        print("[MASTER] Current time      :", time.time_ns() // 1000)
                        print("[MASTER] Time diff (μs)    :", (time.time_ns() // 1000) - timestamp)
                        time_now = time.time_ns() // 1000
                        time_diff = time_now - timestamp
                        if not any(math.isnan(v) for v in (x, y, z)) and time_diff <= POSE_UPDATE_THRESHOLD:
                            total_x += x
                            total_y += y
                            total_z += z
                            count += 1
                        
                        if count > 0:
                            aggregator.update_pose((total_x, total_y, total_z), count)
                            pose = aggregator.get_average_pose()
                            if pose:
                                x, y, z = pose
                                flaskServer.updatePosition(x, y, z)
                                print(f"Filtered Camera Position -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")
                        
                    time.sleep(0.0035)
                         
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
    camera = Camera(gui_available=_gui_available)

    while not recalibrate:
        mean_error = camera.calibrate_camera()
        if mean_error < 0.5:
            print("Calibration is GOOD ✅")
            recalibrate = True
        else:
            print("Retrying calibration. Got error:", mean_error)
        
    detector = Detection(known_markers_path="core/utils/known_markers.json")

    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json", detector=detector)
    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json", detector=detector)
    stop_event = threading.Event()

    aggregator = PoseAggregator()

    
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    threading.Thread(target=plot_updater_thread, args = (aggregator, stop_event), daemon=True).start()
    server_thread.start()
    
    # Choose communication method: 'wifi' or 'i2c'

    communication_method = 'i2c'  # ← change to 'i2c' when needed
    receive_from_clients(communication_method, aggregator, flaskServer, stop_event)

    
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
            
            corners, twoDArray, threeDArray, threeDCenters, frame = detector.aruco_detect(frame=frame)
            
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
                    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs, flags)
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        position = -R.T @ tvec
                        measured = np.array(position, dtype=np.float32).reshape(3, 1)
                        kalman.correct(measured)
                        predicted = kalman.predict()
                        filtered_pos = predicted[:3]
                
                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
                aggregator.update_pose((filtered_pos[0][0],filtered_pos[1][0],filtered_pos[2][0]),1)
                pose = aggregator.get_average_pose()
                if pose is not None:
                    x,y,z = pose
                    # Update server with smoothed average
                    flaskServer.updatePosition(x, y, z)
                    #print Average Camera Position
                    print(f"Filtered Camera Position -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")
    

                
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break

    cv2.destroyAllWindows()

    #Now wait for all threads to end
    server_thread.join()
    #averaging_thread.join()

if __name__ == "__main__":
    main()
