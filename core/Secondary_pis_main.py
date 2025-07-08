import struct
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection
import sys
from server.flaskServer import server
import threading
# --- General GLOBAL Variables --- 
COMMUNICATION_METHOD = 'wifi'  # ← change to 'WiFi or i2c' when needed
GENERATE_ARUCO_BOARD = False
#POSE_DIFF_THRESHOLD = 0.01  # 1 cm

# --- GLOBAL Variables and Import specific Libraries for WiFi ---
if COMMUNICATION_METHOD == "wifi":
    HEADER = 0xFAF320
    import socket
    HOST = "192.168.0.100"
    PORT = 6002
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- GLOBAL Variables and Import specific Libraries for I2C ---
elif COMMUNICATION_METHOD == "i2c":
    import queue
    import threading
    from communication.i2c_slave_emulated import SimpleI2CSlave
    DATA_READY_GPIO = 17
    MAX_ARUCO_LENGTH = 12
    SLAVE_ADDRESS = 0x08
    HEADER = 0xEB90
    data_queue = queue.Queue(maxsize = 500)
    aruco_detect_queue = queue.Queue(maxsize = 500)
    payload_data = None
    counter = 0
    slave = SimpleI2CSlave(SLAVE_ADDRESS, DATA_READY_GPIO)

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

def runServer(flaskServer: server):
    
    flaskServer.setup_routes()
    flaskServer.run()

# def should_send_pose(current_pose, last_pose):
#     if last_pose is None:
#         return True
    
#     dx = current_pose[0] - last_pose[0]
#     dy = current_pose[1] - last_pose[1]
#     dz = current_pose[2] - last_pose[2]
#     dist_squared = dx**2 + dy**2 + dz**2
#     return dist_squared >= POSE_DIFF_THRESHOLD**2

def decode_bsc_status(status: int):
    return {
        "bytes_copied_to_fifo": (status >> 16) & 0x1F,
        "rx_fifo_bytes":          (status >> 11) & 0x1F,
        "tx_fifo_bytes":          (status >> 6)  & 0x1F,
        "tx_fifo_full":           bool((status >> 2) & 1),
        "tx_fifo_empty":          bool((status >> 5) & 1),
        "tx_busy":                bool((status >> 1) & 1),
        "rx_empty":               bool((status >> 2) & 1),
    }
   
def try_send_data(addr, data, data_len):
    control_abort = (addr << 16) | 0x385
    slave.pi.bsc_xfer(control_abort, b'')
    control_send = (addr << 16) | 0x305
    status, _, _ = slave.pi.bsc_xfer(control_send, data)
    st = decode_bsc_status(status)
    print(f"Queued {st['bytes_copied_to_fifo']} bytes, FIFO now has {st['tx_fifo_bytes']} bytes")
    if st['bytes_copied_to_fifo'] == data_len:
        slave.pi.write(DATA_READY_GPIO, 1)
        return True
    else:
        return False

def clear_data_ready_signal():
    slave.pi.write(DATA_READY_GPIO, 0)
    
def slave_listener(stop_event):
    slave.pi.write(DATA_READY_GPIO, 1)
    global data_queue, aruco_detect_queue
    is_pose_turn = True
    game_start = False
    push_new_data = True  # allow sending new data
    try:
        while not stop_event.is_set():
            status, bytes_read, rx_data = slave.pi.bsc_i2c(slave.address)
            if not game_start and bytes_read > 0 and rx_data[0] == 0xA5:
                clear_data_ready_signal()
                game_start = True
            
            if game_start:
            # Master read acknowledgment
                if bytes_read > 0 and rx_data[0] == 0x00:
                    push_new_data = True  # Master read previous packet
                    clear_data_ready_signal()

                if push_new_data:
                    if is_pose_turn and not data_queue.empty():
                        data = data_queue.queue[0]  # Peek
                    elif not is_pose_turn and not aruco_detect_queue.empty():
                        data = aruco_detect_queue.queue[0]
                    else:
                        continue

                    if try_send_data(slave.address, data, len(data)):
                        if is_pose_turn:
                            data_queue.get()  # Remove after successful send
                        else:
                            aruco_detect_queue.get()
                            
                        is_pose_turn = not is_pose_turn
                        push_new_data = False
                    
    except KeyboardInterrupt:
        slave.close()

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

def send_pose(pose, num_of_aruco, aruco_list):
    global udp_socket, first_time
    x, y, z = pose
    payload = None
    try:
        header_bytes = ((HEADER >> 16) & 0xFF, (HEADER >> 8) & 0xFF, HEADER & 0xFF)
        timestamp = time.time_ns() // 1000
        payload_format = f'<BBB3fQ{num_of_aruco}B'
        payload = struct.pack(payload_format, *header_bytes, x, y, z, timestamp, *aruco_list)
        udp_socket.sendto(payload, (HOST, PORT))
        
    except Exception as e:
        print(f"[UDP ERROR] Failed to send pose: {e}")

def main():
    
    global counter, payload_data, data_queue, aruco_detect_queue, udp_socket, last_sent_pose
    # --- Generate Aruco board for camera calibration ---
    is_gui_available()
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
    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json", detector=detector)
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    server_thread.start()
    # Choose communication method: 'wifi' or 'i2c'
    if COMMUNICATION_METHOD == 'i2c':
        stop_event = threading.Event()
        threading.Thread(target=slave_listener,args=(stop_event,), daemon=True).start()
        
    # --- Open Camera for Video Capturing ---
    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Error: Could not Open Video")
        sys.exit(1)
    
    kalman = kalman_filter_config()
    filtered_pos = None
    
    # Main thread displays
    while True:

            ret, frame = video.read()
            if not ret:
                break
            
            corners, twoDArray, threeDArray, frame, aruco_ids = detector.aruco_detect(frame=frame)
            measured = None
            if twoDArray is not None and threeDArray is not None and twoDArray and threeDArray:
                
                obj_pts = np.vstack(threeDArray)
                img_pts = np.vstack(twoDArray)
                num_points = obj_pts.shape[0]
                if obj_pts.shape[0] != img_pts.shape[0]:
                    print("[WARN] Mismatched 3D and 2D points.")
                    continue

                if num_points >= 4:
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
                # marker_id = aruco_ids[0]  # Use the first detected marker
                # marker_pose = detector.known_markers.get(str(marker_id), {})
                # theta_deg = marker_pose.get("theta", 0)
                # theta_rad = np.radians(theta_deg)

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
				
                aruco_id_list = np.array(aruco_ids).flatten().tolist() if aruco_ids is not None else []
                if COMMUNICATION_METHOD == 'i2c':
                    aruco_id_list = aruco_id_list[:MAX_ARUCO_LENGTH]
    
                num_of_aruco_ids = len(aruco_id_list)
                pose_global = (filtered_pos).flatten()
                if COMMUNICATION_METHOD == "i2c" and (not data_queue.full()) and (not aruco_detect_queue.full()):
                    counter = (counter + 1) % 256
                    pose = np.array(pose_global, dtype = np.float16).view(np.uint16)
                    timestamp = (time.time_ns() // 1000) & 0xFFFFFFFF
                    payload_data = struct.pack('<BBBB3HI', ((HEADER >> 8) & 0xFF), (HEADER & 0xFF), counter, 0x01, pose[0], pose[1], pose[2], timestamp)
                    data_queue.put(payload_data)
                    payload_data = struct.pack(f'<BBBBB{num_of_aruco_ids}B', ((HEADER >> 8) & 0xFF),  (HEADER & 0xFF), counter, 0x02, num_of_aruco_ids, *aruco_id_list)
                    aruco_detect_queue.put(payload_data)
                
                #if should_send_pose(tuple(pose_global), last_sent_pose):
                send_pose(tuple(pose_global), num_of_aruco_ids, aruco_id_list)
                    #last_sent_pose = tuple(pose_global)
                    
                #print Average Camera Position
                print(f"Filtered Camera Position -> X: {pose_global[0]:.4f}, Y: {pose_global[1]:.4f}, Z: {pose_global[2]:.4f}")
    
            else:
                if COMMUNICATION_METHOD == "i2c" and (not data_queue.full()) and (not aruco_detect_queue.full()):
                    counter = (counter + 1) % 256
                    # Create float16 NaNs for x, y, z
                    pose_nan = np.array([np.nan, np.nan, np.nan], dtype=np.float16).view(np.uint16)
                    timestamp = (time.time_ns() // 1000) & 0xFFFFFFFF
                    payload_data = struct.pack('<BBBB3HI', ((HEADER >> 8) & 0xFF), (HEADER & 0xFF), counter, 0x01, pose_nan[0], pose_nan[0], pose_nan[0], timestamp)
                    data_queue.put(payload_data)
                    payload_data = struct.pack('<BBBBB', ((HEADER >> 8) & 0xFF), (HEADER & 0xFF), counter, 0x02, 0)
                    aruco_detect_queue.put(payload_data)
            if _gui_available: 
                cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break

    cv2.destroyAllWindows()
    if COMMUNICATION_METHOD == "i2c":
        slave.close()

if __name__ == "__main__":
    main()
