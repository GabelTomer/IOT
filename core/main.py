import threading
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection
import sys
from communication.pose_aggregator import PoseAggregator
from server.flaskServer import server
import random
import asyncio
import aiohttp
import threading

async def compute_average(session, flaskServer, ip, direction):
    MAX_ALLOWED_AGE = 0.2  # seconds, adjust as needed
    try:
        async with session.get(f"http://{ip}:5000/get_position", timeout=2) as response:
            if response.status == 200:
                data = await response.json()
                x = data.get("x", 0)
                y = data.get("y", 0)
                z = data.get("z", 0)
                timeStamp = data.get("timestamp", 0)
                if time.time() - timeStamp < MAX_ALLOWED_AGE:
                    oldData = flaskServer.getPos()
                    if oldData is not None:
                        x = (x + oldData['x']) / 2
                        y = (y + oldData['y']) / 2
                        z = (z + oldData['z']) / 2
                        flaskServer.updatePosition(x, y, z)
    except aiohttp.ClientError as e:
        print(f"Error fetching position from {direction} server: {e}")
    except asyncio.TimeoutError:
        print(f"Timeout when fetching from {direction} server.")

async def averaging_loop(flaskServer):
    ip_left = "192.168.2.2"
    ip_right = "192.168.3.2"

    async with aiohttp.ClientSession() as session:
        while True:
            await asyncio.gather(
                compute_average(session, flaskServer, ip_left, "left"),
                compute_average(session, flaskServer, ip_right, "right")
            )
            await asyncio.sleep(0.1)

def start_averaging_loop_in_thread(flaskServer):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(averaging_loop(flaskServer))
        
            

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


def main():
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
    flaskServer = server(port = 5000)
    aggregator = PoseAggregator()
    stop_event = threading.Event()
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    threading.Thread(target=plot_updater_thread, args = (aggregator, stop_event), daemon=True).start()
    server_thread.start()
    averaging_thread =  threading.Thread(target=start_averaging_loop_in_thread, args=(flaskServer,), daemon=True)
    averaging_thread.start()

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
                
                aruco_markers_detected = np.array(aruco_markers_detected).flatten().tolist() if aruco_markers_detected is not None else []
                with aruco_ids_lock:
                    combined_aruco_ids.update(aruco_markers_detected)
                    
                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
                aggregator.update_pose((filtered_pos[0][0],filtered_pos[1][0],filtered_pos[2][0]))
                pose = aggregator.get_average_pose()
                if pose is not None:
                    x, y, z = pose
                    # Update server with smoothed average
                    flaskServer.updatePosition(x, y, z)
                    #print Average Camera Position
                    print(f"Filtered Camera Position -> X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")
    
                current_pos = flaskServer.getPos()
                target_pos = flaskServer.get_target()
                print(f"Target position: {target_pos}")

                dx = target_pos['x'] - current_pos['x']
                dy = target_pos['y'] - current_pos['y']
                distance = math.hypot(dx, dy)

                if previous_pos:
                    delta_x = current_pos['x'] - previous_pos['x']
                    delta_y = current_pos['y'] - previous_pos['y']
                    if delta_x != 0 or delta_y != 0:
                        R, _ = cv2.Rodrigues(rvec)
                        R_cam2world = - R.T @ tvec
                        forward_vec = R_cam2world @ np.array[0,1,1]
                        robot_heading = math.atan2(forward_vec[1], forward_vec[0]) #y,x

                if distance < REACHED_THRESHOLD:
                    send_command("stop")
                else:
                    angle_to_target = math.atan2(dy, dx)
                    heading_error = angle_to_target - robot_heading
                    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error)) # Normalize to get the shortets rotation 
                    heading_error = math.degrees(heading_error)

                # Simple steering logic
                if distance < REACHED_THRESHOLD:
                    send_command("stop")
                    print("=== Reached Target ===")
                else:
                    if abs(heading_error) < math.degrees(5):
                        send_command("forward")
                    elif heading_error > math.radians(5):
                        send_command("leftShort")
                        # send_command("forward")
                    elif heading_error < -math.radians(5):
                        send_command("rightShort")
                        # send_command("forward")

                previous_pos = current_pos

                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
            
                    print(f"Filtered Camera Position -> X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}")
    
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
    averaging_thread.join()

if __name__ == "__main__":
    main()
