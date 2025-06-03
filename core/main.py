import threading
import queue
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection
import sys
from server.flaskServer import server
import random
import requests
import math

CAR_IP = "192.168.0.104"

def send_command(cmd):
    url = f"http://{CAR_IP}/{cmd}"
    try:
        response = requests.get(url, timeout=0.5)
        if response.status_code == 200:
            print(f"Sent command: {cmd}")
        else:
            print(f"Failed to send command: {cmd}: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending {cmd}: {e}")


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
    positions_m = {id: pos for id, pos in zip(marker_ids, positions_list)}

    # Center point on canvas
    center_px = (a4_width_px // 2, a4_height_px // 2)

    # Draw all markers
    for marker_id in marker_ids:
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

def main():
    #generate_aruco_board()
    recalibrate = False
    camera = Camera()

    while not recalibrate:
        mean_error = camera.calibrate_camera()
        if mean_error < 0.5:
            print("Calibration is GOOD ✅")
            recalibrate = True
        
    detector = Detection(known_markers_path="core/utils/known_markers.json")

    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json", detector=detector)
    stop_event = threading.Event()
    
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))    
    server_thread.start()

    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Error: Could not Open Video")
        sys.exit(1)
    
    # for navigating:
    LOOP_DELAY = 0.2 # seconds 
    REACHED_THRESHOLD = 0.15 # meters
    previous_pos = None
    robot_heading = 0.0

    # Main thread displays
    while True:

            ret, frame = video.read()
            if not ret:
                break
            
            corners, twoDArray, threeDArray, threeDCenters, frame = detector.aruco_detect(frame=frame)
            
            current_pos = flaskServer.getPos()
            target_pos = flaskServer.get_target()
            
            if twoDArray is not None and threeDArray is not None:
                if twoDArray.shape[0] < 4:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, camera.MARKER_LENGTH, camera.camera_matrix, camera.dist_coeffs)
                    rvec = rvecs[0]
                    tvec = tvecs[0]
                    R_marker2world = np.eye(3)
                #angle_rad = np.pi / 2
                #R_marker2world = cv2.Rodrigues(np.array([angle_rad, 0, 0]))[0]
                #R_marker2world[:, 1] *= -1  # Flip the y-axis
                    R_marker2cam, _ = cv2.Rodrigues(rvec)
                    t_marker2cam = tvec.reshape(3, 1)
                    R_cam2marker = R_marker2cam.T
                    t_cam2marker = -R_marker2cam.T @ t_marker2cam
                    R_cam2world = R_marker2world @ R_cam2marker
                    print(threeDArray.shape)
                    t_cam2world = R_marker2world @ t_cam2marker + threeDCenters.reshape(3, 1)
                    print(f"Camera Position: {t_cam2world}")
                #print(f"Camera Position: {t_cam2world.flatten()}")
                    flaskServer.updatePosition(t_cam2world[0][0], t_cam2world[1][0], t_cam2world[2][0])
                elif len(twoDArray) >= 4 and len(threeDArray) >= 4 and len(twoDArray) == len(threeDArray):
                    img_pts = twoDArray.reshape(-1, 2).astype(np.float32)
                    obj_pts = threeDArray.reshape(-1, 3).astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera.camera_matrix, camera.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        position = -R.T @ tvec
                        flaskServer.updatePosition(position[0][0], position[1][0], position[2][0])
                        print(f"Position -> X: {position[0][0]:.2f}, Y: {position[1][0]:.2f}, Z: {position[2][0]:.2f}")
                    else:
                        print(f"[ERROR] Mismatch or not enough points: {len(twoDArray)} 2D points, {len(threeDArray)} 3D points")
                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
                current_pos = flaskServer.getPos()
                target_pos = flaskServer.get_target()

                dx = target_pos['x'] - current_pos['x']
                dy = target_pos['y'] - current_pos['y']
                distance = math.hypot(dx, dy)

                if previous_pos:
                    delta_x = current_pos['x'] - previous_pos['x']
                    delta_y = current_pos['y'] - previous_pos['y']
                    if delta_x != 0 or delta_y != 0:
                        robot_heading = math.atan2(delta_y, delta_x)

                if distance < REACHED_THRESHOLD:
                    send_command("stop")
                else:
                    angle_to_target = math.atan2(dy, dx)
                    heading_error = angle_to_target - robot_heading
                    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error)) # Normalize

                # Simple steering logic
                if abs(heading_error) < math.radians(15):
                    send_command("forward")
                elif heading_error > 0:
                    send_command("leftShort")
                else:
                    send_command("rightShort")

                previous_pos = current_pos

                cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
            
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
