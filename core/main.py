import threading
import queue
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection
import sys
from server.flaskServer import server
def runServer(flaskServer : server):
    
    flaskServer.setup_routes()
    flaskServer.run()
     


def main():
 

    camera = Camera()
    camera.calibrate_camera()
    input("Press Enter to continue...")
    detector = Detection(known_markers_path="core/utils/known_markers.json")
    flaskServer = server(port = 5000, known_markers_path="core/utils/known_markers.json")
    stop_event = threading.Event()
    
    server_thread = threading.Thread(target=runServer, args=(flaskServer,))
    

    server_thread.start()
    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Error: Could not Open Video")
        sys.exit(1)
    # Main thread displays
    while True:
        ret, frame = video.read()
        if not ret:
            break

        corners, twoDArray, threeDArray, frame = detector.aruco_detect(frame=frame)
       

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
                t_cam2world = R_marker2world @ t_cam2marker + threeDArray[0].reshape(3, 1)
                print(f"Camera Position: {t_cam2world}")
                #print(f"Camera Position: {t_cam2world.flatten()}")
                flaskServer.updatePosition(t_cam2world[0][0], t_cam2world[1][0], t_cam2world[2][0])
            else:
                #print(f"twoDArray: {twoDArray}\nthreeDArray: {threeDArray}\n")
                if len(twoDArray) >= 4 and len(threeDArray) >= 4:  # Ensure the number of points match
                    twoDArray = np.array(twoDArray, dtype=np.float32).reshape(-1, 2)
                    threeDArray = np.array(threeDArray, dtype=np.float32).reshape(-1, 3)
                    success, rvec, tvec = cv2.solvePnP(threeDArray, twoDArray, camera.camera_matrix, camera.dist_coeffs)
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        position = -R.T @ tvec
                        print(f"Camera Position: {position}")
                        flaskServer.updatePosition(position[0][0], position[1][0], position[2][0])
                else:
                    print("[ERROR] Mismatch in the number of 2D and 3D points.")
            cv2.drawFrameAxes(frame, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
            cv2.imshow("Detection", frame)
 
        #else:
           # print("[INFO] No markers detected.")
           

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    cv2.destroyAllWindows()

    # Now wait for all threads to end
    server_thread.join()

if __name__ == "__main__":
    main()
