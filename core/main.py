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
    
    recalibrate = False
    camera = Camera()
    while not recalibrate:
        mean_error = camera.calibrate_camera()
        if mean_error < 0.5:
            print("Calibration is GOOD ✅")
            recalibrate = True
        elif mean_error == 1:
            recalibrate = False
        else:
            answer = input("Calibration failed ❌. Do you want to retry? (Y/N): ").strip().lower()
            if answer == 'y':
                recalibrate = False  # meaning: run calibration again
            else:
                recalibrate = True   # meaning: stop recalibrating and continue
        
    detector = Detection(known_markers_path="/home/admin/ArucoTracker/IOT/IOT-Dor-branch/core/utils/known_markers.json")
    flaskServer = server(port = 5000)
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
            twoDArray, threeDArray, frame = detector.aruco_detect(frame = frame)
            cv2.imshow("Detection", frame)
            if twoDArray is not None:
                print(f"twoDArray:  {twoDArray}\n")
                success, rvec, tvec = cv2.solvePnP(threeDArray[0], twoDArray[0][0], camera.camera_matrix, camera.dist_coeffs)
                if success:
                    R, _ = cv2.Rodrigues(rvec)
                    position = -R.T @ tvec
                    flaskServer.updatePosition(position[0], position[1], position[2])
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break

    cv2.destroyAllWindows()

    # Now wait for all threads to end
    server_thread.join()

if __name__ == "__main__":
    main()
