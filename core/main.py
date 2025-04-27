import threading
import queue
import cv2
import time
import numpy as np
from utils import Camera
from detection import Detection

def capture_frames(video_src, frame_queue, stop_event):
    cap = cv2.VideoCapture(video_src)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera.")
        return
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            break
        if not frame_queue.full():
            frame_queue.put(frame)
        time.sleep(0.01)
    cap.release()

def detect_markers(frame_queue, detection_queue, display_queue, detector, stop_event):
    while not stop_event.is_set():
        if not frame_queue.empty():
            frame = frame_queue.get()
            points_2d, points_3d, frame_copy = detector.aruco_detect(frame)
            if frame_copy is None:
                frame_copy = frame
            if points_2d is not None:
                detection_queue.put((points_2d, points_3d))
            if frame_copy is not None:
                display_queue.put(frame_copy)

def solve_pnp(detection_queue, camera, stop_event):
    while not stop_event.is_set():
        if not detection_queue.empty():
            points_2d, points_3d = detection_queue.get()
            if camera.camera_matrix is None:
                continue
            if len(points_2d) >= 4:
                success, rvec, tvec = cv2.solvePnP(points_3d, points_2d, camera.camera_matrix, camera.dist_coeffs)
                if success:
                    R, _ = cv2.Rodrigues(rvec)
                    position = -R.T @ tvec
                    print(f"[INFO] Camera position: {position.flatten()}")

def main():
    frame_queue = queue.Queue(maxsize=10)
    detection_queue = queue.Queue(maxsize=10)
    display_queue = queue.Queue(maxsize=10)

    camera = Camera()
    #camera.calibrate_camera()
    detector = Detection(known_markers_path="/Users/dorlugasi/Desktop/טכניון/אביב 2025/IoT/Project/IOT/core/utils/known_markers.json")
    stop_event = threading.Event()

    capture_thread = threading.Thread(target=capture_frames, args=(0, frame_queue, stop_event))
    detection_thread = threading.Thread(target=detect_markers, args=(frame_queue, detection_queue, display_queue, detector, stop_event))
    pnp_thread = threading.Thread(target=solve_pnp, args=(detection_queue, camera, stop_event))

    capture_thread.start()
    detection_thread.start()
    pnp_thread.start()

    # Main thread displays
    while True:
        if not display_queue.empty():
            frame = display_queue.get()
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()  # <<<<<< Tell all threads to stop
                break

    cv2.destroyAllWindows()

    # Now wait for all threads to end
    capture_thread.join()
    detection_thread.join()
    pnp_thread.join()

if __name__ == "__main__":
    main()