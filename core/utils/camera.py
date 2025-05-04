import cv2
import cv2.aruco as aruco
import numpy as np
import os
import glob

'''This script is for generating data
1. Provide desired path to store images.
2. Press 'c' to capture image and display it.
3. Press any button to continue.
4. Press 'q' to quit.
'''

def generate_data_for_calibrate():

    camera = cv2.VideoCapture(0)
    ret, img = camera.read()

    path = os.path.dirname(os.path.abspath(__file__)) + "/aruco_data/"  # Folder of current script
    os.makedirs(path, exist_ok = True)
    count = 0
    while True:
        name = path + str(count)+".jpg"
        ret, img = camera.read()
        cv2.imshow("img", img)

        if cv2.waitKey(20) & 0xFF == ord('c'):
            cv2.imwrite(name, img)
            cv2.imshow("img", img)
            count += 1
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break

def generate_charuco_board(squaresX: int = 14, squaresY: int = 10, squareLength: float = 0.04, markerLength: float = 0.032):
    # Use a high-capacity ArUco dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    # Create the board
    board = cv2.aruco.CharucoBoard_create(squaresX, squaresY, squareLength, markerLength, dictionary)
    # A3 landscape resolution (150 DPI): 1754x1240 pixels
    img = board.draw((1754, 1240))
    # Save it
    cv2.imwrite("charuco_board.png", img)
    return board

class Camera:
    def __init__(self, camera_id=0, camera_name="robot", camera_type="onboard", 
                 charuco_rows=10, charuco_cols=14, square_length=0.04, marker_length=0.032, 
                 save_path="camera_calib.yaml"):
        
        self.camera_id = camera_id
        self.camera_name = camera_name
        self.camera_type = camera_type
        self.CHARUCO_ROWS = charuco_rows
        self.CHARUCO_COLS = charuco_cols
        self.SQUARE_LENGTH = square_length
        self.MARKER_LENGTH = marker_length
        self.DICTIONARY = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.SAVE_PATH = save_path
        self.charuco_board = generate_charuco_board(self.CHARUCO_COLS, self.CHARUCO_ROWS, self.SQUARE_LENGTH, self.MARKER_LENGTH)
        self.camera_matrix = None
        self.dist_coeffs = None

    def __repr__(self):
        return f"Camera(id={self.camera_id}, name={self.camera_name}, type={self.camera_type})"
         
    def calibrate_camera(self):
        #generate_data_for_calibrate()
        # === CONFIGURATION ===
        images_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aruco_data")
        aruco_dict = self.DICTIONARY
        board = self.charuco_board

        # === GATHER CORNERS ===
        all_corners = []
        all_ids = []
        image_size = None

        for fname in sorted(glob.glob(os.path.join(images_folder, "*.jpg"))):
            img = cv2.imread(fname)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if image_size is None:
                image_size = gray.shape[::-1]

            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
            if len(corners) > 0:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
                if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 4:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)

        if len(all_corners) == 0 or len(all_ids) == 0:
            print("❌ No valid Charuco detections found. Calibration aborted.")
            return 1

        print(f"✅ Used {len(all_corners)} valid frames for calibration")

        # === CALIBRATION ===
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None)

        print("\n📷 Calibration Results:")
        print("Reprojection error:", ret)
        
        if ret >= 0.5:
            return ret
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        # === VALIDATION: Reprojection Errors ===
        count_error = 0
        all_errors = []
        for i in range(len(all_corners)):
            imgpoints2, _ = cv2.projectPoints(board.chessboardCorners[all_ids[i].flatten()],
                                            rvecs[i], tvecs[i],
                                            camera_matrix, dist_coeffs)
            error = cv2.norm(all_corners[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            if error >= 0.3:
                count_error += 1
            all_errors.append(error)
            
        if (count_error / len(all_corners)) * 100 >= 10:
            print("❌ Validation Failed, Recalibrating...")
            return 1
        
        # === SAVE RESULTS ===
        np.savez("charuco_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        print("\n📁 Calibration saved to 'charuco_calibration.npz'")
        return ret
            
    # def calibrate_camera(self):
    #     calibrate_fail = False
    #     cap = cv2.VideoCapture(self.camera_id)
    #     if not cap.isOpened():
    #         print("[ERROR] Camera not found.")
    #         exit()

    #     print("Capture frames. Need {} valid frames.".format(self.NUM_FRAMES))
    #     print("Press ESC to quit.")

    #     all_corners = []
    #     all_ids = []
    #     image_size = None
    #     valid_count = 0

    #     while valid_count < self.NUM_FRAMES:
    #         ret, frame = cap.read()
    #         if not ret:
    #             continue

    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         corners, ids, _ = aruco.detectMarkers(gray, self.DICTIONARY)

    #         if len(corners) > 0:
    #             aruco.drawDetectedMarkers(frame, corners, ids)
    #             retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, self.charuco_board)

    #             if retval > 4:
    #                 aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
    #                 all_corners.append(charuco_corners)
    #                 all_ids.append(charuco_ids)
    #                 valid_count += 1
    #                 print(f"Frame {valid_count} captured automatically.")
    #                 if image_size is None:
    #                     image_size = gray.shape[::-1]

    #         cv2.putText(frame, f"Captured: {valid_count}/{self.NUM_FRAMES}", (10, 30),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #         cv2.imshow("Calibration", frame)
    #         key = cv2.waitKey(1)
    #         if key == 27:  # ESC
    #             break

    #     cap.release()
    #     cv2.destroyAllWindows()

    #     print("Running calibration...")
    #     calibrate_fail, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
    #         charucoCorners=all_corners,
    #         charucoIds=all_ids,
    #         board=self.charuco_board,
    #         imageSize=image_size,
    #         cameraMatrix=None,
    #         distCoeffs=None
    #     )
        
    #     if not calibrate_fail:
    #         print("[WARNING] Calibration failed. Restarting calibration process.")
    #         return 1

    #     print("Calibration done.")
    #     print("Camera matrix:\n", camera_matrix)
    #     print("Distortion coefficients:\n", dist_coeffs)

    #     cv_file = cv2.FileStorage(self.SAVE_PATH, cv2.FILE_STORAGE_WRITE)
    #     cv_file.write("camera_matrix", camera_matrix)
    #     cv_file.write("dist_coeffs", dist_coeffs)
    #     cv_file.release()

    #     self.camera_matrix = camera_matrix
    #     self.dist_coeffs = dist_coeffs
    #     print(f"Saved calibration to {self.SAVE_PATH}")
    #     return self.validate_camera()
        
        
    # def validate_camera(self, num_validation_frames=20):
            
    #     if self.camera_matrix is None or self.dist_coeffs is None:
    #         print("[ERROR] Camera not calibrated yet.")
    #         return

    #     cap = cv2.VideoCapture(self.camera_id)
    #     if not cap.isOpened():
    #         print("[ERROR] Camera not found.")
    #         exit()

    #     print("Starting validation...")
    #     rvec = np.zeros((3, 1))
    #     tvec = np.zeros((3, 1))
    #     all_detected_corners = []
    #     all_projected_corners = []
    #     frame_count = 0

    #     while frame_count < num_validation_frames:
    #         ret, frame = cap.read()
    #         if not ret:
    #             continue

    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         corners, ids, _ = aruco.detectMarkers(gray, self.DICTIONARY)

    #         if len(corners) > 0:
    #             retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
    #                 corners, ids, gray, self.charuco_board)

    #             if retval > 4:
    #                 success = aruco.estimatePoseCharucoBoard(
    #                     charucoCorners=charuco_corners,
    #                     charucoIds=charuco_ids,
    #                     board=self.charuco_board,
    #                     cameraMatrix=self.camera_matrix,
    #                     distCoeffs=self.dist_coeffs,
    #                     rvec=rvec,
    #                     tvec=tvec)
                    
    #                 if success:
    #                     cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

    #                     # Project the 3D points to 2D
    #                     obj_points = np.array([self.charuco_board.chessboardCorners[i[0]] for i in charuco_ids], dtype=np.float32)                        
    #                     img_points, _ = cv2.projectPoints(obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
    #                     all_detected_corners.append(charuco_corners.reshape(-1, 2))
    #                     all_projected_corners.append(img_points.reshape(-1, 2))
                        
    #                     frame_count += 1
    #                     print(f"Validation frame {frame_count}/{num_validation_frames} captured.")

    #         cv2.putText(frame, f"Validating: {frame_count}/{num_validation_frames}", (10, 30),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    #         cv2.imshow("Validation", frame)
    #         key = cv2.waitKey(1)
    #         if key == 27:  # ESC
    #             break

    #     cap.release()
    #     cv2.destroyAllWindows()

    #     # Compute reprojection error
    #     total_error = 0
    #     total_points = 0
    #     for detected, projected in zip(all_detected_corners, all_projected_corners):
    #         error = cv2.norm(detected, projected, cv2.NORM_L2)
    #         total_error += error ** 2
    #         total_points += detected.shape[0]

    #     mean_error = (total_error / total_points) ** 0.5
    #     print("\nValidation done.")
    #     print(f"Average reprojection error: {mean_error:.4f} pixels.")
        
    #     return mean_error
        
