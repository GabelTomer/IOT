import cv2
import cv2.aruco as aruco
import numpy as np

def generate_charuco_board(squaresX: int, squaresY: int, squareLength: float, markerLength: float):
    
    # Use a 5x5 dictionary (more IDs = more unique markers)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    # Create a Charuco board: 7x10 squares, 3 cm square size, 2.4 cm marker size
    board = cv2.aruco.CharucoBoard((squaresX, squaresY), squareLength, markerLength, dictionary)
    # Draw and save at higher resolution, landscape orientation
    img = board.generateImage((1200, 850))
    cv2.imwrite("charuco_board.png", img)
    return board

class Camera:
    def __init__(self, camera_id=0, camera_name="robot", camera_type="onboard", 
                 charuco_rows=7, charuco_cols=10, square_length=0.03, marker_length=0.024, 
                 num_frames=20, save_path="camera_calib.yaml"):
        
        self.camera_id = camera_id
        self.camera_name = camera_name
        self.camera_type = camera_type
        self.CHARUCO_ROWS = charuco_rows
        self.CHARUCO_COLS = charuco_cols
        self.SQUARE_LENGTH = square_length
        self.MARKER_LENGTH = marker_length
        self.DICTIONARY = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.NUM_FRAMES = num_frames
        self.SAVE_PATH = save_path
        self.charuco_board = generate_charuco_board(self.CHARUCO_COLS, self.CHARUCO_ROWS, self.SQUARE_LENGTH, self.MARKER_LENGTH)
        self.camera_matrix = None
        self.dist_coeffs = None

    def __repr__(self):
        return f"Camera(id={self.camera_id}, name={self.camera_name}, type={self.camera_type})"
            
    def calibrate_camera(self):
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print("[ERROR] Camera not found.")
            exit()

        print("Capture frames. Need {} valid frames.".format(self.NUM_FRAMES))
        print("Press ESC to quit.")

        all_corners = []
        all_ids = []
        image_size = None
        valid_count = 0

        while valid_count < self.NUM_FRAMES:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.DICTIONARY)

            if len(corners) > 0:
                aruco.drawDetectedMarkers(frame, corners, ids)
                retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, self.charuco_board)

                if retval > 4:
                    aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    valid_count += 1
                    print(f"Frame {valid_count} captured automatically.")
                    if image_size is None:
                        image_size = gray.shape[::-1]

            cv2.putText(frame, f"Captured: {valid_count}/{self.NUM_FRAMES}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Calibration", frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break

        cap.release()
        cv2.destroyAllWindows()

        print("Running calibration...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=self.charuco_board,
            imageSize=image_size,
            cameraMatrix=None,
            distCoeffs=None
        )

        print("Calibration done.")
        print("Camera matrix:\n", camera_matrix)
        print("Distortion coefficients:\n", dist_coeffs)

        cv_file = cv2.FileStorage(self.SAVE_PATH, cv2.FILE_STORAGE_WRITE)
        cv_file.write("camera_matrix", camera_matrix)
        cv_file.write("dist_coeffs", dist_coeffs)
        cv_file.release()

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.validate_camera()
        print(f"Saved calibration to {self.SAVE_PATH}")
        
        
    def validate_camera(self, num_validation_frames=20):
            
        if self.camera_matrix is None or self.dist_coeffs is None:
            print("[ERROR] Camera not calibrated yet.")
            return

        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print("[ERROR] Camera not found.")
            exit()

        print("Starting validation...")
        rvec = np.zeros((3, 1))
        tvec = np.zeros((3, 1))
        all_detected_corners = []
        all_projected_corners = []
        frame_count = 0

        while frame_count < num_validation_frames:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.DICTIONARY)

            if len(corners) > 0:
                retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.charuco_board)

                if retval > 4:
                    success = aruco.estimatePoseCharucoBoard(
                        charucoCorners=charuco_corners,
                        charucoIds=charuco_ids,
                        board=self.charuco_board,
                        cameraMatrix=self.camera_matrix,
                        distCoeffs=self.dist_coeffs,
                        rvec=rvec,
                        tvec=tvec)
                    
                    if success:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                        # Project the 3D points to 2D
                        self.charuco_board.getChessboardCorners
                        obj_points = self.charuco_board.getChessboardCorners()[charuco_ids.flatten()]
                        img_points, _ = cv2.projectPoints(obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)

                        all_detected_corners.append(charuco_corners.reshape(-1, 2))
                        all_projected_corners.append(img_points.reshape(-1, 2))
                        
                        frame_count += 1
                        print(f"Validation frame {frame_count}/{num_validation_frames} captured.")

            cv2.putText(frame, f"Validating: {frame_count}/{num_validation_frames}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Validation", frame)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break

        cap.release()
        cv2.destroyAllWindows()

        # Compute reprojection error
        total_error = 0
        total_points = 0
        for detected, projected in zip(all_detected_corners, all_projected_corners):
            error = cv2.norm(detected, projected, cv2.NORM_L2)
            total_error += error ** 2
            total_points += detected.shape[0]

        mean_error = (total_error / total_points) ** 0.5
        print("\nValidation done.")
        print(f"Average reprojection error: {mean_error:.4f} pixels.")

        if mean_error < 0.5:
            print("Calibration is GOOD ✅")
        else:
            print("Calibration is NOT GOOD ❌ — Consider recalibrating.")
        