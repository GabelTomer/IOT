import cv2
import cv2.aruco as aruco

class Camera:
    def __init__(self, camera_id=0, camera_name="robot", camera_type="onboard", 
                 charuco_rows=7, charuco_cols=5, square_length=0.02, marker_length=0.016, 
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
        self.charuco_board = aruco.CharucoBoard((self.CHARUCO_COLS, self.CHARUCO_ROWS), 
                                                self.SQUARE_LENGTH, self.MARKER_LENGTH, 
                                                self.DICTIONARY)
        self.camera_matrix = None
        self.dist_coeffs = None

    def __repr__(self):
        return f"Camera(id={self.camera_id}, name={self.camera_name}, type={self.camera_type})"

    def calibrate_camera(self):
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print("[ERROR] Camera not found.")
            exit()

        print("Press SPACE to capture a frame. Need {} valid frames.".format(self.NUM_FRAMES))
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

            cv2.putText(frame, f"Captured: {valid_count}/{self.NUM_FRAMES}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Calibration", frame)
            key = cv2.waitKey(1)

            if key == 27:  # ESC
                break
            elif key == 32 and retval > 4:  # SPACE
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                valid_count += 1
                print(f"Frame {valid_count} captured.")
                if image_size is None:
                    image_size = gray.shape[::-1]

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
        print(f"Saved calibration to {self.SAVE_PATH}")