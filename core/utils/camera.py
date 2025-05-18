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

        key = cv2.waitKey(10) & 0xFF
        if key == ord('c'):
            cv2.imwrite(name, img)
            cv2.imshow("img", img)
            count += 1
        elif key == ord('q'):
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
        images_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aruco_data")
        if not os.path.exists(images_folder) and any(fname.endswith(".jpg") for fname in os.listdir(images_folder)):
            generate_data_for_calibrate()
        else:
            print("📂 Existing images detected — skipping calibration.")
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
