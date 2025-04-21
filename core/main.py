from detection import Detection
from utils import Camera
import json
import cv2
def main(): #need to change flow of program
    # Initialize the detection class with an image path and type
    detector = Detection(image_path="C:/Users/talsh/Workspaces/Semester 7 worksapce/IOT/IOT/core/utils/example_01.png", type="image")
    known_markers = get_data_from_json("C:/Users/talsh/Workspaces/Semester 7 worksapce/IOT/IOT/core/utils/known_markers.json")
    # Call the aruco_detect method with a specific marker dictionary
    result_frame = detector.aruco_detect(marker_dict="DICT_5X5_100")
    result_video = detector.aruco_detect_video(marker_dict="DICT_5X5_100", known_markers=known_markers)
    
def find_place_in_3d_world(detector : Detection):
    
    camera = Camera()
    camera.calibrate_camera()
    success, rvec, tvec = cv2.solvePnP(detector.found_markers_in_3d, detector.found_markers_in_2d, camera.camera_matrix, camera.dist_coeffs) #change detector.found_markers_in_3d and in_2d to the correct data structre
    R, _ = cv2.Rodrigues(rvec)
    return -R.T @ tvec
    
def get_data_from_json(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
            return data
    except FileNotFoundError:
        print(f"Error: File not found: {file_path}")
        return None
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in file: {file_path}")
        return None
    except Exception as e:
         print(f"An unexpected error occurred: {e}")
         return None

    
if __name__ == "__main__":
    main()
    # Call the main function to run the detection