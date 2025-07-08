import cv2 as cv
import numpy as np
import sys
import imutils as im
import json
import os
from scipy.spatial.transform import Rotation as R

class Detection:
    def __init__(self, known_markers_path=None):
        self.image_path = None
        self.video_path = None
        self.type = "video"
        self.image = None
        self.found_markers_in_3d = {}
        self.found_markers_in_2d = {}
        self.known_markers_path = known_markers_path
        self.ARUCO_DICT = {
	        "DICT_4X4_50": cv.aruco.DICT_4X4_50,
	        "DICT_4X4_100": cv.aruco.DICT_4X4_100,
	        "DICT_4X4_250": cv.aruco.DICT_4X4_250,
	        "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
	        "DICT_5X5_50": cv.aruco.DICT_5X5_50,
	        "DICT_5X5_100": cv.aruco.DICT_5X5_100,
	        "DICT_5X5_250": cv.aruco.DICT_5X5_250,
	        "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
	        "DICT_6X6_50": cv.aruco.DICT_6X6_50,
	        "DICT_6X6_100": cv.aruco.DICT_6X6_100,
	        "DICT_6X6_250": cv.aruco.DICT_6X6_250,
	        "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
	        "DICT_7X7_50": cv.aruco.DICT_7X7_50,
	        "DICT_7X7_100": cv.aruco.DICT_7X7_100,
	        "DICT_7X7_250": cv.aruco.DICT_7X7_250,
	        "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
	        "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
	        "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
	        "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
	        "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
	        "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
        }


        self.directions, self.known_markers = self.load_known_markers(known_markers_path) if known_markers_path else {}
    
    def update_known_markers(self, room="1"):
        self.directions, self.known_markers = self.load_known_markers(self.known_markers_path, room)

    
    
    def load_known_markers(self, path, room="1"):
        with open(path, 'r') as file:
            data = json.load(file)
        
        thetas = {}
        known_markers = {}
        directions = {}
        if room in data:
            data = data[room]
            for  marker, values in data.items():
                if marker == "origin" or marker == "boundry" or marker == "width" or marker == "height":
                    # Skip origin and boundary markers
                    continue
                if isinstance(values, dict):
                    # Extract x, y, z from dict
                    known_markers[str(marker)] = np.array([values["x"], values["y"], values["z"]], dtype=np.double)

                    directions[str(marker)] = np.array([values["yaw"], values["pitch"], values["roll"]], dtype=np.double)

                elif isinstance(values, list):
                    # Directly use the list
                    known_markers[str(marker)] = np.array(values, dtype=np.double)
                else:
                    raise ValueError(f"[ERROR] Invalid marker format for marker {marker}: {values}")

        return directions, known_markers


    def aruco_detect(self, frame, marker_dict="DICT_4X4_50"):
        aruco_dict = cv.aruco.getPredefinedDictionary(self.ARUCO_DICT[marker_dict])
        parameters = cv.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        all_corners = []
        all_found_2d = []
        all_centers_3d = []
        corners, ids, _ = cv.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        aruco_markers = []
        foundMarkers = False

        if ids is not None and len(corners) > 0:
            ids = ids.flatten()
            for markerCorner, markerID in zip(corners, ids):
                corner_points = markerCorner[0]
                cX = np.mean(corner_points[:, 0])
                cY = np.mean(corner_points[:, 1])
                center_2d = np.array([cX, cY], dtype=np.double)
                cv.polylines(frame, [np.int32(corner_points)], True, (0, 255, 0), 2)
                cv.putText(frame, f"{markerID}", (int(cX) - 15, int(cY) - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)     
                if str(markerID) in self.known_markers:
                    aruco_markers.append(markerID)
                    marker_center = self.known_markers[str(markerID)]
                    directions = self.directions[str(markerID)]
                    all_corners.append(markerCorner.reshape(-1, 2))
                    all_found_2d.append(markerCorner.reshape(-1, 2))
                    all_centers_3d.append(get_object_points_from_pose( marker_center,directions[0], directions[1], directions[2]))

            return all_corners, all_found_2d, all_centers_3d, frame, aruco_markers
        else:
            return None, None, None, frame, None


# def get_object_points(theta_deg, center_point):
#     theta_rad = np.deg2rad(theta_deg)  # Convert degrees to radians
#     half = 0.0585 / 2  # Half marker size

#     # Local marker corner offsets relative to center (in marker's own XY/Z plane)
#     local_offsets = np.array([
#         [-half, -half],
#         [ half, -half],
#         [ half,  half],
#         [-half,  half]
#     ])

#     # 2D rotation matrix
#     R = np.array([
#         [np.cos(theta_rad), -np.sin(theta_rad)],
#         [np.sin(theta_rad),  np.cos(theta_rad)]
#     ])

#     # Apply rotation to each offset
#     rotated = (R @ local_offsets.T).T  # shape (4, 2)

#     # Insert into full 3D world positions (assume marker lies in a fixed plane)
#     object_points = np.zeros((4, 3), dtype=np.float32)

#     # You can decide which axis is the marker's plane.
#     # Let's assume the marker lies in the XZ plane, with Y as up.
#     for i in range(4):
#         object_points[i] = [
#             center_point[0] + rotated[i, 0],  # X
#             center_point[1],                 # Y stays the same (height)
#             center_point[2] + rotated[i, 1]  # Z
#         ]

#     return object_points

def get_object_points_from_pose(center_point, yaw, pitch, roll, marker_size=0.0585):
    """
    Compute 3D corner positions of a marker given full orientation.
    yaw, pitch, roll in degrees
    """
    # Define marker corners in local marker frame (centered at origin, lying in XY plane)
    half = marker_size / 2
    local_corners = np.array([
        [-half, -half, 0],  # bottom-left
        [ half, -half, 0],  # bottom-right
        [ half,  half, 0],  # top-right
        [-half,  half, 0]   # top-left
    ])

    # Convert Euler angles to rotation matrix
    # You can change the order to match your input convention: here we use ZYX (roll, pitch, yaw)
    rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    R_marker = rot.as_matrix()  # 3x3 rotation matrix

    # Apply rotation to marker corners
    rotated_corners = (R_marker @ local_corners.T).T  # shape (4, 3)

    # Translate to global position
    object_points = rotated_corners + np.array(center_point, dtype=np.float32)

    return object_points  # shape (4, 3)