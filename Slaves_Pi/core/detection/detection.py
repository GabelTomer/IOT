import cv2 as cv
import numpy as np
import sys
import imutils as im
import json
import os

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

        self.known_markers = self.load_known_markers(known_markers_path) if known_markers_path else {}
    
    def update_known_markers(self):
        self.known_markers = self.load_known_markers(self.known_markers_path)
    
    def load_known_markers(self, path):
        with open(path, 'r') as file:
            data = json.load(file)
        
        known_markers = {}
        for k, v in data.items():
            if isinstance(v, dict):
                # Extract x, y, z from dict
                known_markers[str(k)] = np.array([v["x"], v["y"], v["z"]], dtype=np.float32)
            elif isinstance(v, list):
                # Directly use the list
                known_markers[str(k)] = np.array(v, dtype=np.float32)
            else:
                raise ValueError(f"[ERROR] Invalid marker format for marker {k}: {v}")
        return known_markers

    def aruco_detect(self, frame):
        parameters = cv.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        all_corners = []
        all_found_2d = []
        all_found_3d = []
        all_centers_3d = []
        all_detected_ids = []

        for name, dict_id in self.ARUCO_DICT.items():
            aruco_dict = cv.aruco.getPredefinedDictionary(dict_id)
            corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is None:
                continue

            ids = ids.flatten()
            for markerCorner, markerID in zip(corners, ids):
                corner_points = markerCorner[0]
                cX = int(np.average(corner_points[:, 0]))
                cY = int(np.average(corner_points[:, 1]))
                cv.polylines(frame, [np.int32(corner_points)], True, (0, 255, 0), 2)
                cv.putText(frame, f"{markerID}", (cX - 15, cY - 15),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                markerID_str = str(markerID)
                if markerID_str in self.known_markers:
                    data = self.known_markers[markerID_str]
                    if isinstance(data, tuple):
                        marker_center, marker_size = data
                    else:
                        marker_center = data
                        marker_size = 0.06  # fallback default

                    half = marker_size / 2.0
                    object_corners = np.array([
                        [-half,  half, 0.0],
                        [ half,  half, 0.0],
                        [ half, -half, 0.0],
                        [-half, -half, 0.0]
                    ], dtype=np.float32) + marker_center

                    all_corners.append(markerCorner)
                    all_found_2d.append(np.int32(corner_points))
                    all_found_3d.append(object_corners)
                    all_centers_3d.append(marker_center)
                    all_detected_ids.append(markerID)

        if all_detected_ids:
            return (
                all_corners,
                np.array(all_found_2d, dtype=np.float32),
                np.array(all_found_3d, dtype=np.float32),
                np.array(all_centers_3d, dtype=np.float32),
                frame,
                all_detected_ids
            )
        else:
            return None, None, None, None, frame, None
