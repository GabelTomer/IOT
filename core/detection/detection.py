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
    
    def update_known_markers(self, room="1"):
        self.known_markers = self.load_known_markers(self.known_markers_path, room)
    
    
    def load_known_markers(self, path, room="1"):
        with open(path, 'r') as file:
            data = json.load(file)
        
        known_markers = {}
        if room in data:
            data = data[room]
            for  marker, values in data.items():
                if marker == "origin" or marker == "boundry":
                    # Skip origin and boundary markers
                    continue
                if isinstance(values, dict):
                    # Extract x, y, z from dict
                    known_markers[str(marker)] = np.array([values["x"], values["y"], values["z"]], dtype=np.float32)
                elif isinstance(values, list):
                    # Directly use the list
                    known_markers[str(marker)] = np.array(values, dtype=np.float32)
                else:
                    raise ValueError(f"[ERROR] Invalid marker format for marker {marker}: {values}")
        return known_markers

    def aruco_detect(self, frame, marker_dict="DICT_4X4_50"):
        aruco_dict = cv.aruco.getPredefinedDictionary(self.ARUCO_DICT[marker_dict])
        parameters = cv.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
        aruco_markers = []
        foundMarkers = False

        if ids is not None and len(corners) > 0:
            ids = ids.flatten()
            found_2d = []
            found_3d = []
            found_3d_centers = []
            for markerCorner, markerID in zip(corners, ids):
                corner_points = markerCorner[0]
                cX = int(np.average(corner_points[:, 0]))
                cY = int(np.average(corner_points[:, 1]))
                cv.polylines(frame, [np.int32(corner_points)], True, (0, 255, 0), 2)
                cv.putText(frame, str(markerID), (cX - 15, cY - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                if str(markerID) in self.known_markers:
                    aruco_markers.append(markerID)
                    marker_center = self.known_markers[str(markerID)]
                    marker_size = 0.06  # 6 cm marker
                    half = marker_size / 2.0
                    # Define corners in marker coordinate system and shift to global position
                    object_corners = np.array([
                        [-half,  half, 0.0],
                        [ half,  half, 0.0],
                        [ half, -half, 0.0],
                        [-half, -half, 0.0]
                    ], dtype=np.float32) + marker_center
                    found_3d.append(object_corners)
                    found_3d_centers.append(marker_center)
                    found_2d.append(np.int32(corner_points))
                    foundMarkers = True
		

            if foundMarkers:
                # Ensure the arrays are in the correct format
                twoDArray = np.array(found_2d, dtype=np.float32)
                threeDArray = np.array(found_3d, dtype=np.float32)
                centersArray = np.array(found_3d_centers, dtype=np.float32)
                return corners,twoDArray, threeDArray, centersArray, frame, aruco_markers

        # If no markers are found, return None for arrays
        return None,None, None, None, frame, None
