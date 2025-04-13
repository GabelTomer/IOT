import cv2 as cv
import numpy as np
import sys 
import imutils as im 
class detection:
    def __init__(self, image_path=None, video_path=None, type="image"):
        self.image_path = image_path
        self.video_path = video_path
        self.type = type
        print("[INFO] loading image...")
        self.image = cv.imread(self.image_path)
        self.image = im.resize(self.image, width=600)
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
        }# dictionary of all available aruco markers
        
    def aruco_detect(self,marker_dict="DICT_4X4_50", fromImage=True, videoImage = None):
        # Load the dictionary that was used to generate the markers.
        aruco_dict = cv.aruco.getPredefinedDictionary(self.ARUCO_DICT[marker_dict])
        # Initialize the detector parameters using default values
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
        # Detect the markers in the image
        if fromImage:
            corners, ids, rejectedImgPoints = detector.detectMarkers(self.image)
        else:
            corners, ids, rejectedImgPoints = detector.detectMarkers(videoImage)
        # If at least one marker is detected
        if len(corners) > 0:
            # Flatten the list of ids
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                # Extract the marker corners (which are always a list of 4 Numpy arrays)
                corners = markerCorner[0]
                # Draw a bounding box around the detected marker
                cv.polylines(self.image, [np.int32(corners)], True, (0, 255, 0), 2)
                # Compute and draw the center (x, y)-coordinates for the marker
                cX = int(np.average(corners[:, 0]))
                cY = int(np.average(corners[:, 1]))
                cv.circle(self.image, (cX, cY), 4, (0, 0, 255), -1)
                # Draw the marker ID on the frame
                cv.putText(self.image, str(markerID), (cX - 15, cY - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2) 
            print("[INFO] ArUco marker ID: {}".format(markerID))

	    	# show the output image
        if fromImage:
            cv.imshow("Image", self.image)
            cv.waitKey(0)
        else:
            cv.imshow("Frame", videoImage)
           
        return self.image
    def aruco_detect_video(self, marker_dict="DICT_4X4_50"):
        # Load the dictionary that was used to generate the markers.
        
        # Open the video file or capture device
        video = cv.VideoCapture(0 if self.video_path is None else self.video_path)
        if not video.isOpened():
            print("Error: Could not open video.")
            sys.exit(1)
        while True:
            # Read a frame from the video
            ret, frame = video.read()
            if not ret:
                break
            # Detect the markers in the frame
            self.aruco_detect(marker_dict, False, frame)
            	# if the `q` key was pressed, break from the loop
            key = cv.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        video.release()
        cv.destroyAllWindows()
