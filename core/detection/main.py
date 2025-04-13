from detection import detection

def main():
    # Initialize the detection class with an image path and type
    detector = detection(image_path="C:/Users/talsh/Downloads/example_01.png", type="image")
    
    # Call the aruco_detect method with a specific marker dictionary
    result_frame = detector.aruco_detect(marker_dict="DICT_5X5_100")
    result_video = detector.aruco_detect_video(marker_dict="DICT_5X5_100")


if __name__ == "__main__":
    main()
    # Call the main function to run the detection