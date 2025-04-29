import cv2

# Initialize the camera
cap = cv2.VideoCapture(0)  

if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    print("Camera opened successfully. Press 'q' to quit.")

# Show camera feed
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow('Camera Test', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
