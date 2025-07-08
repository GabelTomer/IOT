## Barcode Localization Project  
  Tal Shamir, Dor Lugasi, Tomer Gabel
  
## Details about the project:
Vision-based navigation system for RC cars, addressing
the need for low-cost indoor localization using ArUco barcodes to pinpoint
the car’s location in real-time. ​

Our system uses multiple Raspberry Pi cameras to map the surroundings
and estimate the robot's real-time position and orientation. ​

Using our Flutter app, users can define room layouts, place markers, and
set navigation goals, enabling the system to autonomously command the
car to its target.​

## Folder description :
IOT/
├── app/                # Flutter mobile app
├── server/             # Flask API server (runs on master Pi)
├── main.py             # Entry point for detection and communication
├── detection.py        # ArUco marker detection and pose logic
├── communication/      # UDP communication interfaces
├── utils/              # Kalman filter, camera tools, helpers
└── calibration/        # Intrinsic camera calibration files


## ESP32 SDK version used in this project: 

## Arduino/ESP32 libraries used in this project:


## Connection diagram:

## Project Poster:
 
This project is part of ICST - The Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion
https://icst.cs.technion.ac.il/
