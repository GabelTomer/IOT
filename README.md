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
```text
IOT/
├── app/                # Flutter mobile app
│
├── core/               # Detection and communication
│   ├── server/         # Flask API server
│   ├── communication/  # Communication related code
│   ├── detection/      # Markers detection related code
│   ├── utils/          # Camera tools, helpers
│
├── Car/                # RC Car
│
├── python_ui/          # Python UI
│
├── Unit Tests/         # HW and server tests
│
├── Documantation/      # Project documantation
```

## Hardware Used

| Component              | Quantity |
|------------------------|----------|
| Raspberry Pi Zero 2 W  | 3        |
| JT-Zero V2.0 Camera    | 3        |
| ESP32 DevKit           | 1        |
| RC Car Frame + Motors  | 1        |
| Power Bank             | 3        |

## Library Versions:
- OpenCV: 4.5.5.62
- Numpy: 1.24.2
- Flask: 3.1.0
- PySide: 6.9.0


## Project Poster:
 ![poster](.Documentation/IOT_Poster_Barcode_Localization.pdf)

This project is part of ICST - The Interdisciplinary Center for Smart Technologies, Taub Faculty of Computer Science, Technion
https://icst.cs.technion.ac.il/
