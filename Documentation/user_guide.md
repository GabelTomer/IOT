# User Guide: Barcode Localization using ArUco markers

## Overview

This project is a modular, room-scale barcode localization system for an RC car, using multiple Raspberry Pis with cameras, one ESP32 controller, and a dedicated router with static IP configurations. All hardware components automatically connect to the local network, and the entire system can be managed through a flutter app or a python UI.

---

## System Setup

### 1. Network Configuration

- The system uses a **dedicated router** configured for the project.
- All Raspberry Pis and the ESP32 car are pre-configured to connect to this router.
- Each device has a **static IP** assigned in the router’s DHCP settings.
- No internet access is required — the system runs on a **local network only**.

---

## 2. Hardware Startup Sequence

- **Power on all Raspberry Pis** — each has a service that:
  - Captures video from its camera.
  - Detects ArUco markers and estimates the robot’s position relative to the room.
  - Communicates the estimated position to the master Pi via UDP.

- **Power on the ESP32 car**, which:
  - Automatically connects to the Wi-Fi.
  - Listens for UDP commands from the master Pi or the flutter app to move the car.

---

## 3. Control and Calibration

### UI Interface

- From any PC or mobile device connected to the local Wi-Fi:
  1. Run the python UI or the flutter app, connect to the server hosted by the **master Raspberry Pi** (e.g. `http://192.168.0.100:5000`)
  2. Use the interface to:
     - Define rooms and place ArUco markers.
     - View real-time estimated position of the robot.
     - Optional: Via flutter app, set a target location for the robot to navigate to.

### Adding a New Camera Node (Pi)

The system is **modular** — you can add as many Raspberry Pi + Camera units as needed.

To add one:

1. Assign it a **static IP**.
2. Clone the repo.
3. Create a virtual environment, installing all the prequisits in requirements.txt
3. Preform **camera calibration**:
   ```bash
   python3 calibrate_camera.py
4. Update the master Pi’s known IP list
5. Create a service running `Secondary_Pi.py`, reboot the Pi to auto-start the service.

Each Pi will begin sending position data to the master, and the robot’s position will be calculated by averaging the inputs from all available nodes.

## 4. Room Setup

Accurate room setup is critical for the success of localization. ArUco markers define the robot's coordinate frame and position, so incorrect placement or calibration will lead to significant errors in navigation.

### Step 1: Print and Place ArUco Markers

- Print the ArUco markers used in the system using `generate_aruco_board()` method.
- Place each marker **exactly 90 degrees upright from the floor** (i.e., vertical on walls).
  - Even slight tilting may affect the robot's estimated height and localization accuracy.
- Markers should be placed throughout the room.

> If markers are not perfectly vertical, localization may suffer from incorrect height estimation.

### Step 2: Define a Coordinate System

Choose a consistent coordinate system for your room:
- Define the origin (0,0,0), typically at one corner of the room or the center of the robot's starting area.
- Align axes (e.g., X along width, Z along length, Y pointing up).
- Measure and record each marker’s:
  - **Center position** in the room: `(x, y, z)` in centimeters.
  - **Orientation** as **roll, pitch, yaw** angles (in degrees).

Use measuring tools for accuracy.

### Step 3: Insert Room and Markers via UI

- Open the Flutter app or Python UI and connect to the master Pi server.
- Create a new room or select an existing one.
- For each marker:
  - Enter its unique ID (as printed on it).
  - Input its measured position `(x, y, z)` in your room's coordinate system.
  - Input its orientation: roll, pitch, yaw.
- Save the room configuration.

This information is used by each Pi to translate camera coordinates into global room coordinates.


## 5. Controlling the Robot

The master Pi continuously:
1. Collects robot position estimates from all camera nodes.
2. Filters and averages the results.
3. When needed - sends movement commands to the ESP32-controlled car.

### Movements Commands (sent over UDP)
 
`forward` : Drive forward
`left` : Turn left 
`leftShort` : Small left correction 
`right` : Turn right
`rightShort` : Small right correction
`stop` : Halt all movement

## 6. Maintanace and Debugging

- Restaring any Pi will automatically rejoin it to the system.
- use `systemctl status` on each Pi to verify its service is running.
- Camera calibration can be re-run at any time if accuracy drops. 
