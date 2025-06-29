import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'package:logger/logger.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'dart:math';

class RobotControl extends StatefulWidget {
  final String ipAddress;
  final String room;
  final Logger logger;
  const RobotControl({super.key, required this.ipAddress, required this.room, required this.logger});

  @override
  State<RobotControl> createState() => _RobotControl();
}

class Marker {
  final String id;
  Offset position;
  double z;
  Marker({required this.id, required this.position, required this.z});
  Map<String, dynamic> toJson() {
    return {'id': id, 'x': position.dx, 'y': position.dy, 'z': z};
  }

  factory Marker.fromJson(Map<String, dynamic> json) {
    return Marker(
      id: json['id'],
      position: Offset(json['x'], json['y']),
      z: json['z'],
    );
  }
}

class _RobotControl extends State<RobotControl> {
  Timer? timer;
  @override
  void initState() {
    super.initState();
    startFetchPosition();
  }

  @override
  void dispose() {
    timer?.cancel();
    knownMarkers.clear();
    super.dispose();
  }

  bool errorOccurred = true;
  String error = "Connection error";
  String robotPosition = "Fetching...";
  double robotX = 0;
  double robotY = 0;
  Offset? targetPositionWorld;
  bool isConnected = false;
  int lastKnownMarkerId = 0;
  final int robotIconSize = 30;
  final int markerIconSize = 20;
  var knownMarkers = <String, Marker>{};
  DateTime _lastSent = DateTime.now();
  bool pressed = false;
  double roomWidth = 0;
  double roomHeight = 0;
  List<Offset> roomBoundary = [];
  Offset roomOrigin = Offset.zero;
  // Adjust based on your robot's IP
  final String robotIP = "192.168.1.104";
  double pixelsPerMeterWidth = 0;
  double pixelsPerMeterHeight = 0;
  double robotHeading = 0; // Robot's heading in radians
  String lastCommand = '';
  double commandTime = 0.0;


  Offset pixelToWorld(Offset pixel, Offset origin) {
    return Offset(
      (pixel.dx - origin.dx) / pixelsPerMeterWidth,
      (origin.dy - pixel.dy) / pixelsPerMeterHeight, // Flip Y-axis
    );
  }

  // Send joystick data, throttle to every 100ms
  void sendJoystickCommand(double x, double y) async {
    final now = DateTime.now();
    if (now.difference(_lastSent).inMilliseconds < 100) return;
    _lastSent = now;

    // Normalize x and y to -100..100
    int normX = (x * 100).toInt();
    int normY = (y * 100).toInt();

    final uri = Uri.parse('http://$robotIP/joystick?x=$normX&y=$normY');

    try {
      await http.get(uri);
    } catch (e) {
      widget.logger.e("Error sending joystick data: $e");
    }
  }

  void startFetchPosition() async {
    await fetchKnownMarkers();
    timer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      await fetchRobotPosition();
      await fetchLastCommand();
    });
  }

  double getRoomWidth(List<Offset> points) {
    final xs = points.map((p) => p.dx);
    final minX = xs.reduce((a, b) => a < b ? a : b);
    final maxX = xs.reduce((a, b) => a > b ? a : b);
    return maxX - minX;
  }

  double getRoomHeight(List<Offset> points) {
    final ys = points.map((p) => p.dy);
    final minY = ys.reduce((a, b) => a < b ? a : b);
    final maxY = ys.reduce((a, b) => a > b ? a : b);
    return maxY - minY;
  }

  Future<void> fetchKnownMarkers() async {
    try {
      final response = await http.get(
        Uri.parse(
          'http://${widget.ipAddress}:5000/get_Known_Markers/${widget.room}',
        ),
      );
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (mounted) {
          setState(() {
            data.forEach((markerId, value) {
              if (int.tryParse(markerId) != null) {
                knownMarkers[markerId] = Marker(
                  id: markerId,
                  position: Offset(
                    (value['x'] as num).toDouble(),
                    (value['y'] as num).toDouble(),
                  ),
                  z: (value['z'] as num).toDouble(),
                );
                if (int.parse(markerId) > lastKnownMarkerId) {
                  lastKnownMarkerId = int.parse(markerId);
                }
              }
            });
            roomOrigin = Offset(
              (data['origin']['x'] as num).toDouble(),
              (data['origin']['y'] as num).toDouble(),
            );
            roomBoundary =
                data['boundry']
                    .map<Offset>((p) => Offset(p['x'], p['y']))
                    .toList();
            double minX = roomBoundary.map((p) => p.dx).reduce(min);
            double maxX = roomBoundary.map((p) => p.dx).reduce(max);
            double minY = roomBoundary.map((p) => p.dy).reduce(min);
            double maxY = roomBoundary.map((p) => p.dy).reduce(max);

            roomWidth = maxX - minX;
            roomHeight = maxY - minY;
            pixelsPerMeterWidth = roomWidth / data['width'];
            pixelsPerMeterHeight = roomHeight / data['height'];

            widget.logger.i("Room boundary: $roomBoundary");
            widget.logger.i("Room origin: $roomOrigin");
            widget.logger.i("Room width: $roomWidth, height: $roomHeight");
            widget.logger.i("Known markers fetched successfully!");
            isConnected = true;
            errorOccurred = false;
          });
        }
      } else {
        if (mounted) {
          setState(() {
            error = "Error: ${response.statusCode}";
            widget.logger.e("Error: ${response.statusCode}");
            isConnected = false;
            errorOccurred = true;
          });
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          isConnected = false;
          error = "Connection error";
          widget.logger.e("Connection error: $e");
          errorOccurred = true;
        });
      }
    }
  }

  Future<void> fetchRobotPosition() async {
    try {
      final response = await http.get(
        Uri.parse('http://${widget.ipAddress}:5000/get_position'),
      );
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (mounted) {
          setState(() {
            robotPosition =
                "X: ${data['x'].toStringAsFixed(2)}, Y: ${data['y'].toStringAsFixed(2)}, Z: ${data['z'].toStringAsFixed(2)}";
            robotX = data['x']; // Scale for display
            robotY = data['z']; // Scale for display
            robotHeading = data['heading'] ?? 0.0; // Robot's heading in radians
            isConnected = true;
            errorOccurred = false;
          });
        }
      } else {
        if (mounted) {
          setState(() {
            error = "Error: ${response.statusCode}";
            widget.logger.e("Error: ${response.statusCode}");
            isConnected = false;
            errorOccurred = true;
          });
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          isConnected = false;
          error = "Connection error";
          widget.logger.e("Connection error: $e");
          errorOccurred = true;
        });
      }
    }
  }

  Future<void> fetchLastCommand() async {
    try {
      final response = await http.get(
        Uri.parse('http://${widget.ipAddress}:5000/get_last_command'),
      );
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (mounted) {
          setState(() {
            lastCommand = data['command'] ?? '';
            commandTime = data['timestamp']; 
          });
        }
      } else {
        if (mounted) {
          setState(() {
            widget.logger.e("Command fetch error: ${response.statusCode}");
          });
        }
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          widget.logger.e("Command fetch failed: $e");
        });
      }
    }
  }

  Future<void> addMarker(String id, Offset pos, double z) async {
    final url = 'http://${widget.ipAddress}:5000/add_marker';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'id': id,
        'x': pos.dx,
        'y': pos.dy,
        'z': z,
        'room': widget.room,
      }),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Marker $id added successfully");
    } else {
      widget.logger.e("Failed to add marker: ${response.statusCode}");
    }
  }

  Future<void> updateMarker(String id, Offset pos, double z) async {
    final url = 'http://${widget.ipAddress}:5000/update_marker';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'id': id,
        'x': pos.dx,
        'y': pos.dy,
        'z': z,
        'room': widget.room,
      }),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Marker $id updated successfully");
    } else {
      widget.logger.e("Failed to update marker: ${response.statusCode}");
    }
  }

  Future<void> deleteMarker(String id) async {
    final url = 'http://${widget.ipAddress}:5000/delete_marker';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'id': id, 'room': widget.room}),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Marker $id deleted");
      knownMarkers.remove(id);
    } else {
      widget.logger.e("Failed to delete marker: ${response.statusCode}");
    }
  }

  @override
  Widget build(BuildContext context) {
    double screenWidth = MediaQuery.of(context).size.width;
    double screenHeight = MediaQuery.of(context).size.height;
    Size mapSize = Size(screenWidth * 0.8, screenHeight * 0.4);
    Offset? lastTapPosition;
    final double maxMarkerId = 256; // Maximum marker ID

    Offset screenToWorld(Offset pos) {
      double worldX = (pos.dx - roomOrigin.dx) / pixelsPerMeterWidth;
      double worldY = (roomOrigin.dy - pos.dy) / pixelsPerMeterHeight; // flip Y back
      return Offset(worldX, worldY);
    }

    void showEditMarkerDialog(
      BuildContext context,
      String markerId,
      Offset pos,
      double z,
    ) {
      final xController = TextEditingController(
        text: pos.dx.toStringAsFixed(2),
      );
      final yController = TextEditingController(
        text: pos.dy.toStringAsFixed(2),
      );
      final zController = TextEditingController(text: z.toStringAsFixed(2));

      showDialog(
        context: context,
        builder:
            (context) => AlertDialog(
              title: Text('Edit Marker $markerId'),
              content: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  TextField(
                    controller: xController,
                    decoration: const InputDecoration(labelText: 'X'),
                  ),
                  TextField(
                    controller: yController,
                    decoration: const InputDecoration(labelText: 'Y'),
                  ),
                  TextField(
                    controller: zController,
                    decoration: const InputDecoration(labelText: 'Z'),
                  ),
                ],
              ),
              actions: [
                TextButton(
                  onPressed: () => Navigator.pop(context),
                  child: const Text('Cancel'),
                ),
                TextButton(
                  onPressed: () async {
                    await deleteMarker(markerId);
                    Navigator.pop(context);
                    fetchKnownMarkers();
                  },
                  child: const Text(
                    'Delete',
                    style: TextStyle(color: Colors.red),
                  ),
                ),
                ElevatedButton(
                  onPressed: () async {
                    final x = double.tryParse(xController.text);
                    final y = double.tryParse(yController.text);
                    final z = double.tryParse(zController.text);
                    if (x != null && y != null && z != null) {
                      await updateMarker(markerId, Offset(x, y), z);
                      if (mounted) {
                        Navigator.pop(context);
                        fetchKnownMarkers();
                      }
                    }
                  },
                  child: const Text('Update'),
                ),
              ],
            ),
      );
    }

    void showAddMarkerDialog(BuildContext context, Offset initialPos) {
      final idController = TextEditingController(
        text: (lastKnownMarkerId + 1).toString(),
      );
      final xController = TextEditingController(
        text: initialPos.dx.toStringAsFixed(2),
      );
      final yController = TextEditingController(
        text: initialPos.dy.toStringAsFixed(2),
      );
      final zController = TextEditingController(text: "0.0");

      showDialog(
        context: context,
        builder:
            (context) => AlertDialog(
              title: const Text('Add Marker'),
              content: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  TextField(
                    controller: idController,
                    decoration: const InputDecoration(labelText: 'Marker ID'),
                  ),
                  TextField(
                    controller: xController,
                    decoration: const InputDecoration(labelText: 'X'),
                  ),
                  TextField(
                    controller: yController,
                    decoration: const InputDecoration(labelText: 'Y'),
                  ),
                  TextField(
                    controller: zController,
                    decoration: const InputDecoration(labelText: 'Z'),
                  ),
                ],
              ),
              actions: [
                TextButton(
                  onPressed: () => Navigator.pop(context),
                  child: const Text('Cancel'),
                ),
                ElevatedButton(
                  onPressed: () async {
                    final id = idController.text.trim();
                    final x = double.tryParse(xController.text);
                    final y = double.tryParse(yController.text);
                    final z = double.tryParse(zController.text);
                    if (id.isNotEmpty && knownMarkers.containsKey(id)) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          content: Text('Marker ID already exists'),
                        ),
                      );
                      return;
                    }

                    if (id.isNotEmpty && double.parse(id) > maxMarkerId) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          content: Text(
                            'Marker ID exceeds maximum value, Invalid ID',
                          ),
                        ),
                      );
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                          content: Text('Marker ID already exists'),
                        ),
                      );
                      return;
                    }
                    if (id.isNotEmpty && x != null && y != null && z != null) {
                      await addMarker(id, Offset(x, y), z);
                      Navigator.pop(context);
                      fetchKnownMarkers(); // Refresh
                    }
                  },
                  child: const Text('Add'),
                ),
              ],
            ),
      );
    }

    bool isPointInsidePolygon(Offset point, List<Offset> polygon) {
      int intersectCount = 0;
      for (int j = 0; j < polygon.length; j++) {
        Offset a = polygon[j];
        Offset b = polygon[(j + 1) % polygon.length];

        if (((a.dy > point.dy) != (b.dy > point.dy)) &&
            (point.dx <
                (b.dx - a.dx) * (point.dy - a.dy) / (b.dy - a.dy + 1e-10) +
                    a.dx)) {
          intersectCount++;
        }
      }
      return (intersectCount % 2 == 1); // Odd = inside
    }

    Future<void> sendTargetPosition(Offset target) async {
      final url =
          'http://${widget.ipAddress}:5000/set_target'; // replace with your endpoint
      final response = await http.post(
        Uri.parse(url),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': target.dx, 'y': target.dy}),
      );
      if (response.statusCode == 200) {
        // Handle success
        widget.logger.i("Target position sent successfully!");
      } else {
        // Handle error
        widget.logger.e('Failed to send target position: ${response.statusCode}');
      }
    }

    void handleMapTap(Offset tapPosition) {
    
      sendTargetPosition(tapPosition);
    }

    return Scaffold(
      body: Stack(
        children: [
          // Back button
          Positioned(
            top: 20,
            left: 16,
            child: IconButton(
              icon: Icon(Icons.arrow_back, size: 28),
              onPressed: () => Navigator.pop(context),
            ),
          ),

          // "Connected"/"Disconnected" aligned below the tip of the arrow
          Positioned(
            top: 58, // adjust this to fine-tune vertical spacing
            left: 40, // this puts the 'C' under the tip of the arrow
            child: Text(
              isConnected ? 'Connected' : 'Disconnected',
              style: TextStyle(
                color: isConnected ? Colors.green : Colors.red,
                fontWeight: FontWeight.bold,
              ),
            ),
          ),
          if (errorOccurred) ...[
            Positioned(
              top: 78, // adjust this to fine-tune vertical spacing
              left: 40, // this puts the 'C' under the tip of the arrow
              child: Text(
                error,
                style: TextStyle(
                  color: Colors.red,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ],
          // Error text or other content below

          // Status Indicator in the top-right corner

          // Main content in the center
          Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              mainAxisAlignment: MainAxisAlignment.start,
              children: [
                Container(
                  width: screenWidth * 0.8,
                  height: screenHeight * 0.4,
                  child: Stack(
                    children: [
                      GestureDetector(
                        onTapUp: (TapUpDetails details) {
                          lastTapPosition = details.localPosition;
                        },
                        onTap: () async {
                          if (lastTapPosition == null) return;

                          // ✅ Check if inside the room boundary
                          if (!isPointInsidePolygon(
                            lastTapPosition!,
                            roomBoundary,
                          )) {
                            ScaffoldMessenger.of(context).showSnackBar(
                              SnackBar(
                                content: Text(
                                  "Tapped outside the room boundary",
                                ),
                              ),
                            );
                            return;
                          }

                          final confirm = await showDialog<bool>(
                            context: context,
                            builder:
                                (context) => AlertDialog(
                                  title: Text('Confirm Action'),
                                  content: Text(
                                    'Send Robot to tapped position?',
                                  ),
                                  actions: [
                                    TextButton(
                                      onPressed:
                                          () => Navigator.pop(context, false),
                                      child: Text('Cancel'),
                                    ),
                                    TextButton(
                                      onPressed:
                                          () => Navigator.pop(context, true),
                                      child: Text('Confirm'),
                                    ),
                                  ],
                                ),
                          );

                          if (confirm == true) {
                            final worldPos = screenToWorld(lastTapPosition!);
                            setState(() {
                              targetPositionWorld = worldPos; 
                            });
                            handleMapTap(worldPos);
                          }
                        },
                        onLongPressStart: (LongPressStartDetails details) {
                          final tapPosition = details.localPosition;

                          // ✅ Also check for long press
                          if (!isPointInsidePolygon(
                            tapPosition,
                            roomBoundary,
                          )) {
                            ScaffoldMessenger.of(context).showSnackBar(
                              SnackBar(
                                content: Text(
                                  "Long press outside the room boundary",
                                ),
                              ),
                            );
                            return;
                          }

                          final worldPos = screenToWorld(tapPosition);
                          showAddMarkerDialog(context, worldPos);
                        },

                        child: Container(
                          color:
                              Colors.transparent, // Required to register taps
                          child: CustomPaint(
                            size: mapSize,
                            painter: RoomPainterMap(
                              boundaryPoints: roomBoundary,
                              origin: roomOrigin,
                              mapWidthMeters: roomWidth,
                              mapHeightMeters: roomHeight,
                              //robotPosition: worldToScreen(
                              //Offset(robotX, robotY),
                              //roomOrigin,
                              //pixelsPerMeterWidth,
                              //pixelsPerMeterHeight,
                            //),
                              //robotHeading: robotHeading,
                             
                            ),
                          ),
                        ),
                      ),
                      for (var marker in knownMarkers.entries)
                        if (isPointInsidePolygon(
                          worldToScreen(
                            marker.value.position,
                            roomOrigin,
                            pixelsPerMeterWidth,
                            pixelsPerMeterHeight,
                          ),
                          roomBoundary,
                        ))
                          Positioned(
                            left:
                                worldToScreen(
                                  marker.value.position,
                                  roomOrigin,
                                  pixelsPerMeterWidth,
                                  pixelsPerMeterHeight,
                                ).dx -
                                10,
                            top:
                                worldToScreen(
                                  marker.value.position,
                                  roomOrigin,
                                  pixelsPerMeterWidth,
                                  pixelsPerMeterHeight,
                                ).dy -
                                markerIconSize,
                            child: GestureDetector(
                              onTap:
                                  () => showEditMarkerDialog(
                                    context,
                                    marker.key,
                                    marker.value.position,
                                    marker.value.z,
                                  ),
                              child: Icon(
                                Icons.qr_code_2,
                                size: markerIconSize.toDouble(),
                                color: Colors.black,
                              ),
                            ),
                          ),

                      // Robot position
                      Positioned(
                        left:
                            worldToScreen(
                              Offset(robotX, robotY),
                              roomOrigin,
                              pixelsPerMeterWidth,
                              pixelsPerMeterHeight,
                            ).dx,
                        top:
                            worldToScreen(
                              Offset(robotX, robotY),
                              roomOrigin,
                              pixelsPerMeterWidth,
                              pixelsPerMeterHeight,
                            ).dy -
                            robotIconSize,
                        child: Transform.rotate(
                          angle: robotHeading,
                          child: Icon(
                          Icons.navigation_rounded,
                          size: robotIconSize.toDouble(),
                          color: Colors.blue,
                          ),
                           // Use the robot's heading
                        ),
                      ),
                      // destination position
                      if (targetPositionWorld != null)
                        Positioned(
                          left: worldToScreen(
                            targetPositionWorld!,
                            roomOrigin,
                            pixelsPerMeterWidth,
                            pixelsPerMeterHeight,
                          ).dx - markerIconSize / 2, // center the icon
                          top: worldToScreen(
                            targetPositionWorld!,
                            roomOrigin,
                            pixelsPerMeterWidth,
                            pixelsPerMeterHeight,
                          ).dy - markerIconSize,
                          child: Icon(
                            Icons.location_on,
                            size: markerIconSize.toDouble(),
                            color: Colors.red,
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 10),
                if (pressed) ...[
                  Text(
                    'Robot Position:',
                    style: Theme.of(context).textTheme.headlineMedium,
                  ),
                  const SizedBox(height: 10),

                  Text(
                    robotPosition,
                    style: Theme.of(context).textTheme.headlineSmall,
                  ),
                  const SizedBox(height: 10),


                  Joystick(
                    mode: JoystickMode.all,
                    listener: (details) {
                      sendJoystickCommand(details.x, details.y);
                    },
                  ),
                ] else ...[
                  Text(
                    'Robot Position:',
                    style: Theme.of(context).textTheme.headlineMedium,
                  ),
                  const SizedBox(height: 10),
                  Text(
                    robotPosition,
                    style: Theme.of(context).textTheme.headlineSmall,
                  ),
                  const SizedBox(height: 10),
                  Text(
                    'Last Command:  $lastCommand', 
                    style: Theme.of(context).textTheme.headlineSmall,
                  ),
                  
                ],
              ],
            ),
          ),

          Positioned(
            top: 50, // push it down from the top
            right: 16, // push it away from the right edge
            child: ElevatedButton(
              style: ElevatedButton.styleFrom(
                elevation: 6, // gives a floating effect
                backgroundColor: Colors.white,
                shadowColor: Colors.black54,
              ),
              child: Text(
                pressed ? "Disable Joystick" : "Enable Joystick",
                style: TextStyle(color: pressed ? Colors.red : Colors.green),
              ),
              onPressed: () {
                setState(() {
                  pressed = !pressed;
                });
              },
            ),
          ),
        ],
      ),
    );
  }
}

Offset worldToScreen(Offset world, Offset origin, double pixelsPerMeterWidth,
    double pixelsPerMeterHeight) {
  return Offset(
    (origin.dx + world.dx * pixelsPerMeterWidth),
    origin.dy - world.dy * pixelsPerMeterHeight, // Flip Y-axis
  );
}

class RoomPainterMap extends CustomPainter {
  final List<Offset> boundaryPoints; // in world meters
  final Offset origin; // in world meters
  final double mapWidthMeters;
  final double mapHeightMeters;
  //final Offset robotPosition;
  //final double robotHeading; // in radians
 // Adjust based on your scal

  RoomPainterMap({
    required this.boundaryPoints,
    required this.origin,
    required this.mapWidthMeters,
    required this.mapHeightMeters,
    //required this.robotPosition,
    //required this.robotHeading,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final paint =
        Paint()
          ..color = Colors.blue[50]!
          ..style = PaintingStyle.fill;

    if (boundaryPoints.length < 3) return;
    //final double robotIconSize = 30.0; // Size of the robot icon
    final path = Path();
    path.moveTo(boundaryPoints[0].dx, boundaryPoints[0].dy);

    for (int i = 1; i < boundaryPoints.length; i++) {
      path.lineTo(boundaryPoints[i].dx, boundaryPoints[i].dy);
    }

    path.close();
    canvas.drawPath(path, paint);

    // Optional: draw grid or border
    final border =
        Paint()
          ..color = Colors.blue[50]!
          ..strokeWidth = 2
          ..style = PaintingStyle.stroke;
    canvas.drawPath(path, border);
  //   final headingColor =
  //       Paint()
  //         ..color = const Color.fromARGB(255, 37, 247, 104)
  //         ..strokeWidth = 2
  //         ..style = PaintingStyle.stroke;
  //   final heading = Path();
  //  final pos = Offset(
  //     robotPosition.dx+robotIconSize/2,
  //     robotPosition.dy-robotIconSize/2,
  //   );
  //   final theta = robotHeading - pi / 2; // Adjust heading to point upwards
  //   heading.moveTo(
  //     pos.dx,
  //     pos.dy,
  //   );
  //   heading.lineTo(
  //     pos.dx + 30 * cos(theta),
  //     pos.dy + 30 * sin(theta),
  //   );
  //   heading.close();
  //   canvas.drawPath(heading, headingColor);
  }

  @override
  bool shouldRepaint(RoomPainterMap oldDelegate) =>
      oldDelegate.boundaryPoints != boundaryPoints ||
      oldDelegate.origin != origin;
}
