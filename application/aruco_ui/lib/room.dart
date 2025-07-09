import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'package:logger/logger.dart';
import 'robot.dart';
import 'dart:math';

class RoomCustomizerPage extends StatefulWidget {
  final String ipAddress;
  final String roomName;
  final Logger logger;
  const RoomCustomizerPage({
    super.key,
    required this.ipAddress,
    required this.roomName,
    required this.logger,
  });

  @override
  State<RoomCustomizerPage> createState() => _RoomCustomizerPageState();
}

class _RoomCustomizerPageState extends State<RoomCustomizerPage> {
  final List<Offset> points = [];
  Offset? origin;
  bool settingOrigin = false;
  int? draggingIndex;
  final double dragThreshold = 20; // distance in pixels to "grab" a point
  double? roomWidthMeters;
  double? roomHeightMeters;
  double meterToPixelX = 1.0;
  double meterToPixelY = 1.0;
  Size? canvasSize;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {

      _showDefinitionMethodDialog();
    });
  }

  void _showDefinitionMethodDialog() {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder:
          (context) => AlertDialog(
            title: Text('How would you like to define the room?'),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.pop(context);
                  _showInitialRoomDimensionDialog(); // Freeform drawing
                },
                child: Text('Draw Room'),
              ),
              TextButton(
                onPressed: () {
                  Navigator.pop(context);
                  _showCoordinateInputDialog(); // Coordinate input
                },
                child: Text('Enter Coordinates'),
              ),
            ],
          ),
    );
  }


  void _showInitialRoomDimensionDialog() {
    final widthController = TextEditingController();
    final heightController = TextEditingController();

    showDialog(
      context: context,
      barrierDismissible: false,
      builder:
          (context) => AlertDialog(
            title: Text('Set Room Dimensions (meters)'),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextField(
                  controller: widthController,
                  keyboardType: TextInputType.numberWithOptions(decimal: true),
                  decoration: InputDecoration(labelText: 'Room Width'),
                ),
                TextField(
                  controller: heightController,
                  keyboardType: TextInputType.numberWithOptions(decimal: true),
                  decoration: InputDecoration(labelText: 'Room Height'),
                ),
              ],
            ),
            actions: [
              TextButton(
                onPressed: () {
                  final width = double.tryParse(widthController.text);
                  final height = double.tryParse(heightController.text);

                  if (width != null &&
                      height != null &&
                      width > 0 &&
                      height > 0) {
                    setState(() {
                      roomWidthMeters = width;
                      roomHeightMeters = height;
                    });
                    Navigator.pop(context);
                  } else {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(content: Text('Enter valid dimensions')),
                    );
                  }
                },
                child: Text('Continue'),
              ),
            ],
          ),
    );
  }


  void _showCoordinateInputDialog() {
  final List<TextEditingController> xControllers = [];
  final List<TextEditingController> yControllers = [];

  // Initialize with 3 default points
  for (int i = 0; i < 3; i++) {
    xControllers.add(TextEditingController());
    yControllers.add(TextEditingController());
  }

  void addPointField() {
    xControllers.add(TextEditingController());
    yControllers.add(TextEditingController());
  }

  showDialog(
    context: context,
    barrierDismissible: false,
    builder: (context) {
      return StatefulBuilder(
        builder: (context, setDialogState) {
          return AlertDialog(
            title: Text('Define Room Coordinates'),
            content: SingleChildScrollView(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    'Enter at least 3 points (meters). First point is treated as the origin.',
                    style: TextStyle(fontSize: 12, color: Colors.grey[700]),
                  ),
                  const SizedBox(height: 8),
                  for (int i = 0; i < xControllers.length; i++)
                    Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: xControllers[i],
                            decoration: InputDecoration(labelText: 'X${i + 1}'),
                            keyboardType: TextInputType.numberWithOptions(decimal: true),
                          ),
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: TextField(
                            controller: yControllers[i],
                            decoration: InputDecoration(labelText: 'Y${i + 1}'),
                            keyboardType: TextInputType.numberWithOptions(decimal: true),
                          ),
                        ),
                        IconButton(
                          icon: Icon(Icons.delete, color: Colors.red),
                          onPressed: () {
                            if (xControllers.length > 3) {
                              setDialogState(() {
                                xControllers.removeAt(i);
                                yControllers.removeAt(i);
                              });
                            }
                          },
                        ),
                      ],
                    ),
                  const SizedBox(height: 10),
                  TextButton.icon(
                    onPressed: () {
                      setDialogState(() {
                        addPointField();
                      });
                    },
                    icon: Icon(Icons.add),
                    label: Text('Add Point'),
                  ),
                ],
              ),
            ),
            actions: [
              TextButton(
                onPressed: () {
                  Navigator.pop(context);
                },
                child: Text('Cancel'),
              ),
              TextButton(
                onPressed: () {
                  try {
                    final coords = <Offset>[];

                    for (int i = 0; i < xControllers.length; i++) {
                      final x = double.tryParse(xControllers[i].text);
                      final y = double.tryParse(yControllers[i].text);
                      if (x == null || y == null) {
                        throw FormatException('Invalid number at point ${i + 1}');
                      }
                      coords.add(Offset(x, y));
                    }

                    if (coords.length < 3) {
                      throw FormatException('At least 3 points are required.');
                    }

                    final minX = coords.map((p) => p.dx).reduce(min);
                    final minY = coords.map((p) => p.dy).reduce(min);
                    final maxX = coords.map((p) => p.dx).reduce(max);
                    final maxY = coords.map((p) => p.dy).reduce(max);

                    final widthMeters = maxX - minX;
                    final heightMeters = maxY - minY;

                    final canvas = canvasSize ?? MediaQuery.of(context).size;
                    final scaleX = canvas.width / widthMeters;
                    final scaleY = canvas.height / heightMeters;

                    setState(() {
                      roomWidthMeters = widthMeters;
                      roomHeightMeters = heightMeters;
                      meterToPixelX = scaleX;
                      meterToPixelY = scaleY;

                      points.clear();
                      for (var coord in coords) {
                        points.add(Offset(
                          (coord.dx - minX) * meterToPixelX,
                          (coord.dy - minY) * meterToPixelY,
                        ));
                      }

                      origin = points.isNotEmpty ? points[0] : null;

                      // Centering
                      final minPx = points.map((p) => p.dx).reduce(min);
                      final maxPx = points.map((p) => p.dx).reduce(max);
                      final minPy = points.map((p) => p.dy).reduce(min);
                      final maxPy = points.map((p) => p.dy).reduce(max);
                      final roomPixelWidth = maxPx - minPx;
                      final roomPixelHeight = maxPy - minPy;

                      final dx = (canvas.width - roomPixelWidth) / 2 - minPx;
                      final dy = (canvas.height - roomPixelHeight) / 2 - minPy;

                      for (int i = 0; i < points.length; i++) {
                        points[i] = points[i].translate(dx, dy);
                      }
                      if (origin != null) {
                        origin = origin!.translate(dx, dy);
                      }
                    });

                    Navigator.pop(context);
                  } catch (e) {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text(e.toString()),
                      ),
                    );
                  }
                },
                child: Text('Apply'),
              ),
            ],
          );
        },
      );
    },
  );
}


  void handleTap(TapUpDetails details) {
    final localPos = details.localPosition;
    setState(() {
      if (settingOrigin) {
        origin = localPos;
        settingOrigin = false;
      } else {
        points.add(localPos);
      }
    });
  }

  void saveRoomData(double width, double height) {
    final roomData = {
      'room': widget.roomName,
      'boundary':
          points.map((p) => {'x': p.dx * 0.8, 'y': p.dy * 0.4}).toList(),
      'origin': {'x': origin!.dx * 0.8, 'y': origin!.dy * 0.4},
      'width': width,
      'height': height,
    };
    widget.logger.i("Saving room data: $roomData");
    sendRoomData(roomData); // ← call Flask when endpoint ready
    Navigator.pop(context);
  }

  Future<void> sendRoomData(Map<String, Object> roomShape) async {
    final url = 'http://${widget.ipAddress}:5000/change_room_shape';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'room': widget.roomName,
        'boundry': roomShape['boundary'],
        'origin': roomShape['origin'],
        'width': roomShape['width'],
        'height': roomShape['height'],
      }),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Room ${widget.roomName} changed successfully");
    } else {
      widget.logger.e("Failed to change room: ${response.statusCode}");
    }
  }

  void handlePanStart(DragStartDetails details) {
    final local = details.localPosition;
    for (int i = 0; i < points.length; i++) {
      if ((points[i] - local).distance < dragThreshold) {
        setState(() {
          draggingIndex = i;
        });
        break;
      }
    }
  }

  void handlePanUpdate(DragUpdateDetails details) {
    if (draggingIndex != null) {
      setState(() {
        points[draggingIndex!] += details.delta;
      });
    }
  }

  void _showRoomDimensionsDialog(BuildContext context) {
    if (points.length < 3 || origin == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Define a shape and set an origin')),
      );
      return;
    }
    saveRoomData(roomHeightMeters!, roomWidthMeters!);
  }

  void _applyPrecisionScaling(
    List<double> pixelLengths,
    List<double> realWorldLengths,
  ) {
    if (canvasSize == null ||
        roomWidthMeters == null ||

        roomHeightMeters == null) {
      return;
    }

    double maxWidth = 0;
    double maxHeight = 0;
    for (int i = 0; i < points.length; i++) {
      for (int j = i; j < points.length; j++) {
        double width = (points[j].dx - points[i].dx).abs();
        double height = (points[j].dy - points[i].dy).abs();
        if (i == j) continue;
        if (width > maxWidth) {
          maxWidth = width;
        }
        if (height > maxHeight) {
          maxHeight = height;
        }
      }
    }
    // Use average pixel-per-meter across both dimensions
    final double pxPerMeterwidth = maxWidth / roomWidthMeters!;
    final double pxPerMeterHeight = maxHeight / roomHeightMeters!;

    setState(() {
      final List<Offset> newPoints = [];
      Offset current = points[0];
      newPoints.add(current);

      for (int i = 0; i < realWorldLengths.length; i++) {
        final Offset p1 = points[i];
        final Offset p2 = points[(i + 1) % points.length];

        final double theta = (p2 - p1).direction;

        final double lengthInPixelsWidth =
            realWorldLengths[i] * pxPerMeterwidth * cos(theta);
        final double lengthInPixelsHeight =
            realWorldLengths[i] * pxPerMeterHeight * sin(theta);
        current = Offset(
          current.dx + lengthInPixelsWidth,
          current.dy + lengthInPixelsHeight,
        );
        newPoints.add(current);
      }

      for (int i = 0; i < points.length; i++) {
        points[i] = newPoints[i];
      }

      if (origin != null) {
        final Offset shift = newPoints[0] - points[0];
        origin = origin! + shift;
      }

      widget.logger.i(
        "Precision rescale complete. width: px/m: $pxPerMeterwidth, height: px/m: $pxPerMeterHeight",
      );
    });
  }

  void _showPrecisionSetupDialog() {
    final List<double> pixelLengths = [];
    final List<TextEditingController> controllers = [];

    for (int i = 0; i < points.length; i++) {
      final p1 = points[i];
      final p2 = points[(i + 1) % points.length];
      final pixelLength = (p2 - p1).distance;
      pixelLengths.add(pixelLength);
      controllers.add(TextEditingController());
    }

    showDialog(
      context: context,
      builder: (context) {
        return AlertDialog(
          title: Text('Improve Precision'),
          content: SizedBox(
            width: double.maxFinite,
            child: ListView.builder(
              shrinkWrap: true,
              itemCount: controllers.length,
              itemBuilder: (context, index) {
                return TextField(
                  controller: controllers[index],
                  keyboardType: TextInputType.numberWithOptions(decimal: true),
                  decoration: InputDecoration(
                    labelText: 'Line ${index + 1} length (m)',
                  ),
                );
              },
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(context),
              child: Text('Cancel'),
            ),
            TextButton(
              onPressed: () {
                final List<double> realWorldLengths = [];
                for (final controller in controllers) {
                  final value = double.tryParse(controller.text);
                  if (value == null || value <= 0) {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text('Enter valid lengths for all lines'),
                      ),
                    );
                    return;
                  }
                  realWorldLengths.add(value);
                }

                _applyPrecisionScaling(pixelLengths, realWorldLengths);
                Navigator.pop(context);
              },
              child: Text('Apply'),
            ),
          ],
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Customize ${widget.roomName}'),
        actions: [
          IconButton(
            icon: Icon(Icons.delete),
            tooltip: 'Clear Drawing',
            onPressed: () {
              setState(() {
                points.clear();
                origin = null;
                draggingIndex = null;
              });
            },
          ),
          IconButton(
            icon: Icon(Icons.my_location),
            tooltip: 'Set Origin',
            onPressed: () => setState(() => settingOrigin = true),
          ),
          IconButton(
            icon: Icon(Icons.settings), // For precision setup (Step 3)
            tooltip: 'Improve Precision',
            onPressed: () {
              if (points.length > 1) {
                _showPrecisionSetupDialog(); // Implement in Step 3
              } else {
                ScaffoldMessenger.of(context).showSnackBar(
                  SnackBar(content: Text('Draw at least two points')),
                );
              }
            },
          ),

          IconButton(
            icon: Icon(Icons.save),
            onPressed: () => _showRoomDimensionsDialog(context),
          ),
        ],
      ),
      body: LayoutBuilder(
        builder: (context, constraints) {
          canvasSize = Size(constraints.maxWidth, constraints.maxHeight);

          if (roomWidthMeters != null && roomHeightMeters != null) {
            meterToPixelX = canvasSize!.width / roomWidthMeters!;
            meterToPixelY = canvasSize!.height / roomHeightMeters!;
          }

          return GestureDetector(
            onTapUp: handleTap,
            onPanStart: handlePanStart,
            onPanUpdate: handlePanUpdate,
            onPanEnd: (_) => setState(() => draggingIndex = null),
            child: CustomPaint(
              painter: RoomPainter(points: points, origin: origin),
              child: Container(
                width: double.infinity,
                height: double.infinity,
                color: Colors.transparent,
              ),
            ),
          );
        },
      ),
    );
  }
}

class RoomPainter extends CustomPainter {
  final List<Offset> points;
  final Offset? origin;

  RoomPainter({required this.points, this.origin});

  List<double> computeInteriorAngles(List<Offset> pts) {
    final angles = <double>[];
    final count = pts.length;

    for (int i = 0; i < count; i++) {
      final prev = pts[(i - 1 + count) % count];
      final curr = pts[i];
      final next = pts[(i + 1) % count];

      final v1 = (prev - curr);
      final v2 = (next - curr);

      final angleRad = (v1.direction - v2.direction).abs();
      final angleDeg =
          (angleRad > 3.14159 ? (2 * 3.14159 - angleRad) : angleRad) *
          180 /
          3.14159;

      angles.add(angleDeg);
    }

    return angles;
  }

  bool isClosedShape(List<Offset> pts) {
    return pts.length > 2;
  }

  @override
  void paint(Canvas canvas, Size size) {
    final paint =
        Paint()
          ..strokeWidth = 2
          ..color = Colors.blue
          ..style = PaintingStyle.stroke;

    final dotPaint = Paint()..color = Colors.red;

    if (points.length > 1) {
      final path = Path()..moveTo(points.first.dx, points.first.dy);
      for (int i = 1; i < points.length; i++) {
        path.lineTo(points[i].dx, points[i].dy);
      }
      path.close();
      canvas.drawPath(path, paint);
    }

    for (var point in points) {
      canvas.drawCircle(point, 4, dotPaint);
    }

    if (origin != null) {
      canvas.drawCircle(origin!, 6, Paint()..color = Colors.green);
      final textPainter = TextPainter(
        text: TextSpan(
          text: 'Origin (0,0)',
          style: TextStyle(color: Colors.green),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      textPainter.paint(canvas, origin! + Offset(8, -12));
    }
    if (isClosedShape(points)) {
      final angles = computeInteriorAngles(points);

      for (int i = 0; i < points.length; i++) {
        final textPainter = TextPainter(
          text: TextSpan(
            text: '${angles[i].toStringAsFixed(1)}°',
            style: TextStyle(color: Colors.black, fontSize: 12),
          ),
          textDirection: TextDirection.ltr,
        );
        textPainter.layout();
        textPainter.paint(canvas, points[i] + Offset(10, -20));
        final namePainter = TextPainter(
          text: TextSpan(
            text: 'line ${i + 1}',
            style: TextStyle(color: Colors.black, fontSize: 12),
          ),
          textDirection: TextDirection.ltr,
        );
        namePainter.layout();
        // Position the name in the middle of the line segment
        final lineStart = points[i];
        final lineEnd = points[(i + 1) % points.length];
        final midPoint = Offset(
          (lineStart.dx + lineEnd.dx) / 2,
          (lineStart.dy + lineEnd.dy) / 2,
        );
        namePainter.paint(canvas, midPoint + Offset(10, -20));
      }
    }
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}

class RoomSelectorPage extends StatefulWidget {
  final String ipAddress;
  final Logger logger;
  const RoomSelectorPage({
    super.key,
    required this.ipAddress,
    required this.logger,
  });
  @override
  State<RoomSelectorPage> createState() => _RoomSelectorPageState();
}

class _RoomSelectorPageState extends State<RoomSelectorPage> {
  List<String> roomNames = [];
  bool editMode = false;
  String? selectedRoom;

  @override
  void initState() {
    super.initState();
    fetchRoomNames();
  }

  @override
  void dispose() {
    roomNames.clear();
    super.dispose();
  }

  Future<void> addRoom(String room) async {
    final url = 'http://${widget.ipAddress}:5000/add_room';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'room': room}),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Room $room added successfully");
    } else {
      widget.logger.e("Failed to add room: ${response.statusCode}");
    }
  }

  Future<void> fetchRoomNames() async {
    final response = await http.get(
      Uri.parse('http://${widget.ipAddress}:5000/get_rooms'),
    );

    if (response.statusCode == 200) {
      List<dynamic> jsonData = json.decode(response.body);
      roomNames = jsonData.cast<String>();
      setState(() {}); // Update UI
    } else {
      throw Exception('Failed to load room names');
    }
  }

  Future<void> deleteRoom(String room) async {
    final url = 'http://${widget.ipAddress}:5000/delete_room';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'room': room}),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Room $room deleted successfully");
    } else {
      widget.logger.e("Failed to delete room: ${response.statusCode}");
    }
  }

  void showAddRoomDialog() {
    String newRoom = '';
    showDialog(
      context: context,
      builder:
          (context) => AlertDialog(
            title: Text('Add Room'),
            content: TextField(
              autofocus: true,
              decoration: InputDecoration(hintText: 'Enter room name'),
              onChanged: (value) => newRoom = value,
            ),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(context),
                child: Text('Cancel'),
              ),
              TextButton(
                onPressed: () {
                  if (newRoom.isNotEmpty) {
                    if (roomNames.contains(newRoom)) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('Room already exists')),
                      );
                      return;
                    }
                    addRoom(newRoom);

                    setState(() {
                      roomNames.add(newRoom);
                    });
                  }
                  Navigator.pop(context);
                  openRoomCustomizer(newRoom);

                },
                child: Text('Add'),
              ),
            ],
          ),
    );
  }

  void toggleEditMode() {
    setState(() {
      editMode = !editMode;
    });
  }

  void deleteRoomFromUI(int index) {
    if (roomNames.isEmpty) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('No rooms to delete')));
      return;
    }
    if (index < 0 || index >= roomNames.length) {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(SnackBar(content: Text('Invalid room index')));
      return;
    }
    deleteRoom(roomNames[index]);
    setState(() {
      if (roomNames[index] == selectedRoom) selectedRoom = null;
      roomNames.removeAt(index);
    });
  }

  void openRoomCustomizer(String roomName) {
    Navigator.push(
      context,
      MaterialPageRoute(
        builder:
            (context) => RoomCustomizerPage(
              ipAddress: widget.ipAddress,
              roomName: roomName,
              logger: widget.logger,
            ),
      ),
    );
  }

  Future<void> notifyRoomSelection() async {
    final url = 'http://${widget.ipAddress}:5000/notify_room_selection';
    final response = await http.post(
      Uri.parse(url),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'room': selectedRoom}),
    );
    if (response.statusCode == 200) {
      widget.logger.i("Selected Room: $selectedRoom");
    } else {
      widget.logger.e("Failed to select room: ${response.statusCode}");
    }
  }

  void connectToRoom(String room) {
    setState(() {
      selectedRoom = room;
    });
    callRobotControl();
  }

  void callRobotControl() {
    if (selectedRoom != null) {
      notifyRoomSelection();
      Navigator.push(
        context,
        MaterialPageRoute(
          builder:
              (context) => RobotControl(
                ipAddress: widget.ipAddress,
                room: selectedRoom!,
                logger: widget.logger,
              ),
        ),
      );
    } else {
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('Please select a room')));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Rooms'),
        actions: [
          IconButton(
            icon: Icon(editMode ? Icons.check : Icons.edit),
            onPressed: toggleEditMode,
          ),
        ],
      ),
      body: ListView.builder(
        itemCount: roomNames.length,
        itemBuilder: (context, index) {
          final room = roomNames[index];
          return Card(
            margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
            elevation: 2,
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
            ),
            child: ListTile(
              title: Text(room),
              trailing:
                  editMode
                      ? Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          IconButton(
                            icon: Icon(Icons.edit),
                            tooltip: "Customize Room",
                            onPressed: () => openRoomCustomizer(room),
                          ),
                          IconButton(
                            icon: Icon(Icons.delete, color: Colors.red),
                            onPressed: () => deleteRoomFromUI(index),
                          ),
                        ],
                      )
                      : null,
              onTap:
                  editMode
                      ? null
                      : () {
                        connectToRoom(room);
                      },
              selected: room == selectedRoom,
              selectedTileColor: Colors.blue.shade100,
            ),
          );
        },
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: showAddRoomDialog,
        tooltip: 'Add Room',
        child: Icon(Icons.add),
      ),
    );
  }
}
