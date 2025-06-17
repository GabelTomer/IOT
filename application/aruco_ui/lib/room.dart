import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'package:logger/logger.dart';
import 'robot.dart';

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
    final TextEditingController widthController = TextEditingController();
    final TextEditingController heightController = TextEditingController();

    showDialog(
      context: context,
      builder:
          (context) => AlertDialog(
            title: Text('Enter Room Dimensions'),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextField(
                  controller: widthController,
                  keyboardType: TextInputType.numberWithOptions(decimal: true),
                  decoration: InputDecoration(labelText: 'Room Width (meters)'),
                ),
                TextField(
                  controller: heightController,
                  keyboardType: TextInputType.numberWithOptions(decimal: true),
                  decoration: InputDecoration(
                    labelText: 'Room Height (meters)',
                  ),
                ),
              ],
            ),
            actions: [
              TextButton(
                onPressed: () => Navigator.of(context).pop(), // Cancel
                child: Text('Cancel'),
              ),
              TextButton(
                onPressed: () {
                  final double? width = double.tryParse(widthController.text);
                  final double? height = double.tryParse(heightController.text);

                  if (width != null && height != null) {
                    Navigator.of(context).pop(); // Close dialog

                    saveRoomData(width, height);
                  } else {
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(content: Text('Please enter valid numbers')),
                    );
                  }
                },
                child: Text('Save'),
              ),
            ],
          ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Customize ${widget.roomName}'),
        actions: [
          IconButton(
            icon: Icon(Icons.my_location),
            tooltip: 'Set Origin',
            onPressed: () {
              setState(() {
                settingOrigin = true;
              });
            },
          ),
          IconButton(
            icon: Icon(Icons.save),
            onPressed: () => _showRoomDimensionsDialog(context),
          ),
        ],
      ),
      body: GestureDetector(
        onTapUp: handleTap,
        onPanStart: handlePanStart,
        onPanUpdate: handlePanUpdate,
        onPanEnd: (_) => setState(() => draggingIndex = null),
        child: CustomPaint(
          painter: RoomPainter(points: points, origin: origin),
          child: Container(
            width: double.infinity,
            height: double.infinity,
            color: Colors.transparent, // Must be visible to detect gestures
          ),
        ),
      ),
    );
  }
}

class RoomPainter extends CustomPainter {
  final List<Offset> points;
  final Offset? origin;

  RoomPainter({required this.points, this.origin});

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
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}

class RoomSelectorPage extends StatefulWidget {
  final String ipAddress;
  final Logger logger;
  const RoomSelectorPage({super.key, required this.ipAddress, required this.logger});
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