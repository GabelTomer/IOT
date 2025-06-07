import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'package:logger/logger.dart';
import 'package:validator_regex/validator_regex.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_typeahead/flutter_typeahead.dart';
import 'dart:math';
final logger = Logger();
void main() {
  logger.i('Aruco Robot UI started');
  runApp(const ArucoApp());
}

class ArucoApp extends StatelessWidget {
  const ArucoApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Aruco Robot UI',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        scaffoldBackgroundColor: const Color.fromARGB(255, 230, 232, 244),
        appBarTheme: AppBarTheme(
          backgroundColor: const Color.fromARGB(
            255,
            230,
            232,
            244,
          ), // Global AppBar color
          foregroundColor: const Color.fromARGB(
            203,
            0,
            0,
            0,
          ), // Global text/icon color
          elevation: 0, // Optional: Flat style
        ),
      ),
      home: const ConnectionPage(),
    );
  }
}

class ConnectionPage extends StatefulWidget {
  const ConnectionPage({super.key});

  @override
  State<ConnectionPage> createState() => _ConnectionPage();
}

class _ConnectionPage extends State<ConnectionPage> {
  final String password = "aruco";
  final TextEditingController ipController = TextEditingController();
  final TextEditingController passwordController = TextEditingController();
  @override
  void ConnectToRobot() {
    String ip = ipController.text;
    String password = passwordController.text;
    if (ip.isNotEmpty && password.isNotEmpty) {
      if (password != this.password) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text('Incorrect Password')));
        return;
      }
      if (Validator.ipAddress(ip) == false) {
        ScaffoldMessenger.of(
          context,
        ).showSnackBar(const SnackBar(content: Text('Invalid IP Address')));
        return;
      }
      logger.i("Connecting to robot at $ip with password $password");
      passwordController.clear();
      Navigator.push(
        context,
        MaterialPageRoute(
          builder: (context) => RoomSelectorPage(ipAddress: ip),
        ),
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Please enter IP and Password')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            TextField(
              controller: ipController,
              decoration: const InputDecoration(
                labelText: 'Robot IP Address',
                border: OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 20),
            TextField(
              controller: passwordController,
              obscureText: true,
              decoration: const InputDecoration(
                labelText: 'Password',
                border: OutlineInputBorder(),
              ),
            ),
            const SizedBox(height: 30),
            ElevatedButton(
              onPressed: ConnectToRobot,
              child: const Text('Connect'),
            ),
          ],
        ),
      ),
    );
  }
}

// class roomControl extends StatefulWidget {
//   final String ipAddress;
//   const roomControl({super.key, required this.ipAddress});

//   @override
//   State<roomControl> createState() => _roomControl();
// }

// class _roomControl extends State<roomControl> {
//   List<String> roomNames = [];
//   String? selectedRoom;

//   @override
//   void initState() {
//     super.initState();
//     fetchRoomNames();
//   }

//   @override
//   void dispose() {
//     roomNames.clear();
//     super.dispose();
//   }

//   Future<void> fetchRoomNames() async {
//     final response = await http.get(
//       Uri.parse('http://${widget.ipAddress}:5000/get_rooms'),
//     );

//     if (response.statusCode == 200) {
//       List<dynamic> jsonData = json.decode(response.body);
//       roomNames = jsonData.cast<String>();
//       setState(() {}); // Update UI
//     } else {
//       throw Exception('Failed to load room names');
//     }
//   }

//   Future<void> notifyRoomSelection() async {
//     final url = 'http://${widget.ipAddress}:5000/notify_room_selection';
//     final response = await http.post(
//       Uri.parse(url),
//       headers: {'Content-Type': 'application/json'},
//       body: jsonEncode({'room': selectedRoom}),
//     );
//     if (response.statusCode == 200) {
//       logger.i("Selected Room: $selectedRoom");
//     } else {
//       logger.e("Failed to select room: ${response.statusCode}");
//     }
//   }

//   Iterable<String> _roomOptionsBuilder(TextEditingValue textEditingValue) {
//     if (textEditingValue.text == '') {
//       return roomNames;
//     }
//     return roomNames.where((String option) {
//       return option.toLowerCase().contains(textEditingValue.text.toLowerCase());
//     });
//   }

//  void showEditRoomDialog(BuildContext context)  {
//     final roomController = TextEditingController();
//     var autoCompleteController;
//     showDialog(
//       context: context,
//       builder:
//           (context) => AlertDialog(
//             title: const Text('Edit Rooms'),
//             content: Column(
//               mainAxisSize: MainAxisSize.min,
//               children: [
//                 // TextField(
//                 //   controller: roomController,
//                 //   decoration: const InputDecoration(labelText: 'Room Name'),
//                 // ),
//                 Autocomplete(
//                   optionsBuilder: _roomOptionsBuilder,
//                   fieldViewBuilder: (
//                     context,
//                     roomController,
//                     focusNode,
//                     onFieldSubmitted,
//                   ) {
//                     autoCompleteController = roomController;
//                     return TextField(
//                       controller: roomController,
//                       focusNode: focusNode,
//                       decoration: const InputDecoration(labelText: 'Room Name'),
//                     );
//                   },
//                   displayStringForOption: (option) => option,
//                   onSelected: (option) {
//                     roomController.text = option;
//                   },
//                 ),
//                 SizedBox(height: 20),
//               ],

//             ),

//             actions: [
//               TextButton(
//                 onPressed: () =>{
//                     FocusScope.of(context).unfocus(),
//                    Navigator.pop(context)
//                    },
//                 child: const Text('Cancel'),
//               ),
//               TextButton(
//                 onPressed: () async {
//                    FocusScope.of(context).unfocus();
//                    Future.microtask(() async {
//                   final roomName = autoCompleteController == null ? roomController.text.trim() : autoCompleteController.text.trim();
//                   if (roomName.isNotEmpty && !roomNames.contains(roomName)) {
//                     ScaffoldMessenger.of(context).showSnackBar(
//                       const SnackBar(content: Text('Room does not exist')),
//                     );
//                     return;
//                   }
//                   if (roomName.isNotEmpty) {
//                     await deleteRoom(roomName);
//                     Navigator.pop(context);
//                     fetchRoomNames();
//                     setState(() {}); // Refresh
//                   }
//                     });
//                 },
//                 child: const Text('delete'),
//               ),
//               TextButton(
//                 onPressed: () async {
//                    FocusScope.of(context).unfocus();
//                    Future.microtask(() async {
//                   final roomName = autoCompleteController == null ? roomController.text.trim() : autoCompleteController.text.trim();
//                   if (roomName.isNotEmpty && roomNames.contains(roomName)) {
//                     ScaffoldMessenger.of(context).showSnackBar(
//                       const SnackBar(content: Text('Room Name already exists')),
//                     );
//                     return;
//                   }
//                   if (roomName.isNotEmpty) {
//                     await addRoom(roomName);
//                     Navigator.pop(context);
//                     fetchRoomNames();
//                     setState(() {}); // Refresh
//                   }
//                    });
//                 },
//                 child: const Text('Add'),
//               ),
//             ],
//           ),
//     );
//   }

//   Future<void> deleteRoom(String room) async {
//     final url = 'http://${widget.ipAddress}:5000/delete_room';
//     final response = await http.post(
//       Uri.parse(url),
//       headers: {'Content-Type': 'application/json'},
//       body: jsonEncode({'room': room}),
//     );
//     if (response.statusCode == 200) {
//       logger.i("Room $room deleted successfully");
//     } else {
//       logger.e("Failed to delete room: ${response.statusCode}");
//     }
//   }

//   Future<void> addRoom(String room) async {
//     final url = 'http://${widget.ipAddress}:5000/add_room';
//     final response = await http.post(
//       Uri.parse(url),
//       headers: {'Content-Type': 'application/json'},
//       body: jsonEncode({'room': room}),
//     );
//     if (response.statusCode == 200) {
//       logger.i("Room $room added successfully");
//     } else {
//       logger.e("Failed to add room: ${response.statusCode}");
//     }
//   }

//   void callRobotControl() {
//     if (selectedRoom != null) {
//       notifyRoomSelection();
//       Navigator.push(
//         context,
//         MaterialPageRoute(
//           builder:
//               (context) => RobotControl(
//                 ipAddress: widget.ipAddress,
//                 room: selectedRoom!,
//               ),
//         ),
//       );
//     } else {
//       ScaffoldMessenger.of(
//         context,
//       ).showSnackBar(const SnackBar(content: Text('Please select a room')));
//     }
//   }

//   @override
//   Widget build(BuildContext context) {
//     return Scaffold(
//       appBar: AppBar(title: const Text('Choose Room')),
//       body: Center(
//         child: Column(
//           mainAxisAlignment: MainAxisAlignment.center,
//           children: [
//             const Text('Choose a room:'),
//             DropdownButton<String>(
//               value: selectedRoom,
//               hint: Text("Select a Room"),
//               items:
//                   roomNames.map((room) {
//                     return DropdownMenuItem(value: room, child: Text(room));
//                   }).toList(),
//               onChanged: (value) {
//                 setState(() {
//                   selectedRoom = value;
//                 });
//               },
//             ),
//             const SizedBox(height: 30),
//             Row(
//               mainAxisAlignment: MainAxisAlignment.center,
//               children: [
//                 const SizedBox(width: 20),
//                 ElevatedButton(
//                   onPressed: () => showEditRoomDialog(context),
//                   child: const Text('Edit Rooms'),
//                 ),
//                 const SizedBox(width: 20),
//                 ElevatedButton(
//                   onPressed: callRobotControl,
//                   child: const Text('Connect'),
//                 ),
//               ],
//             ),
//           ],
//         ),
//       ),
//     );
//   }
// }
class RoomCustomizerPage extends StatefulWidget {
  final String ipAddress;
  final String roomName;

  const RoomCustomizerPage({
    super.key,
    required this.ipAddress,
    required this.roomName,
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

  void saveRoomData() {
    if (points.length < 3 || origin == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Define a shape and set an origin')),
      );
      return;
    }

    final roomData = {
      'room': widget.roomName,
      'boundary': points.map((p) => {'x': p.dx, 'y': p.dy}).toList(),
      'origin': {'x': origin!.dx, 'y': origin!.dy},
    };
    logger.i("Saving room data: $roomData");
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
      }),
    );
    if (response.statusCode == 200) {
      logger.i("Room ${widget.roomName} changed successfully");
    } else {
      logger.e("Failed to change room: ${response.statusCode}");
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
          IconButton(icon: Icon(Icons.save), onPressed: saveRoomData),
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
  const RoomSelectorPage({super.key, required this.ipAddress});
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
      logger.i("Room $room added successfully");
    } else {
      logger.e("Failed to add room: ${response.statusCode}");
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
      logger.i("Room $room deleted successfully");
    } else {
      logger.e("Failed to delete room: ${response.statusCode}");
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
      logger.i("Selected Room: $selectedRoom");
    } else {
      logger.e("Failed to select room: ${response.statusCode}");
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

class RobotControl extends StatefulWidget {
  final String ipAddress;
  final String room;
  const RobotControl({super.key, required this.ipAddress, required this.room});

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
  double pixelsPerMeter = 50.0; // Adjust based on your scale
  Offset pixelToWorld(Offset pixel, Offset origin, double pixelsPerMeter) {
    return Offset(
      (pixel.dx - origin.dx) / pixelsPerMeter,
      (origin.dy - pixel.dy) / pixelsPerMeter, // Flip Y-axis
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
      logger.e("Error sending joystick data: $e");
    }
  }

  void startFetchPosition() async {
    await fetchKnownMarkers();
    timer = Timer.periodic(const Duration(seconds: 1), (timer) async {
      await fetchRobotPosition();
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
                data['boundary']
                    .map((p) => pixelToWorld(p, roomOrigin, pixelsPerMeter))
                    .toList();
            double minX = roomBoundary.map((p) => p.dx).reduce(min);
            double maxX = roomBoundary.map((p) => p.dx).reduce(max);
            double minY = roomBoundary.map((p) => p.dy).reduce(min);
            double maxY = roomBoundary.map((p) => p.dy).reduce(max);

            roomWidth = maxX - minX;
            roomHeight = maxY - minY;
            logger.i("Room boundary: $roomBoundary");
            logger.i("Room origin: $roomOrigin");
            logger.i("Room width: $roomWidth, height: $roomHeight");
            roomOrigin = Offset.zero;
            roomWidth = 0;
            roomHeight = 0;
            logger.i("Known markers fetched successfully!");
            isConnected = true;
            errorOccurred = false;
          });
        }
      } else {
        if (mounted) {
          setState(() {
            error = "Error: ${response.statusCode}";
            logger.e("Error: ${response.statusCode}");
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
          logger.e("Connection error: $e");
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
            robotY = data['y']; // Scale for display
            isConnected = true;
            errorOccurred = false;
          });
        }
      } else {
        if (mounted) {
          setState(() {
            error = "Error: ${response.statusCode}";
            logger.e("Error: ${response.statusCode}");
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
          logger.e("Connection error: $e");
          errorOccurred = true;
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
      logger.i("Marker $id added successfully");
    } else {
      logger.e("Failed to add marker: ${response.statusCode}");
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
      logger.i("Marker $id updated successfully");
    } else {
      logger.e("Failed to update marker: ${response.statusCode}");
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
      logger.i("Marker $id deleted");
      knownMarkers.remove(id);
    } else {
      logger.e("Failed to delete marker: ${response.statusCode}");
    }
  }

  @override
  Widget build(BuildContext context) {
    double screenWidth = MediaQuery.of(context).size.width;
    double screenHeight = MediaQuery.of(context).size.height;
    Size mapSize = Size(screenWidth * 0.8, screenHeight * 0.4);
    final double mapWidthMeters = 10.0; // Width of the map in meters
    final double mapHeightMeters = 10.0; // Height of the map in meters
    double xScaleFactor = (mapSize.width / mapWidthMeters).floorToDouble();
    double yScaleFactor = (mapSize.height / mapHeightMeters).floorToDouble();

    final double maxMarkerId = 256; // Maximum marker ID
    Offset screenToWorld(Offset pos, Size mapSize) {
      const double mapWidthMeters = 10.0;
      const double mapHeightMeters = 10.0;

      double x = (pos.dx / mapSize.width) * mapWidthMeters;
      double y = ((mapSize.height - pos.dy) / mapSize.height) * mapHeightMeters;

      return Offset(x, y);
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

    Future<void> sendTargetPosition(Offset target) async {
      final url =
          'http://${widget.ipAddress}:5000/update_position'; // replace with your endpoint
      final response = await http.post(
        Uri.parse(url),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': target.dx, 'y': target.dy}),
      );
      if (response.statusCode == 200) {
        // Handle success
        logger.i("Target position sent successfully!");
      } else {
        // Handle error
        logger.e('Failed to send target position: ${response.statusCode}');
      }
    }

    void handleMapTap(Offset tapPosition) {
      final worldPos = screenToWorld(tapPosition, mapSize);
      sendTargetPosition(worldPos);
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
                  color: Colors.blue[50],
                  child: Stack(
                    children: [
                      GestureDetector(
                        onTapDown: (TapDownDetails details) async {
                          final tapPosition = details.localPosition;

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
                            handleMapTap(tapPosition);
                          }
                        },
                        onLongPressStart: (LongPressStartDetails details) {
                          final tapPosition = details.localPosition;
                          final worldPos = screenToWorld(tapPosition, mapSize);
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
                              pixelsPerMeter: pixelsPerMeter,
                            ),
                          ),
                        ),
                      ),
                      for (var marker in knownMarkers.entries)
                        Positioned(
                          left:
                              worldToScreen(
                                marker.value.position,
                                roomOrigin,
                                pixelsPerMeter,
                              ).dx -
                              10,
                          top:
                              worldToScreen(
                                marker.value.position,
                                roomOrigin,
                                pixelsPerMeter,
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
                              Icons.location_on,
                              size: markerIconSize.toDouble(),
                              color: Colors.red,
                            ),
                          ),
                        ),

                      // Robot position
                      Positioned(
                        left:
                            worldToScreen(
                              Offset(robotX, robotY),
                              roomOrigin,
                              pixelsPerMeter,
                            ).dx,
                        top:
                            worldToScreen(
                              Offset(robotX, robotY),
                              roomOrigin,
                              pixelsPerMeter,
                            ).dy -
                            robotIconSize,
                        child: Icon(
                          Icons.android,
                          size: robotIconSize.toDouble(),
                          color: Colors.green,
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

Offset worldToScreen(Offset world, Offset origin, double pixelsPerMeter) {
  return Offset(
    origin.dx + world.dx * pixelsPerMeter,
    origin.dy - world.dy * pixelsPerMeter, // Flip Y-axis
  );
}

// class GridPainter extends CustomPainter {
//   final int gridSize = 10;

//   @override
//   void paint(Canvas canvas, Size size) {
//     final paint =
//         Paint()
//           ..color = Colors.grey
//           ..strokeWidth = 1;

//     double stepX = size.width / gridSize;
//     double stepY = size.height / gridSize;

//     // Vertical lines (X axis)
//     for (double x = 0; x <= size.width; x += stepX) {
//       canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
//     }

//     // Horizontal lines (Y axis) - flip Y so 0,0 is at bottom-left
//     for (double y = 0; y <= size.height; y += stepY) {
//       double flippedY = size.height - y;
//       canvas.drawLine(Offset(0, flippedY), Offset(size.width, flippedY), paint);
//     }
//   }

//   @override
//   bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
// }

class RoomPainterMap extends CustomPainter {
  final List<Offset> boundaryPoints; // in world meters
  final Offset origin; // in world meters
  final double mapWidthMeters;
  final double mapHeightMeters;
  final double pixelsPerMeter; // Adjust based on your scal

  RoomPainterMap({
    required this.boundaryPoints,
    required this.origin,
    required this.mapWidthMeters,
    required this.mapHeightMeters,
    required this.pixelsPerMeter,
  });




  @override
  void paint(Canvas canvas, Size size) {
    final paint =
        Paint()
          ..color = Colors.green.withOpacity(0.3)
          ..style = PaintingStyle.fill;

    if (boundaryPoints.length < 3) return;

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
          ..color = Colors.green
          ..strokeWidth = 2
          ..style = PaintingStyle.stroke;
    canvas.drawPath(path, border);
  }

  @override
  bool shouldRepaint(RoomPainterMap oldDelegate) =>
      oldDelegate.boundaryPoints != boundaryPoints ||
      oldDelegate.origin != origin;
}
