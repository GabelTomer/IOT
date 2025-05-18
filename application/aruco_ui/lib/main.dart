import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'package:logger/logger.dart';
import 'package:validator_regex/validator_regex.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_typeahead/flutter_typeahead.dart';

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
      theme: ThemeData(primarySwatch: Colors.blue),
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
        MaterialPageRoute(builder: (context) => roomControl(ipAddress: ip)),
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
      appBar: AppBar(title: const Text('Connect to Robot')),
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

class roomControl extends StatefulWidget {
  final String ipAddress;
  const roomControl({super.key, required this.ipAddress});

  @override
  State<roomControl> createState() => _roomControl();
}

class _roomControl extends State<roomControl> {
  List<String> roomNames = [];
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




  Iterable<String> _roomOptionsBuilder(TextEditingValue textEditingValue) {
    if (textEditingValue.text == '') {
      return roomNames;
    }
    return roomNames.where((String option) {
      return option.toLowerCase().contains(textEditingValue.text.toLowerCase());
    });
  }

 void showEditRoomDialog(BuildContext context)  {
    final roomController = TextEditingController();
    var autoCompleteController = null;
    showDialog(
      context: context,
      builder:
          (context) => AlertDialog(
            title: const Text('Edit Rooms'),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                // TextField(
                //   controller: roomController,
                //   decoration: const InputDecoration(labelText: 'Room Name'),
                // ),
                Autocomplete(
                  optionsBuilder: _roomOptionsBuilder,
                  fieldViewBuilder: (
                    context,
                    roomController,
                    focusNode,
                    onFieldSubmitted,
                  ) {
                    autoCompleteController = roomController;
                    return TextField(
                      controller: roomController,
                      focusNode: focusNode,
                      decoration: const InputDecoration(labelText: 'Room Name'),
                    );
                  },
                  displayStringForOption: (option) => option,
                  onSelected: (option) {
                    roomController.text = option;
                  },
                ),
                SizedBox(height: 20),
              ],
              
            ),
            
            actions: [
              TextButton(
                onPressed: () =>{
                    FocusScope.of(context).unfocus(),
                   Navigator.pop(context)
                   },
                child: const Text('Cancel'),
              ),
              TextButton(
                onPressed: () async {
                   FocusScope.of(context).unfocus(); 
                   Future.microtask(() async {
                  final roomName = autoCompleteController == null ? roomController.text.trim() : autoCompleteController.text.trim();
                  if (roomName.isNotEmpty && !roomNames.contains(roomName)) {
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('Room does not exist')),
                    );
                    return;
                  }
                  if (roomName.isNotEmpty) {
                    await deleteRoom(roomName);
                    Navigator.pop(context);
                    fetchRoomNames();
                    setState(() {}); // Refresh
                  }
                    });
                },
                child: const Text('delete'),
              ),
              TextButton(
                onPressed: () async {
                   FocusScope.of(context).unfocus(); 
                   Future.microtask(() async {
                  final roomName = autoCompleteController == null ? roomController.text.trim() : autoCompleteController.text.trim();
                  if (roomName.isNotEmpty && roomNames.contains(roomName)) {
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('Room Name already exists')),
                    );
                    return;
                  }
                  if (roomName.isNotEmpty) {
                    await addRoom(roomName);
                    Navigator.pop(context);
                    fetchRoomNames();
                    setState(() {}); // Refresh
                  }
                   });
                },
                child: const Text('Add'),
              ),
            ],
          ),
    );
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
      appBar: AppBar(title: const Text('Choose Room')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text('Choose a room:'),
            DropdownButton<String>(
              value: selectedRoom,
              hint: Text("Select a Room"),
              items:
                  roomNames.map((room) {
                    return DropdownMenuItem(value: room, child: Text(room));
                  }).toList(),
              onChanged: (value) {
                setState(() {
                  selectedRoom = value;
                });
              },
            ),
            const SizedBox(height: 30),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const SizedBox(width: 20),
                ElevatedButton(
                  onPressed: () => showEditRoomDialog(context),
                  child: const Text('Edit Rooms'),
                ),
                const SizedBox(width: 20),
                ElevatedButton(
                  onPressed: callRobotControl,
                  child: const Text('Connect'),
                ),
              ],
            ),
          ],
        ),
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

  // Adjust based on your robot's IP
  final String robotIP = "192.168.1.104";

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
            });
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
      appBar: AppBar(title: const Text('Robot ArUco UI')),
      body: Stack(
        children: [
          // Status Indicator in the top-right corner
          Positioned(
            top: 0,
            left: 0,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              child: Text(
                isConnected ? 'Connected' : 'Disconnected',
                style: TextStyle(
                  color: isConnected ? Colors.green : Colors.red,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ),
          // Main content in the center
          Positioned(
            top: 20,
            left: 0,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              child: Text(
                error,
                style: TextStyle(
                  color: errorOccurred ? Colors.red : Colors.white,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ),

          Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              mainAxisAlignment: MainAxisAlignment.start,
              children: [
                SizedBox(height: 80),
                Container(
                  width: screenWidth * 0.8,
                  height: screenHeight * 0.4,
                  color: Colors.blue[50],
                  child: Stack(
                    children: [
                      GestureDetector(
                        onTapDown: (TapDownDetails details) {
                          final tapPosition = details.localPosition;
                          handleMapTap(tapPosition);
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
                            painter: GridPainter(),
                          ),
                        ),
                      ),
                      for (var marker in knownMarkers.entries)
                        Positioned(
                          left:
                              worldToScreen(
                                marker.value.position,
                                mapSize,
                                mapWidthMeters,
                                mapHeightMeters,
                              ).dx -
                              10,
                          top:
                              worldToScreen(
                                marker.value.position,
                                mapSize,
                                mapWidthMeters,
                                mapHeightMeters,
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
                              mapSize,
                              mapWidthMeters,
                              mapHeightMeters,
                            ).dx,
                        top:
                            worldToScreen(
                              Offset(robotX, robotY),
                              mapSize,
                              mapWidthMeters,
                              mapHeightMeters,
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
          Align(
            alignment: Alignment.topRight,
            child: ElevatedButton(
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

Offset worldToScreen(
  Offset world,
  Size mapSize,
  double mapWidthMeters,
  double mapHeightMeters,
) {
  double x = (world.dx.floor() / mapWidthMeters.floor()) * mapSize.width;
  double y =
      mapSize.height.floor() -
      (world.dy.floor() / mapHeightMeters.floor()) *
          mapSize.height; // Flip Y axis

  // Make sure x and y are within the bounds of the map
  x = x.clamp(0.0, mapSize.width);
  y = y.clamp(0.0, mapSize.height);

  return Offset(x.floorToDouble(), y.floorToDouble());
}

class GridPainter extends CustomPainter {
  final int gridSize = 10;

  @override
  void paint(Canvas canvas, Size size) {
    final paint =
        Paint()
          ..color = Colors.grey
          ..strokeWidth = 1;

    double stepX = size.width / gridSize;
    double stepY = size.height / gridSize;

    // Vertical lines (X axis)
    for (double x = 0; x <= size.width; x += stepX) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }

    // Horizontal lines (Y axis) - flip Y so 0,0 is at bottom-left
    for (double y = 0; y <= size.height; y += stepY) {
      double flippedY = size.height - y;
      canvas.drawLine(Offset(0, flippedY), Offset(size.width, flippedY), paint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
