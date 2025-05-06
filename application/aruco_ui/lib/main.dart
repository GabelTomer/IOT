import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'package:logger/logger.dart';
import 'package:validator_regex/validator_regex.dart';

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

  void ConnectToRobot() {
    String ip = ipController.text;
    String password = passwordController.text;
    if (ip.isNotEmpty && password.isNotEmpty) {
      if(password != this.password) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Incorrect Password')),
        );
        return;
      }
      if(Validator.ipAddress(ip) == false) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Invalid IP Address')),
        );
        return;
      }
      logger.i("Connecting to robot at $ip with password $password");
      Navigator.push(
        context,
        MaterialPageRoute(
          builder: (context) => RobotControl(ipAddress: ip),
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

class RobotControl extends StatefulWidget {
  final String ipAddress;
  const RobotControl({
    super.key,
    required this.ipAddress
  });

  @override
  State<RobotControl> createState() => _RobotControl();
}

class _RobotControl extends State<RobotControl> {
  Timer? timer;
  @override
  void initState() {
    super.initState();
    startFetchPosition();
  }
  bool errorOccurred = true;
  String error = "Connection error";
  String robotPosition = "Fetching...";
  double robotX = 0;
  double robotY = 0;
  bool isConnected = false;
  int scaleFactor = 100;
  var knownMarkers = <String, Offset>{};
  void startFetchPosition() async{
    await fetchKnownMarkers();
    timer = Timer.periodic(const Duration(seconds: 1), (timer) async{
      await fetchRobotPosition();
    });
  }

  Future<void> fetchKnownMarkers() async {
    try {
      final response = await http.get(
        Uri.parse('http://${widget.ipAddress}:5000/get_known_markers'),
      );
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          for (var marker in data) {
            knownMarkers[marker['id']] = Offset(marker['x'].toDouble(), marker['y'].toDouble());
          }
          logger.i("Known markers fetched successfully!");
          isConnected = true;
          errorOccurred = false;
        });
      } else {
        setState(() {
          error = "Error: ${response.statusCode}";
          logger.e("Error: ${response.statusCode}");
          isConnected = false;
          errorOccurred = true;
        });
      }
    } catch (e) {
      setState(() {
        isConnected = false;
        error = "Connection error";
        logger.e("Connection error: $e");
        errorOccurred = true;
      });
    }
  }


  Future<void> fetchRobotPosition() async {
    try {
      final response = await http.get(
        Uri.parse('http://${widget.ipAddress}:5000/get_position'),
      );
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          robotPosition = "X: ${data['x'].toStringAsFixed(2)}, Y: ${data['y'].toStringAsFixed(2)}, Z: ${data['z'].toStringAsFixed(2)}";
          robotX = data['x'] * scaleFactor; // Scale for display
          robotY = data['y'] * scaleFactor; // Scale for display
          isConnected = true;
          errorOccurred = false;
        });
      } else {
        setState(() {
          error = "Error: ${response.statusCode}";
          logger.e("Error: ${response.statusCode}");
          isConnected = false;
          errorOccurred = true;
        });
      }
    } catch (e) {
      setState(() {
        isConnected = false;
        error = "Connection error";
        logger.e("Connection error: $e");
        errorOccurred = true;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    double screenWidth = MediaQuery.of(context).size.width;
    double screenHeight = MediaQuery.of(context).size.height;
    Size mapSize = Size(screenWidth * 0.8, screenHeight * 0.4);

    Offset screenToWorld(Offset pos, Size mapSize) {
      const double mapWidthMeters = 10.0;
      const double mapHeightMeters = 10.0;

      double x = (pos.dx / mapSize.width) * mapWidthMeters;
      double y = (pos.dy / mapSize.height) * mapHeightMeters;

      return Offset(x, y);
    }

    Future<void> sendTargetPosition(Offset target) async {
      final url = 'http://${widget.ipAddress}:5000/update_position'; // replace with your endpoint
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
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
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
                      child: Container(
                        color: Colors.transparent, // Required to register taps
                          child:  CustomPaint(
                            size: mapSize,
                            painter: GridPainter(),
                          ),
                        ),
                    ),
                    Positioned(
                        left: robotX,
                        top: robotY,
                        child: Icon(Icons.android, size: 30, color: Colors.green),
                      ),
                    ],
                  ),
                ),
                
              ],
            ),
          ),
          Positioned(
            bottom: 60,
            left: 0,
            right: 0,
            child: Center(
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                child: Text(
                  'Robot Position:',
                  style: Theme.of(context).textTheme.headlineMedium,
                ),
                ),
              )
            ),
            
          Positioned(
            bottom: 20,
            left: 0,
            right: 0,
            child: Center(
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  child: Text(
                    robotPosition,
                    style: Theme.of(context).textTheme.headlineSmall,
                  ),
              ),
            ), 
          ),
        ],
      ),
    );
  }
}




class GridPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint =
        Paint()
          ..color = Colors.grey
          ..strokeWidth = 1;

    double step = size.width / 10; // 10x10 grid

    for (double x = 0; x <= size.width; x += step) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }
    for (double y = 0; y <= size.height; y += step) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
