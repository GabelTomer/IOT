import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';

void main() {
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

  String robotPosition = "Unknown";

  double robotX = 100;
  double robotY = 100;
  bool isConnected = false;
  int scaleFactor = 100;
  void startFetchPosition(){
    timer = Timer.periodic(const Duration(seconds: 1), (timer) async{
      await fetchRobotPosition();
    });
  }
  Future<void> fetchRobotPosition() async {
    try {
      final response = await http.get(
        Uri.parse('http://${widget.ipAddress}:5000/get_position'),
      );
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          robotPosition = "X: ${data['x']}, Y: ${data['y']}, Z: ${data['z']}";
          robotX = data['x'] * scaleFactor; // Scale for display
          robotY = data['y'] * scaleFactor; // Scale for display
          isConnected = true;
        });
      } else {
        setState(() {
          robotPosition = "Error: ${response.statusCode}";
        });
      }
    } catch (e) {
      setState(() {
        isConnected = false;
        robotPosition = "Connection error";
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    double screenWidth = MediaQuery.of(context).size.width;
    double screenHeight = MediaQuery.of(context).size.height;

    return Scaffold(
      appBar: AppBar(title: const Text('Robot ArUco UI')),
      body: Stack(
        children: [
          // Status Indicator in the top-right corner
          Positioned(
            top: 20,
            right: 20,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              decoration: BoxDecoration(
                color: isConnected ? Colors.green : Colors.red,
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                isConnected ? 'Connected' : 'Disconnected',
                style: const TextStyle(
                  color: Colors.white,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ),
          // Main content in the center
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
                      CustomPaint(
                        size: Size(screenWidth * 0.8, screenHeight * 0.4),
                        painter: GridPainter(),
                      ),
                      Positioned(
                        left: robotX,
                        top: robotY,
                        child: Icon(Icons.android, size: 30, color: Colors.green),
                      ),
                    ],
                  ),
                ),
                Text(
                  'Robot Position:',
                  style: Theme.of(context).textTheme.headlineMedium,
                ),
                const SizedBox(height: 20),
                Text(
                  robotPosition,
                  style: Theme.of(context).textTheme.headlineSmall,
                ),
                const SizedBox(height: 40),
              ],
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
