import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';

void main() {
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
      ),
      home: const HomeScreen(),
    );
  }
}

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  String robotPosition = "Unknown";
  final String piAddress = "http://192.168.1.100:5000";  // <-- Change to your Raspberry Pi IP

  Future<void> fetchRobotPosition() async {
    try {
      final response = await http.get(Uri.parse('$piAddress/get_position'));
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          robotPosition = "X: ${data['x']}, Y: ${data['y']}, Z: ${data['z']}";
        });
      } else {
        setState(() {
          robotPosition = "Error: ${response.statusCode}";
        });
      }
    } catch (e) {
      setState(() {
        robotPosition = "Connection error";
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Robot ArUco UI'),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
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
            ElevatedButton(
              onPressed: fetchRobotPosition,
              child: const Text('Get Position'),
            ),
          ],
        ),
      ),
    );
  }
}