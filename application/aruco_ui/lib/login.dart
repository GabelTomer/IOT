import 'package:flutter/material.dart';
import 'package:logger/logger.dart';
import 'package:validator_regex/validator_regex.dart';
import 'room.dart';
class ArucoApp extends StatelessWidget {
  final Logger logger;
  const ArucoApp({super.key, required this.logger});

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
      home: ConnectionPage(logger:logger),
    );
  }
}

class ConnectionPage extends StatefulWidget {
  final Logger logger;
  const ConnectionPage({super.key, required this.logger});

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
      widget.logger.i("Connecting to robot at $ip with password $password");
      passwordController.clear();
      Navigator.push(
        context,
        MaterialPageRoute(
          builder: (context) => RoomSelectorPage(ipAddress: ip, logger: widget.logger),
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
