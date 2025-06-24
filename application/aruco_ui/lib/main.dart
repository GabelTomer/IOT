import 'package:flutter/material.dart';
import 'package:logger/logger.dart';
import 'login.dart';


void main() {
  final logger = Logger();
  logger.i('Aruco Robot UI started');
  runApp(ArucoApp(logger: logger));
}





