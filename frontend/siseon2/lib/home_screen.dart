import 'package:flutter/material.dart';

class HomeScreen extends StatelessWidget {
  final String userName;

  const HomeScreen({super.key, required this.userName});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('홈'),
        backgroundColor: Colors.blue,
      ),
      body: Center(
        child: Text(
          '안녕하세요, $userName님!',
          style: const TextStyle(fontSize: 24),
        ),
      ),
    );
  }
}
