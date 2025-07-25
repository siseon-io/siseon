// 📁 pages/home_screen.dart

import 'package:flutter/material.dart';

class HomeScreen extends StatelessWidget {
  final String userName;

  const HomeScreen({super.key, this.userName = '사용자'}); // 기본값 설정

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('홈'),
        backgroundColor: Colors.blue,
      ),
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              '안녕하세요, $userName님!',
              style: const TextStyle(fontSize: 24),
            ),
            const SizedBox(height: 20),
            const Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: TextField(
                decoration: InputDecoration(
                  border: OutlineInputBorder(),
                  labelText: '여기에 입력하세요',
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
