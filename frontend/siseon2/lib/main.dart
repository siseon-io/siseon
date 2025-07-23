import 'package:flutter/material.dart';
import 'login_screen.dart';

void main() {
  runApp(const MaterialApp(
    debugShowCheckedModeBanner: false,
    home: SplashScreen(),
  ));
}

class SplashScreen extends StatefulWidget {
  const SplashScreen({super.key});

  @override
  State<SplashScreen> createState() => _SplashScreenState();
}

class _SplashScreenState extends State<SplashScreen> {
  @override
  void initState() {
    super.initState();
    Future.delayed(const Duration(seconds: 2), () {
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (context) => const LoginScreen()),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return const Scaffold(
      backgroundColor: Color(0xFF2563FF), // 파란 배경
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Image(
              image: AssetImage('assets/images/eye_closed.png'),
              width: 120,
              height: 120,
            ),
            SizedBox(height: 16),
            Text(
              'SISEON',
              style: TextStyle(
                color: Colors.white,
                fontSize: 24,
                letterSpacing: 2,
              ),
            )
          ],
        ),
      ),
    );
  }
}
