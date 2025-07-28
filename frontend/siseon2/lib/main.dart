import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'login_screen.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.portraitUp,
  ]);

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
      backgroundColor: Color(0xFF0D1117), // 💠 고급진 어두운 남색
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // 👁️ 눈 아이콘 (열린 상태)
            Image(
              image: AssetImage('assets/images/eye_open.png'),
              width: 100,
              height: 100,
            ),
            SizedBox(height: 24),

            // 💬 앱 이름
            Text(
              'SISEON',
              style: TextStyle(
                fontSize: 30,
                fontWeight: FontWeight.w700,
                letterSpacing: 6,
                color: Colors.white,
              ),
            ),

            SizedBox(height: 12),

            // 💡 슬로건
            Text(
              'Smart Vision for Your Comfort',
              style: TextStyle(
                fontSize: 14,
                color: Colors.white60,
                letterSpacing: 1.2,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
