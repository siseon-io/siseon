import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_dotenv/flutter_dotenv.dart'; // ✅ dotenv 추가
import 'package:firebase_core/firebase_core.dart';

import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/fcm_service.dart';

import '/login_screen.dart';
import '/profile_select_screen.dart';

/// 전역 네비게이터 키 (알림 팝업에 사용)
final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // ✅ 환경변수 로드 (.env)
  await dotenv.load(fileName: ".env");

  // ✅ Firebase 초기화 (FCM 사용을 위해 반드시 선행)
  await Firebase.initializeApp();

  // ✅ MQTT 연결
  await mqttService.connect();

  // ✅ 세로 화면 고정
  await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);

  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      navigatorKey: navigatorKey, // ✅ 전역 navigatorKey 등록
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        fontFamily: 'Pretendard',
        scaffoldBackgroundColor: const Color(0xFF0D1117),
      ),
      home: const SplashScreen(),
    );
  }
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
    initApp();
  }

  Future<void> initApp() async {
    await Future.delayed(const Duration(seconds: 2));

    // ✅ FCM 초기화 (Firebase 초기화 후 실행)
    await FCMService.initialize();

    // ✅ 로그인 여부 확인 후 화면 분기
    final token = await AuthService.getValidAccessToken();
    if (token != null) {
      await ProfileCacheService.clearProfile();
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
      );
    } else {
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (_) => const LoginScreen()),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return const Scaffold(
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Image(
              image: AssetImage('assets/images/eye_open.png'),
              width: 100,
              height: 100,
            ),
            SizedBox(height: 24),
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
