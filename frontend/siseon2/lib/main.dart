// 📁 lib/main.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_dotenv/flutter_dotenv.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/device_cache_service.dart';
import 'package:siseon2/services/fcm_service.dart';
import 'package:intl/date_symbol_data_local.dart';

import '/login_screen.dart';
import '/profile_select_screen.dart';

final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  await initializeDateFormatting('ko_KR', null);
  await dotenv.load(fileName: ".env");

  // ✅ Firebase & FCM를 가장 먼저 세팅
  await Firebase.initializeApp();
  await FCMService.initialize();        // ⬅️ 여기서 설정(중요)

  // ✅ 나머지 초기화
  await mqttService.connect();
  await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);

  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      navigatorKey: navigatorKey,
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
    // ⏳ 간단한 스플래시 연출
    await Future.delayed(const Duration(seconds: 2));

    // ❌ 여기서 FCMService.initialize() 호출하던 거 제거함

    // ✅ 로그인 여부 확인
    final token = await AuthService.getValidAccessToken();
    if (!mounted) return;

    if (token != null) {
      await ProfileCacheService.loadProfile();
      await DeviceCacheService.loadDevice();
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
