// 📁 lib/main.dart
import 'dart:io'; // ← 플랫폼 가드용

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

// ⬇️ Foreground service & permission
import 'package:flutter_foreground_task/flutter_foreground_task.dart';
import 'package:permission_handler/permission_handler.dart';

import '/login_screen.dart';
import '/profile_select_screen.dart';

// ✅ 전역 내비게이터 & 스낵바 키 (스낵바 줄세우기 방지용 서비스가 이 키를 사용)
final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();
final GlobalKey<ScaffoldMessengerState> scaffoldMessengerKey =
GlobalKey<ScaffoldMessengerState>();

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  await initializeDateFormatting('ko_KR', null);
  await dotenv.load(fileName: ".env");

  // ✅ Firebase & FCM를 가장 먼저 세팅
  await Firebase.initializeApp();
  await FCMService.initialize(); // ⬅️ 여기서 설정(중요)

  // ✅ ForegroundTask 초기화 (v9.x: await 금지, const 금지)
  FlutterForegroundTask.init(
    androidNotificationOptions: AndroidNotificationOptions(
      channelId: 'siseon_ble_channel',
      channelName: 'Siseon BLE 연결 유지',
      channelDescription: 'BLE 연결을 백그라운드에서 안정적으로 유지합니다.',
      channelImportance: NotificationChannelImportance.LOW,
      priority: NotificationPriority.LOW,
      playSound: false,
      enableVibration: false,
      visibility: NotificationVisibility.VISIBILITY_PUBLIC,
    ),
    iosNotificationOptions: IOSNotificationOptions(), // iOS는 사용 안 함
    foregroundTaskOptions: ForegroundTaskOptions(
      // v9.x: interval 대신 eventAction 사용
      eventAction: ForegroundTaskEventAction.repeat(15000),
      allowWakeLock: true,
      allowWifiLock: true,
      autoRunOnBoot: false,
      autoRunOnMyPackageReplaced: false,
    ),
  );

  // ✅ Android 13+ 알림 권한 (포그라운드 서비스 알림 표시용)
  if (Platform.isAndroid) {
    final notif = await Permission.notification.status;
    if (notif.isDenied || notif.isPermanentlyDenied) {
      await Permission.notification.request();
    }
  }

  // ✅ 나머지 초기화
  await mqttService.connect();
  await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);

  // ✅ ForegroundTask가 동작하려면 이 래퍼로 감싸서 runApp
  runApp(
    WithForegroundTask(
      child: const MyApp(),
    ),
  );
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      navigatorKey: navigatorKey,
      scaffoldMessengerKey: scaffoldMessengerKey, // ⬅️ 전역 스낵바 키 연결 (중요)
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
