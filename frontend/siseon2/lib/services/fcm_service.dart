import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:flutter/material.dart';
import '../main.dart'; // navigatorKey 불러오기

class FCMService {
  static final FirebaseMessaging _messaging = FirebaseMessaging.instance;

  /// 🔥 백그라운드 메시지 핸들러
  @pragma('vm:entry-point')
  static Future<void> backgroundHandler(RemoteMessage message) async {
    await Firebase.initializeApp();
    print("📩 백그라운드 메시지: ${message.notification?.title}");
  }

  /// ✅ 초기화 (앱 시작 시 호출)
  static Future<void> initialize() async {
    // 권한 요청 (Android 13+ 필수)
    await _messaging.requestPermission();

    // FCM 토큰 발급
    final token = await _messaging.getToken();
    print("📱 FCM Token: $token");

    // 🔥 토큰 서버 전송 로직 추가 가능 (예: AuthService.uploadFcmToken(token))

    // 포그라운드 메시지 처리
    FirebaseMessaging.onMessage.listen((message) {
      print("📩 포그라운드 메시지: ${message.notification?.title}");
      _showPopup(message.notification?.title, message.notification?.body);
    });

    // 알림 클릭 처리
    FirebaseMessaging.onMessageOpenedApp.listen((message) {
      print("📩 알림 클릭: ${message.data}");
      // 필요 시 알림 클릭 시 특정 화면 이동 로직 가능
    });

    // 백그라운드 메시지 핸들러 등록
    FirebaseMessaging.onBackgroundMessage(backgroundHandler);
  }

  /// 중앙 팝업 표시 (전역 navigatorKey 사용)
  static void _showPopup(String? title, String? body) {
    final context = navigatorKey.currentContext; // ✅ 항상 살아있는 Context
    if (context == null) return;

    showDialog(
      context: context,
      builder: (_) => AlertDialog(
        title: Text(title ?? "알림"),
        content: Text(body ?? "새 알림이 도착했습니다."),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context), child: const Text("확인")),
        ],
      ),
    );
  }
}
