import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:flutter/material.dart';
import '../main.dart'; // navigatorKey 불러오기
import 'package:siseon2/pages/settings/preset_page.dart';

class FCMService {
  static final FirebaseMessaging _messaging = FirebaseMessaging.instance;

  /// 🔥 백그라운드 메시지 핸들러
  @pragma('vm:entry-point')
  static Future<void> backgroundHandler(RemoteMessage message) async {
    await Firebase.initializeApp();
    print("📩 [백그라운드 메시지]: ${message.notification?.title}");
  }

  /// ✅ 초기화 (앱 시작 시 알림 리스너 등록만 수행)
  static Future<void> initialize() async {
    // 1. 알림 권한 요청
    await _messaging.requestPermission();

    // 2. 포그라운드 메시지 수신
    FirebaseMessaging.onMessage.listen((message) {
      print("📩 [포그라운드] ${message.notification?.title}");
      _handleNotification(message);
    });

    // 3. 알림 클릭 시 (앱 백그라운드 → 포그라운드)
    FirebaseMessaging.onMessageOpenedApp.listen((message) {
      print("📲 [알림 클릭]: ${message.data}");
      _handleNotification(message, fromClick: true);
    });

    // 4. 백그라운드 메시지 등록
    FirebaseMessaging.onBackgroundMessage(backgroundHandler);
  }

  /// ✅ 알림 처리
  static void _handleNotification(RemoteMessage message, {bool fromClick = false}) {
    final title = message.notification?.title ?? "SISEON";
    final body = message.notification?.body ?? "";

    // 알림 데이터 타입에 따라 분기 가능 (예: type: preset)
    if (message.data['type'] == 'preset') {
      _showPresetPopup();
    } else {
      _showInfoPopup(title, body);
    }
  }

  /// 🔵 프리셋 저장 팝업
  static void _showPresetPopup() {
    final context = navigatorKey.currentContext;
    if (context == null) return;

    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: const Text("SISEON", style: TextStyle(fontWeight: FontWeight.bold)),
        content: const Text("이 자세를 프리셋에 저장하시겠습니까?"),
        actions: [
          Row(
            children: [
              Expanded(
                child: ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color(0xFF3B82F6),
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                    padding: const EdgeInsets.symmetric(vertical: 12),
                  ),
                  onPressed: () {
                    Navigator.pop(context);
                    Navigator.push(context, MaterialPageRoute(builder: (_) => const PresetPage()));
                  },
                  child: const Text("예", style: TextStyle(color: Colors.white, fontSize: 16)),
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: OutlinedButton(
                  style: OutlinedButton.styleFrom(
                    side: const BorderSide(color: Colors.grey),
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                    padding: const EdgeInsets.symmetric(vertical: 12),
                  ),
                  onPressed: () => Navigator.pop(context),
                  child: const Text("아니오", style: TextStyle(color: Colors.grey, fontSize: 16)),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  /// 🔔 일반 알림 팝업
  static void _showInfoPopup(String title, String body) {
    final context = navigatorKey.currentContext;
    if (context == null) return;

    showDialog(
      context: context,
      builder: (_) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text(title, style: const TextStyle(fontWeight: FontWeight.bold)),
        content: Text(body),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text("확인"),
          )
        ],
      ),
    );
  }
}
