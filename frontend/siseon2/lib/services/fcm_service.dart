// 📁 lib/services/fcm_service.dart
import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:flutter/material.dart';
import '../main.dart'; // navigatorKey
import 'preset_service.dart'; // ✅ POST 호출용

class FCMService {
  static final FirebaseMessaging _messaging = FirebaseMessaging.instance;

  @pragma('vm:entry-point')
  static Future<void> backgroundHandler(RemoteMessage message) async {
    await Firebase.initializeApp();
    _debug('[BG] ${message.data}');
    // ⚠️ 여기선 UI/네비/POST 금지(앱 프로세스 컨텍스트 없음)
  }

  static Future<void> initialize() async {
    await Firebase.initializeApp();
    await _messaging.requestPermission();

    FirebaseMessaging.onMessage.listen((msg) {
      _debug('[FG] onMessage data=${msg.data}');
      _handleMessage(msg, fromClick: false);
    });

    FirebaseMessaging.onMessageOpenedApp.listen((msg) {
      _debug('[CLICK] onMessageOpenedApp data=${msg.data}');
      _handleMessage(msg, fromClick: true);
    });

    final initial = await _messaging.getInitialMessage();
    if (initial != null) {
      _debug('[INIT] getInitialMessage data=${initial.data}');
      _handleMessage(initial, fromClick: true);
    }

    FirebaseMessaging.onBackgroundMessage(backgroundHandler);
  }

  static void _handleMessage(RemoteMessage message, {required bool fromClick}) {
    final data = message.data;
    final title = message.notification?.title ?? 'SISEON';
    final body  = message.notification?.body ?? '';

    final type    = (data['type'] ?? '').toString();        // 'posture'
    final subtype = (data['subtype'] ?? '').toString();     // 'bad_posture' | 'preset_suggest'
    final profileIdStr = (data['profileId'] ?? '').toString();
    final int? profileId = int.tryParse(profileIdStr);

    final isLegacyPreset = type == 'preset' && subtype.isEmpty;

    if (type == 'posture' || isLegacyPreset) {
      final kind = isLegacyPreset ? 'preset_suggest' : subtype;

      switch (kind) {
        case 'bad_posture':
          _showInfoPopup(title, body.isNotEmpty ? body : '잘못된 자세입니다. 교정해주세요.');
          return;

        case 'preset_suggest':
        // ✅ 포그라운드/클릭 진입 구분 없이 '저장할까요?' 다이얼로그 → 예=POST
          _showPresetPopupAndConfirm(profileId: profileId);
          return;

        default:
          _showInfoPopup(title, body.isNotEmpty ? body : '알림을 확인해주세요.');
          return;
      }
    } else {
      _showInfoPopup(title, body.isNotEmpty ? body : '알림을 확인해주세요.');
    }
  }

  // ────────────────────────────────────────────────────────────────────────────
  // UI helpers
  // ────────────────────────────────────────────────────────────────────────────
  static void _showPresetPopupAndConfirm({int? profileId}) {
    final context = navigatorKey.currentContext;
    if (context == null) return;

    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => AlertDialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: const Text('SISEON', style: TextStyle(fontWeight: FontWeight.bold)),
        content: const Text('이 자세를 프리셋에 저장하시겠습니까?'),
        actionsPadding: const EdgeInsets.fromLTRB(16, 0, 16, 16),
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
                  onPressed: () async {
                    Navigator.of(context).pop(); // 확인창 닫고
                    await _confirmPresetAndToast(profileId: profileId); // ✅ 바로 POST
                  },
                  child: const Text('예', style: TextStyle(color: Colors.white, fontSize: 16)),
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
                  onPressed: () => Navigator.of(context).pop(),
                  child: const Text('아니오', style: TextStyle(color: Colors.grey, fontSize: 16)),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  // ✅ POST 호출 전 '프리셋 개수' 확인 → 3개면 안내 후 중단
  //    성공/실패는 스낵바로 안내
  static Future<void> _confirmPresetAndToast({int? profileId}) async {
    final ctx = navigatorKey.currentContext;
    if (ctx == null) return;

    if (profileId == null) {
      ScaffoldMessenger.of(ctx).showSnackBar(
        const SnackBar(content: Text('❌ profileId가 없어 저장할 수 없어요')),
      );
      return;
    }

    // 1) 현재 프리셋 개수 확인
    List<Map<String, dynamic>> list = const [];
    try {
      list = await PresetService.fetchPresets(profileId);
    } catch (_) {
      // fetchPresets가 내부에서 [] 반환하는 구조라면 여기 안탐. 혹시 모를 예외 대비.
    }

    if (list.length >= 3) {
      ScaffoldMessenger.of(ctx).showSnackBar(
        const SnackBar(content: Text('❌ 프리셋은 최대 3개까지입니다. 기존 프리셋을 삭제한 뒤 다시 시도해 주세요.')),
      );
      return;
    }

    // 2) 저장 진행 (로딩 다이얼로그 표시)
    final nextName = '프리셋 ${list.length + 1}';
    _showBlockingProgress(ctx, '프리셋 저장 중…');
    try {
      await PresetService.confirm(profileId: profileId, name: nextName); // 서버에서 name 옵션 사용
      Navigator.of(ctx, rootNavigator: true).pop(); // progress 닫기
      ScaffoldMessenger.of(ctx).showSnackBar(
        const SnackBar(content: Text('✅ 프리셋이 저장되었습니다')),
      );
    } catch (e) {
      Navigator.of(ctx, rootNavigator: true).pop();
      ScaffoldMessenger.of(ctx).showSnackBar(
        SnackBar(content: Text('❌ 저장 실패: $e')),
      );
    }
  }

  static void _showBlockingProgress(BuildContext ctx, String msg) {
    showDialog(
      context: ctx,
      barrierDismissible: false,
      builder: (_) => WillPopScope(
        onWillPop: () async => false,
        child: Dialog(
          backgroundColor: const Color(0xFF161B22),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          child: Padding(
            padding: const EdgeInsets.all(20),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                const SizedBox(
                  width: 22, height: 22,
                  child: CircularProgressIndicator(strokeWidth: 2),
                ),
                const SizedBox(width: 12),
                Flexible(
                  child: Text(
                    msg,
                    style: const TextStyle(color: Colors.white70),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
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
            onPressed: () => Navigator.of(context).pop(),
            child: const Text('확인'),
          )
        ],
      ),
    );
  }

  static void _debug(Object? msg) {
    // ignore: avoid_print
    print('📩 FCM $msg');
  }
}
