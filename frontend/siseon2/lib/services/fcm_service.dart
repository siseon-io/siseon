// 📁 lib/services/fcm_service.dart
import 'dart:io';
import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:flutter/material.dart';
import '../main.dart'; // navigatorKey
import 'preset_service.dart'; // ✅ POST 호출용

// ────────────────────────────────────────────────────────────────────────────
// (중요) 백그라운드 핸들러는 반드시 최상위 함수여야 함 (Android 프로세스에서 진입)
// ────────────────────────────────────────────────────────────────────────────
@pragma('vm:entry-point')
Future<void> firebaseMessagingBackgroundHandler(RemoteMessage message) async {
  await Firebase.initializeApp();
  // 여기선 UI/네비/POST 금지(앱 프로세스 컨텍스트 없음)
  // ignore: avoid_print
}

class FCMService {
  static final FirebaseMessaging _messaging = FirebaseMessaging.instance;

  // 중복 UI/요청 가드
  static bool _dialogShown = false;
  static bool _savingPreset = false;

  static Future<void> initialize() async {
    if (Firebase.apps.isEmpty) {
      await Firebase.initializeApp();
    }

    // iOS 포그라운드 표시 옵션
    if (Platform.isIOS) {
      await _messaging.setForegroundNotificationPresentationOptions(
        alert: true, badge: true, sound: true,
      );
    }

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

    FirebaseMessaging.onBackgroundMessage(firebaseMessagingBackgroundHandler);
  }


  static void _handleMessage(RemoteMessage message, {required bool fromClick}) {
    final data = message.data;
    final title = message.notification?.title ?? 'SISEON';
    final body  = message.notification?.body ?? '';

    final type    = (data['type'] ?? '').toString().toLowerCase();       // 'posture' | 'preset'(레거시)
    final subtype = (data['subtype'] ?? '').toString().toLowerCase();    // 'bad_posture' | 'preset_suggest'
    final profileIdStr = (data['profileId'] ?? '').toString();
    final int? profileId = int.tryParse(profileIdStr);

    // 서버가 이름 넣어줄 수 있음: presetName or name
    final presetNameFromServer =
    ((data['presetName'] ?? data['name']) ?? '').toString().trim();

    final isLegacyPreset = type == 'preset' && subtype.isEmpty;

    if (type == 'posture' || isLegacyPreset) {
      final kind = isLegacyPreset ? 'preset_suggest' : subtype;

      switch (kind) {
        case 'bad_posture':
          _showInfoPopup(title, body.isNotEmpty ? body : '잘못된 자세입니다. 교정해주세요.');
          return;

        case 'preset_suggest':
        // 포그라운드/클릭 구분 없이 다이얼로그 → 예=POST
          _showPresetPopupAndConfirm(profileId: profileId, presetName: presetNameFromServer);
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
  static void _showPresetPopupAndConfirm({int? profileId, String? presetName}) {
    final context = navigatorKey.currentContext;
    if (context == null) {
      _debug('컨텍스트 없음: 프리셋 확인 다이얼로그를 띄울 수 없음');
      return;
    }
    if (_dialogShown) {
      _debug('이미 다이얼로그가 떠 있음: 중복 표시 방지');
      return;
    }
    _dialogShown = true;

    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => AlertDialog(
        backgroundColor: const Color(0xFF161B22),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: const Text(
          'SISEON',
          style: TextStyle(fontWeight: FontWeight.bold, color: Colors.white),
        ),
        content: const Text(
          '이 자세를 프리셋에 저장하시겠습니까?',
          style: TextStyle(color: Colors.white70),
        ),
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
                    Navigator.of(context, rootNavigator: true).pop();
                    await _confirmPresetAndToast(profileId: profileId, presetName: presetName);
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
                  onPressed: () => Navigator.of(context, rootNavigator: true).pop(),
                  child: const Text('아니오', style: TextStyle(color: Colors.grey, fontSize: 16)),
                ),
              ),
            ],
          ),
        ],
      ),
    ).whenComplete(() {
      _dialogShown = false;
    });
  }

  // ✅ POST 호출 전 '프리셋 개수' 확인 → 3개면 안내 후 중단
  //    성공/실패는 스낵바로 안내
  static Future<void> _confirmPresetAndToast({int? profileId, String? presetName}) async {
    final ctx = navigatorKey.currentContext;
    if (ctx == null) {
      _debug('컨텍스트 없음: 프리셋 저장 처리 불가');
      return;
    }
    if (_savingPreset) {
      _debug('이미 프리셋 저장 진행 중: 중복 요청 방지');
      return;
    }

    if (profileId == null) {
      ScaffoldMessenger.of(ctx).showSnackBar(
        const SnackBar(content: Text('❌ profileId가 없어 저장할 수 없어요')),
      );
      return;
    }

    _savingPreset = true;
    try {
      // 1) 현재 프리셋 개수 확인
      List<Map<String, dynamic>> list = const [];
      try {
        list = await PresetService.fetchPresets(profileId);
      } catch (e) {
        _debug('fetchPresets 예외: $e');
        // fetchPresets가 내부에서 [] 반환하면 여기 안 올 수도 있음
      }

      if (list.length >= 3) {
        ScaffoldMessenger.of(ctx).showSnackBar(
          const SnackBar(content: Text('❌ 프리셋은 최대 3개까지입니다. 기존 프리셋을 삭제한 뒤 다시 시도해 주세요.')),
        );
        return;
      }

      // 2) 저장 진행 (로딩 다이얼로그 표시)
      final nextName = (presetName?.isNotEmpty == true)
          ? presetName!
          : '프리셋 ${list.length + 1}';

      _showBlockingProgress(ctx, '프리셋 저장 중…');
      await PresetService.confirm(profileId: profileId, name: nextName); // 서버에서 name 옵션 사용
      Navigator.of(ctx, rootNavigator: true).maybePop(); // progress 닫기

      ScaffoldMessenger.of(ctx).showSnackBar(
        const SnackBar(content: Text('✅ 프리셋이 저장되었습니다')),
      );
    } catch (e) {
      Navigator.of(ctx, rootNavigator: true).maybePop();
      ScaffoldMessenger.of(ctx).showSnackBar(
        SnackBar(content: Text('❌ 저장 실패: $e')),
      );
    } finally {
      _savingPreset = false;
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
    if (context == null) {
      _debug('컨텍스트 없음: info 팝업 생략 ($title / $body)');
      return;
    }
    if (_dialogShown) {
      _debug('이미 다이얼로그가 떠 있음: info 중복 표시 방지');
      return;
    }
    _dialogShown = true;

    showDialog(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: const Color(0xFF161B22),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: Text(title, style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.white)),
        content: Text(body, style: const TextStyle(color: Colors.white70)),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context, rootNavigator: true).pop(),
            child: const Text('확인'),
          )
        ],
      ),
    ).whenComplete(() {
      _dialogShown = false;
    });
  }

  static void _debug(Object? msg) {
    // ignore: avoid_print
  }
}
