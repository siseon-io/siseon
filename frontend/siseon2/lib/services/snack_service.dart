import 'package:flutter/material.dart';
import '../main.dart'; // scaffoldMessengerKey

class SnackService {
  static ScaffoldFeatureController<SnackBar, SnackBarClosedReason>? _current;
  static DateTime? _lastAt;
  static String? _lastMsg;

  /// 항상 "최신 알림만" 보이도록 설계
  /// - 이미 떠있는 스낵바가 있으면 **즉시 제거**하고 새 알림 표시
  /// - dedupeWindow 안에 같은 메시지는 중복 방지
  static ScaffoldFeatureController<SnackBar, SnackBarClosedReason>? show(
      String message, {
        Duration duration = const Duration(seconds: 2),
        Duration dedupeWindow = const Duration(milliseconds: 600),
        bool floating = true,
      }) {
    final messenger = scaffoldMessengerKey.currentState;
    if (messenger == null) return null;

    final now = DateTime.now();

    // 같은 메시지를 짧은 시간 안에 또 띄우면 무시 (깜빡임 방지)
    if (_lastMsg == message && _lastAt != null && now.difference(_lastAt!) < dedupeWindow) {
      return null;
    }

    // 현재 떠있는 스낵바 즉시 정리하고 최신으로 교체
    try {
      messenger.clearSnackBars();
      _current?.close();
    } catch (_) {}

    _lastMsg = message;
    _lastAt = now;

    final ctrl = messenger.showSnackBar(
      SnackBar(
        content: Text(message),
        duration: duration,
        behavior: floating ? SnackBarBehavior.floating : SnackBarBehavior.fixed,
      ),
    );

    _current = ctrl;
    ctrl.closed.whenComplete(() {
      // 내가 띄운 컨트롤러가 닫힌 것이 맞을 때만 정리
      if (identical(_current, ctrl)) {
        _current = null;
      }
    });

    return ctrl;
  }

  /// 지금 떠있는 스낵바가 있으면 지우고 **바로 교체** (show와 동일한 정책)
  static void replace(String message, {Duration duration = const Duration(seconds: 2)}) {
    show(message, duration: duration);
  }

  /// 강제 종료
  static void clear() {
    scaffoldMessengerKey.currentState?.clearSnackBars();
    try {
      _current?.close();
    } catch (_) {}
    _current = null;
  }
}
