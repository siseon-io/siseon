// lib/services/ble_session.dart
import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_foreground_task/flutter_foreground_task.dart';

/// ─────────────────────────────────────────────────────────────────────────
/// Android 포그라운드 서비스 Task 핸들러 (연결 유지용)
/// v9.1.0 API: onStart(DateTime, TaskStarter) / onRepeatEvent(DateTime) 등
/// ─────────────────────────────────────────────────────────────────────────
@pragma('vm:entry-point')
class _BleKeepAliveTask extends TaskHandler {
  @override
  Future<void> onStart(DateTime timestamp, TaskStarter starter) async {
    // 필요시 초기 로그/설정
    // starter.name 등을 사용할 수 있음.
  }

  // ForegroundTaskOptions.eventAction 에 따라 주기적으로 호출됨.
  @override
  void onRepeatEvent(DateTime timestamp) {
    // 굳이 아무것도 안 해도 됨(유지 목적).
    // 필요하면 FlutterForegroundTask.updateService(...)로 알림 텍스트 갱신 가능.
    // FlutterForegroundTask.sendDataToMain({...}); 로 메인에 정보 전달도 가능.
  }

  @override
  Future<void> onDestroy(DateTime timestamp, bool isTimeout) async {
    // 정리 필요 시
  }

  @override
  void onNotificationPressed() {
    // 알림 탭 시 앱 포그라운드로 복귀
    FlutterForegroundTask.launchApp();
  }

  // 선택 구현(필요 없으면 비워둬도 OK)
  @override
  void onReceiveData(Object data) {}

  @override
  void onNotificationButtonPressed(String id) {}

  @override
  void onNotificationDismissed() {}
}

@pragma('vm:entry-point')
void _bleKeepAliveStartCallback() {
  FlutterForegroundTask.setTaskHandler(_BleKeepAliveTask());
}

/// ─────────────────────────────────────────────────────────────────────────
/// BLE 세션 (전역 사용)
///  - 화면 이동/백그라운드에서도 연결 유지
///  - Android에선 포그라운드 서비스로 Doze 영향 최소화
/// ─────────────────────────────────────────────────────────────────────────
class BleSession extends ChangeNotifier {
  BluetoothDevice? _device;
  BluetoothCharacteristic? _char;

  BluetoothConnectionState _lastState =
      BluetoothConnectionState.disconnected;

  StreamSubscription<BluetoothConnectionState>? _connSub;

  // 단순 write 직렬화를 위한 가드
  bool _writing = false;

  // 포그라운드 서비스 상태 토글
  bool _fgServiceWanted = false;

  BluetoothDevice? get device => _device;
  BluetoothCharacteristic? get char => _char;

  /// 기기/특성 있고, 실제 연결 상태가 connected면 true
  bool get isReady =>
      _device != null &&
          _char != null &&
          _lastState == BluetoothConnectionState.connected;

  /// 이미 연결된 디바이스/특성을 세션에 주입
  Future<void> setConnected(
      BluetoothDevice d,
      BluetoothCharacteristic c,
      ) async {
    await _connSub?.cancel();
    _device = d;
    _char = c;

    _lastState = await d.state.first;

    _connSub = d.state.listen((s) async {
      _lastState = s;
      if (s == BluetoothConnectionState.disconnected) {
        _device = null;
        _char = null;
        // 연결이 끊기면 포그라운드 서비스도 정리(정책상 꺼두는 방향)
        await _maybeStopForegroundService();
      }
      notifyListeners();
    });

    // 연결 성공 시 포그라운드 서비스 시작 (Android 전용)
    await _maybeStartForegroundService();

    notifyListeners();
  }

  /// 안전한 바이트 쓰기 (간단 직렬화)
  Future<void> writeBytes(
      List<int> data, {
        bool withoutResponse = false,
      }) async {
    final ch = _char;
    if (!isReady || ch == null) {
      throw StateError('BLE not connected');
    }
    while (_writing) {
      await Future<void>.delayed(const Duration(milliseconds: 8));
    }
    _writing = true;
    try {
      await ch.write(data, withoutResponse: withoutResponse);
    } finally {
      _writing = false;
    }
  }

  /// 문자열 편의 쓰기 (UTF-8)
  Future<void> writeString(
      String text, {
        bool withoutResponse = false,
      }) =>
      writeBytes(utf8.encode(text), withoutResponse: withoutResponse);

  /// 읽기 지원 특성이면 read 수행
  Future<List<int>> read() async {
    final ch = _char;
    if (!isReady || ch == null || !ch.properties.read) {
      throw StateError('BLE not readable');
    }
    return ch.read();
  }

  /// 연결 닫고 세션 정리
  Future<void> disconnect() async {
    try {
      await _device?.disconnect();
    } catch (_) {}
    await _connSub?.cancel();
    _connSub = null;
    _device = null;
    _char = null;
    _lastState = BluetoothConnectionState.disconnected;
    await _maybeStopForegroundService();
    notifyListeners();
  }

  /// 앱에서 명시적으로 "장시간 유지" 토글
  Future<void> setKeepAliveEnabled(bool enable) async {
    _fgServiceWanted = enable;
    if (enable) {
      await _maybeStartForegroundService();
    } else {
      await _maybeStopForegroundService();
    }
  }

  /// ── Android Foreground Service 제어 (v9 API) ───────────────────────────
  Future<void> _maybeStartForegroundService() async {
    if (!Platform.isAndroid) return;
    if (!isReady) return;

    _fgServiceWanted = true;

    // v9: interval 대신 eventAction 사용
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
      iosNotificationOptions: IOSNotificationOptions(),
      foregroundTaskOptions: ForegroundTaskOptions(
        // v9.x: interval 대신 eventAction 사용(단위: ms)
        eventAction: ForegroundTaskEventAction.repeat(15000),
        autoRunOnBoot: false,
        autoRunOnMyPackageReplaced: false,
        allowWakeLock: true,
        allowWifiLock: true,
      ),
    );


    final running = await FlutterForegroundTask.isRunningService;
    final devName = (_device?.name ?? '').isNotEmpty ? _device!.name : '장치';

    if (!running) {
      await FlutterForegroundTask.startService(
        // 필요하면 serviceTypes 지정 가능 (Manifest 타입과 함께 사용)
        // serviceTypes: [ForegroundServiceTypes.dataSync],
        notificationTitle: 'SISEON 연결 유지 중',
        notificationText: '$devName 와(과) BLE 연결 유지',
        notificationButtons: const [], // 버튼 필요시 추가
        callback: _bleKeepAliveStartCallback,
      );
    } else {
      await FlutterForegroundTask.updateService(
        notificationTitle: 'SISEON 연결 유지 중',
        notificationText: '$devName 와(과) BLE 연결 유지',
      );
    }
  }

  Future<void> _maybeStopForegroundService() async {
    if (!Platform.isAndroid) return;
    if (!_fgServiceWanted) {
      final running = await FlutterForegroundTask.isRunningService;
      if (running) {
        await FlutterForegroundTask.stopService();
      }
    }
  }

  @override
  void dispose() {
    _connSub?.cancel();
    super.dispose();
  }
}

// 전역 인스턴스
final bleSession = BleSession();
