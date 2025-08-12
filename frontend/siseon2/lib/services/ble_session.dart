// // lib/services/ble_session.dart
// import 'dart:async';
// import 'package:flutter/foundation.dart';
// import 'package:flutter_blue_plus/flutter_blue_plus.dart';
//
// class BleSession {
//   static BluetoothDevice? device;
//   static BluetoothCharacteristic? char;
//
//   /// 전역적으로 자동 재연결을 잠시 중단하고 싶을 때 true
//   static bool suspendAutoReconnect = false;
//
//   /// 로그 켜기/끄기
//   static bool enableDebugLog = true;
//
//   /// 연결 상태를 앱 전역에서 구독하고 싶을 때 쓰는 Notifier
//   static final ValueNotifier<bool> isConnected = ValueNotifier<bool>(false);
//
//   static StreamSubscription<BluetoothConnectionState>? _connSub;
//
//   static Future<void> set(BluetoothDevice d, BluetoothCharacteristic c) async {
//     device = d;
//     char = c;
//     await _attachConnListener(d);
//     _log('set: ${d.id.id}, char=${c.uuid}');
//   }
//
//   static void clear() {
//     _connSub?.cancel();
//     _connSub = null;
//     isConnected.value = false;
//     _log('clear()');
//     device = null;
//     char = null;
//   }
//
//   /// 소프트 해제: 연결만 끊고 레퍼런스는 유지 (돌아와 재부착 가능)
//   static Future<void> softDisconnect() async {
//     try {
//       await device?.disconnect();
//       _log('softDisconnect() -> ok');
//     } catch (e) {
//       _log('softDisconnect() -> $e');
//     }
//     // 레퍼런스는 유지 (clear 안함)
//   }
//
//   /// 하드 해제: 연결 끊고 전역 캐시도 비움 (로그아웃/탈퇴용)
//   static Future<void> hardDisconnectAndClear() async {
//     try {
//       await device?.disconnect();
//       _log('hardDisconnectAndClear() -> disconnected');
//     } catch (e) {
//       _log('hardDisconnectAndClear() -> $e');
//     }
//     clear();
//   }
//
//   /// 화면 진입/전환 시 한 줄로 상태 찍을 수 있는 헬퍼
//   static Future<void> debugPing(String where) async {
//     if (!enableDebugLog) return;
//     if (device == null) {
//       _log('[$where] device=null (연결 안 됨)');
//       return;
//     }
//     try {
//       final s = await device!.state.first;
//       _log('[$where] ${device!.id.id} -> $s (char=${char?.uuid})');
//     } catch (e) {
//       _log('[$where] 상태 조회 실패: $e');
//     }
//   }
//
//   /// 내부: 연결상태 리스너 붙이기
//   static Future<void> _attachConnListener(BluetoothDevice d) async {
//     _connSub?.cancel();
//     _connSub = d.connectionState.listen((s) {
//       final on = s == BluetoothConnectionState.connected;
//       isConnected.value = on;
//       _log('[connState] ${d.id.id} -> $s');
//     });
//
//     // 초깃값 세팅
//     try {
//       final s = await d.state.first;
//       isConnected.value = s == BluetoothConnectionState.connected;
//       _log('initial state ${d.id.id} -> $s');
//     } catch (e) {
//       _log('initial state read failed: $e');
//     }
//   }
//
//   static void _log(String msg) {
//     if (!enableDebugLog) return;
//     // print는 바로 터미널(런 콘솔)로 출력됨
//     print('[BLE] $msg');
//   }
// }
