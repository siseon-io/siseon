// lib/services/ble_session.dart
import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class BleSession extends ChangeNotifier {
  BluetoothDevice? _device;
  BluetoothCharacteristic? _char;
  StreamSubscription<BluetoothConnectionState>? _sub;

  BluetoothDevice? get device => _device;
  BluetoothCharacteristic? get char => _char;
  bool get isConnected => _device != null && _char != null;

  Future<void> setConnected(BluetoothDevice d, BluetoothCharacteristic c) async {
    _device = d;
    _char = c;

    await _sub?.cancel();
    _sub = d.state.listen((s) {
      if (s == BluetoothConnectionState.disconnected) {
        _device = null;
        _char = null;
        notifyListeners();
      }
    });

    notifyListeners();
  }

  Future<void> disconnect() async {
    try { await _device?.disconnect(); } catch (_) {}
    await _sub?.cancel();
    _sub = null;
    _device = null;
    _char = null;
    notifyListeners();
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }
}

// 전역 인스턴스
final bleSession = BleSession();
