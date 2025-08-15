// lib/pages/manual_page.dart
import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import '../models/control_mode.dart';
import '../services/mqtt_service.dart';
import '../services/device_cache_service.dart';

class ManualPage extends StatefulWidget {
  final BluetoothCharacteristic writableChar;
  final int profileId;

  const ManualPage({
    Key? key,
    required this.writableChar,
    required this.profileId,
  }) : super(key: key);

  @override
  State<ManualPage> createState() => _ManualPageState();
}

enum PayloadFmt { i8x3, i16x3, i8x4, i8x8, i8x20 }

class _ManualPageState extends State<ManualPage> {
  // ── 상태
  List<int> _payload = [0, 0, 0];
  String _debugMessage = '🔌 연결 상태 확인 중...';
  String? _lastBleError;

  Timer? _sendTimer;
  StreamSubscription<BluetoothConnectionState>? _connectionSub;

  bool _useWriteWithoutResponse = true;
  bool _isConnected = false;
  bool _isWriting = false;

  // pop 시 중복 발행 방지용
  bool _isExiting = false;

  PayloadFmt _fmt = PayloadFmt.i8x3;
  int _mtuSize = 23;

  String? _deviceSerial;
  bool _resolvingSerial = false;

  static const List<PayloadFmt> _formatFallback = [
    PayloadFmt.i8x3,
    PayloadFmt.i8x4,
    PayloadFmt.i8x8,
    PayloadFmt.i16x3,
    PayloadFmt.i8x20,
  ];
  int _currentFormatIndex = 0;

  @override
  void initState() {
    super.initState();

    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ));

    mqttService.connect();

    _resolveDeviceSerial();
    _checkCharacteristicCapabilities();
    _requestMtuSize();
    _listenConnectionState();
    _startSending();
  }

  // ── Device Serial 확보: 캐시 → 서버 → BLE ID
  Future<void> _resolveDeviceSerial() async {
    if (_resolvingSerial) return;
    _resolvingSerial = true;

    String? serial;

    try {
      final dev = await DeviceCacheService.loadDeviceForProfile(widget.profileId);
      serial = dev?['serial']?.toString();
    } catch (_) {}

    if (serial == null || serial.isEmpty) {
      try {
        await DeviceCacheService.fetchAndCacheDevice(profileId: widget.profileId);
        final dev2 = await DeviceCacheService.loadDeviceForProfile(widget.profileId);
        serial = dev2?['serial']?.toString();
      } catch (_) {}
    }

    if (serial == null || serial.isEmpty) {
      serial = _bleFallbackId();
    }

    if (!mounted) return;
    setState(() {
      _deviceSerial = (serial != null && serial.isNotEmpty) ? serial : null;
      _resolvingSerial = false;
      _debugMessage = (_deviceSerial != null)
          ? '🔗 DeviceSerial: $_deviceSerial'
          : '⚠️ DeviceSerial을 찾지 못했습니다. (캐시/서버/BLE 확인 필요)';
    });
  }

  String _bleFallbackId() {
    try {
      return widget.writableChar.device.id.str;
    } catch (_) {
      try {
        // ignore: deprecated_member_use
        return widget.writableChar.device.remoteId.str;
      } catch (_) {
        try {
          // ignore: deprecated_member_use
          return widget.writableChar.remoteId.str;
        } catch (_) {
          try {
            return widget.writableChar.device.id.toString();
          } catch (_) {
            return '';
          }
        }
      }
    }
  }

  void _requestMtuSize() async {
    try {
      final mtu = await widget.writableChar.device.requestMtu(512);
      _mtuSize = mtu;
      setState(() => _debugMessage = 'ℹ️ MTU 크기: $_mtuSize 바이트');
    } catch (_) {}
  }

  void _checkCharacteristicCapabilities() {
    final p = widget.writableChar.properties;
    final supportsWrite = p.write || p.writeWithoutResponse;
    _useWriteWithoutResponse = p.writeWithoutResponse;

    setState(() {
      _debugMessage = supportsWrite
          ? 'ℹ️ 특성 지원: Write=${p.write}, WriteNR=${p.writeWithoutResponse}'
          : '⚠️ 이 특성은 write를 지원하지 않습니다.';
    });
  }

  void _listenConnectionState() {
    _connectionSub = widget.writableChar.device.state.listen((state) {
      _isConnected = (state == BluetoothConnectionState.connected);

      if (_isConnected) {
        _startSending();
        setState(() {
          _debugMessage =
          '✅ BLE 연결됨 (MTU=$_mtuSize, NR=$_useWriteWithoutResponse, fmt=$_fmt)';
        });
      } else if (state == BluetoothConnectionState.disconnected) {
        _sendTimer?.cancel();
        _sendTimer = null;
        final extra = (_lastBleError != null) ? '\n최근 오류: $_lastBleError' : '';
        setState(() {
          _debugMessage = '❌ BLE 연결 끊김 ($state)$extra';
        });
      } else {
        setState(() {
          _debugMessage = 'ℹ️ BLE 상태: $state';
        });
      }
    });
  }

  void _startSending() {
    if (_sendTimer != null) return;
    _sendTimer = Timer.periodic(const Duration(milliseconds: 200), (_) async {
      if (!_isConnected || _isWriting) return;
      _isWriting = true;
      try {
        await _sendWithAutoFormat();
      } finally {
        _isWriting = false;
      }
    });
  }

  Future<void> _sendWithAutoFormat() async {
    final bytes = _encodeAxes(_payload);
    try {
      await _attemptWrite(bytes);
      _lastBleError = null;
      setState(() {
        _debugMessage =
        '📤 전송 성공: ${_payload.join(", ")} → [${bytes.join(", ")}] (${bytes.length}B, fmt=$_fmt)';
      });
    } catch (e) {
      if (_isLenErr(e)) {
        final success = await _tryNextFormat();
        if (!success) _handleWriteError(e);
      } else {
        _handleWriteError(e);
      }
    }
  }

  Future<void> _attemptWrite(List<int> bytes) async {
    try {
      await widget.writableChar.write(bytes, withoutResponse: _useWriteWithoutResponse);
    } on PlatformException catch (e) {
      if (_useWriteWithoutResponse && _looksLikeNoNr(e)) {
        await widget.writableChar.write(bytes, withoutResponse: false);
        _useWriteWithoutResponse = false;
        setState(() => _debugMessage = '↩️ WriteNR 실패 → Write로 전환');
      } else {
        rethrow;
      }
    }
  }

  Future<bool> _tryNextFormat() async {
    if (_currentFormatIndex >= _formatFallback.length - 1) return false;
    _currentFormatIndex++;
    _fmt = _formatFallback[_currentFormatIndex];

    try {
      final newBytes = _encodeAxes(_payload);
      await _attemptWrite(newBytes);
      _lastBleError = null;
      setState(() {
        _debugMessage =
        '✅ 포맷 전환 성공: $_fmt → [${newBytes.join(", ")}] (${newBytes.length}B)';
      });
      return true;
    } catch (e) {
      if (_isLenErr(e)) {
        return await _tryNextFormat();
      } else {
        _handleWriteError(e);
        return false;
      }
    }
  }

  // 여기 범위를 -127 ~ 127로 변경
  List<int> _encodeAxes(List<int> axes) {
    final x = axes[0].clamp(-127, 127);
    final y = axes[1].clamp(-127, 127);
    final z = axes[2].clamp(-127, 127);

    switch (_fmt) {
      case PayloadFmt.i8x3:
        return [_toInt8(x), _toInt8(y), _toInt8(z)];
      case PayloadFmt.i8x4:
        return [_toInt8(x), _toInt8(y), _toInt8(z), 0];
      case PayloadFmt.i8x8:
        return [_toInt8(x), _toInt8(y), _toInt8(z), 0, 0, 0, 0, 0];
      case PayloadFmt.i16x3:
        return [
          x & 0xFF, (x >> 8) & 0xFF,
          y & 0xFF, (y >> 8) & 0xFF,
          z & 0xFF, (z >> 8) & 0xFF,
        ];
      case PayloadFmt.i8x20:
        final base = [_toInt8(x), _toInt8(y), _toInt8(z)];
        return base + List.filled(17, 0);
    }
  }

  int _toInt8(int value) => value >= 0 ? (value & 0xFF) : ((256 + value) & 0xFF);

  bool _isLenErr(Object e) {
    final s = '$e'.toLowerCase();
    return s.contains('invalid_attribute_length') ||
        s.contains('code=13') ||
        s.contains('gatt_invalid_attribute_length');
  }

  bool _looksLikeNoNr(PlatformException e) {
    final s = '${e.code} ${e.message} ${e.details}'.toLowerCase();
    return s.contains('withoutresponse') ||
        s.contains('no write without response') ||
        s.contains('write not permitted') ||
        s.contains('gatt write not permitted') ||
        s.contains('request not supported');
  }

  void _handleWriteError(Object e) {
    final detail = _bleErrorDetail(e);
    _lastBleError = detail;
    setState(() => _debugMessage = '❌ 전송 실패: $detail');
  }

  String _bleErrorDetail(Object e) {
    try {
      final d = e as dynamic;
      final code = d.errorCode ?? d.code;
      final desc = d.description ?? d.message ?? e.toString();
      return (code != null) ? 'code=$code, $desc' : desc.toString();
    } catch (_) {
      return e.toString();
    }
  }

  // ── 조이스틱 핸들러 (-127 ~ 127 적용)
  void _onJoystickXZ(double x, double z) {
    x = _applyDeadzone(x);
    z = _applyDeadzone(z);
    final xi = (x * 127).round().clamp(-127, 127);
    final zi = (z * 127).round().clamp(-127, 127);
    setState(() {
      _payload[0] = xi;
      _payload[2] = zi;
    });
  }

  void _onJoystickY(double y) {
    y = _applyDeadzone(y);
    final yi = (y * 127).round().clamp(-127, 127);
    setState(() => _payload[1] = yi);
  }

  void _resetXZ() => setState(() {
    _payload[0] = 0;
    _payload[2] = 0;
  });

  void _resetY() => setState(() => _payload[1] = 0);

  double _applyDeadzone(double v, [double t = 0.08]) => v.abs() < t ? 0.0 : v;

  Future<void> _exitWithAuto() async {
    if (_isExiting) return;
    _isExiting = true;

    try {
      if (_deviceSerial == null || _deviceSerial!.isEmpty) {
        await _resolveDeviceSerial();
      }

      if (_deviceSerial != null && _deviceSerial!.isNotEmpty) {
        final topic = '/control_mode/${_deviceSerial!}';
        final payload = {
          'profile_id': widget.profileId.toString(),
          'previous_mode': ControlMode.manual.name,
          'current_mode': ControlMode.auto.name,
        };
        mqttService.publish(topic, payload);
        setState(() => _debugMessage = '📶 MQTT 발행 완료 → $topic $payload');
      } else {
        setState(() => _debugMessage = '❌ MQTT 미발행: deviceSerial 없음');
      }
    } finally {
      if (mounted) {
        Navigator.pop(context, ControlMode.auto);
      }
    }
  }

  @override
  void dispose() {
    _sendTimer?.cancel();
    _connectionSub?.cancel();
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.black,
      statusBarIconBrightness: Brightness.light,
    ));
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return PopScope(
      canPop: false,
      onPopInvoked: (didPop) async {
        if (didPop) return;
        await _exitWithAuto();
      },
      child: Scaffold(
        backgroundColor: const Color(0xFF0D1117),
        body: SafeArea(
          child: Stack(
            children: [
              Positioned(
                top: 16,
                left: 16,
                right: 16,
                child: Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.55),
                    borderRadius: BorderRadius.circular(10),
                    border: Border.all(color: Colors.white24, width: 1),
                  ),
                  child: Text(
                    _debugMessage,
                    style: const TextStyle(color: Colors.white, fontSize: 13.5),
                  ),
                ),
              ),
              Positioned(
                left: 28,
                bottom: 28,
                child: _glassJoystick(
                  Joystick(
                    mode: JoystickMode.all,
                    listener: (d) => _onJoystickXZ(d.x, -d.y),
                    onStickDragEnd: _resetXZ,
                  ),
                ),
              ),
              Positioned(
                right: 28,
                bottom: 28,
                child: _glassJoystick(
                  Joystick(
                    mode: JoystickMode.vertical,
                    listener: (d) => _onJoystickY(-d.y),
                    onStickDragEnd: _resetY,
                  ),
                ),
              ),
              Positioned(
                top: 16,
                left: 16,
                child: IconButton(
                  icon: const Icon(Icons.arrow_back, color: Colors.white, size: 26),
                  onPressed: () async {
                    await _exitWithAuto();
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _glassJoystick(Widget joystick) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(22),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 12, sigmaY: 12),
        child: Container(
          width: 200,
          height: 200,
          padding: const EdgeInsets.all(18),
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.06),
            borderRadius: BorderRadius.circular(22),
            border: Border.all(color: Colors.white24),
          ),
          child: joystick,
        ),
      ),
    );
  }
}
