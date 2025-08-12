// lib/pages/manual_page.dart
import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import '../models/control_mode.dart';
import '../services/mqtt_service.dart';
import '../services/profile_cache_service.dart';
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

enum PayloadFmt {
  i8x3,    // 3바이트: [X, Y, Z] 각각 int8
  i16x3,   // 6바이트: [X_low, X_high, Y_low, Y_high, Z_low, Z_high]
  i8x4,    // 4바이트: [X, Y, Z, 0] 패딩 추가
  i8x8,    // 8바이트: [X, Y, Z, 0, 0, 0, 0, 0] 긴 패딩
  i8x20    // 20바이트: 표준 BLE MTU 크기
}

class _ManualPageState extends State<ManualPage> {
  List<int> _payload = [0, 0, 0]; // [X, Y, Z] 범위: -100~100
  String _debugMessage = '🔌 연결 상태 확인 중...';
  String? _lastBleError;

  Timer? _sendTimer;
  StreamSubscription<BluetoothConnectionState>? _connectionSub;

  bool _useWriteWithoutResponse = true;
  bool _publishedOnExit = false;
  bool _isConnected = false;
  bool _isWriting = false;

  PayloadFmt _fmt = PayloadFmt.i8x3;
  int _mtuSize = 23; // 기본 BLE MTU

  // 포맷별 시도 순서 (실패 시 다음 포맷으로 자동 전환)
  static const List<PayloadFmt> _formatFallback = [
    PayloadFmt.i8x3,   // 3바이트부터 시작
    PayloadFmt.i8x4,   // 4바이트 시도
    PayloadFmt.i8x8,   // 8바이트 시도
    PayloadFmt.i16x3,  // 6바이트 시도
    PayloadFmt.i8x20,  // 20바이트 마지막 시도
  ];

  int _currentFormatIndex = 0;

  @override
  void initState() {
    super.initState();

    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ));

    _checkCharacteristicCapabilities();
    _requestMtuSize();
    _listenConnectionState();
    _startSending();
  }

  /// MTU 크기 요청 (옵션)
  void _requestMtuSize() async {
    try {
      final mtu = await widget.writableChar.device.requestMtu(512);
      _mtuSize = mtu;
      setState(() {
        _debugMessage = 'ℹ️ MTU 크기: $_mtuSize 바이트';
      });
    } catch (e) {
      // MTU 요청 실패해도 계속 진행
    }
  }

  /// 특성 쓰기 능력 확인
  void _checkCharacteristicCapabilities() {
    final p = widget.writableChar.properties;
    final supportsWrite = p.write || p.writeWithoutResponse;
    _useWriteWithoutResponse = p.writeWithoutResponse;

    if (!supportsWrite) {
      setState(() {
        _debugMessage = '⚠️ 이 특성은 write를 지원하지 않습니다.';
      });
      return;
    }

    setState(() {
      _debugMessage = 'ℹ️ 특성 지원: Write=${p.write}, WriteNR=${p.writeWithoutResponse}';
    });
  }

  void _listenConnectionState() {
    _connectionSub = widget.writableChar.device.state.listen((state) {
      _isConnected = (state == BluetoothConnectionState.connected);

      if (_isConnected) {
        _startSending();
        setState(() {
          _debugMessage = '✅ BLE 연결됨 (MTU=$_mtuSize, NR=$_useWriteWithoutResponse, fmt=$_fmt)';
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

  /// 자동 포맷 전환하며 전송 시도
  Future<void> _sendWithAutoFormat() async {
    final bytes = _encodeAxes(_payload);

    try {
      // 현재 포맷으로 전송 시도
      await _attemptWrite(bytes);

      // 성공 시
      _lastBleError = null;
      setState(() {
        _debugMessage = '📤 전송 성공: ${_payload.join(", ")} → [${bytes.join(", ")}] (${bytes.length}B, fmt=$_fmt)';
      });

    } catch (e) {
      // 길이 오류면 다음 포맷으로 전환 후 재시도
      if (_isLenErr(e)) {
        final success = await _tryNextFormat();
        if (!success) {
          _handleWriteError(e);
        }
      } else {
        _handleWriteError(e);
      }
    }
  }

  /// WriteNR/WR 자동 폴백하며 전송
  Future<void> _attemptWrite(List<int> bytes) async {
    try {
      await widget.writableChar.write(
        bytes,
        withoutResponse: _useWriteWithoutResponse,
      );
    } on PlatformException catch (e) {
      // WriteNR 거부 시 WR로 폴백
      if (_useWriteWithoutResponse && _looksLikeNoNr(e)) {
        await widget.writableChar.write(bytes, withoutResponse: false);
        _useWriteWithoutResponse = false;
        setState(() {
          _debugMessage = '↩️ WriteNR 실패 → Write로 전환';
        });
      } else {
        rethrow;
      }
    }
  }

  /// 다음 포맷으로 전환 시도
  Future<bool> _tryNextFormat() async {
    if (_currentFormatIndex >= _formatFallback.length - 1) {
      // 모든 포맷 시도했지만 실패
      return false;
    }

    _currentFormatIndex++;
    _fmt = _formatFallback[_currentFormatIndex];

    try {
      final newBytes = _encodeAxes(_payload);
      await _attemptWrite(newBytes);

      _lastBleError = null;
      setState(() {
        _debugMessage = '✅ 포맷 전환 성공: $_fmt → [${newBytes.join(", ")}] (${newBytes.length}B)';
      });
      return true;

    } catch (e) {
      // 이 포맷도 실패하면 재귀적으로 다음 포맷 시도
      if (_isLenErr(e)) {
        return await _tryNextFormat();
      } else {
        _handleWriteError(e);
        return false;
      }
    }
  }

  /// 현재 포맷으로 축값을 바이트 배열로 변환
  List<int> _encodeAxes(List<int> axes) {
    final x = axes[0].clamp(-100, 100);
    final y = axes[1].clamp(-100, 100);
    final z = axes[2].clamp(-100, 100);

    switch (_fmt) {
      case PayloadFmt.i8x3:
        return [_toInt8(x), _toInt8(y), _toInt8(z)];

      case PayloadFmt.i8x4:
        return [_toInt8(x), _toInt8(y), _toInt8(z), 0];

      case PayloadFmt.i8x8:
        return [_toInt8(x), _toInt8(y), _toInt8(z), 0, 0, 0, 0, 0];

      case PayloadFmt.i16x3:
      // Little-endian int16
        return [
          x & 0xFF, (x >> 8) & 0xFF,  // X
          y & 0xFF, (y >> 8) & 0xFF,  // Y
          z & 0xFF, (z >> 8) & 0xFF,  // Z
        ];

      case PayloadFmt.i8x20:
        final base = [_toInt8(x), _toInt8(y), _toInt8(z)];
        return base + List.filled(17, 0); // 20바이트까지 패딩
    }
  }

  /// int8 2의 보수 변환 (-100~100 → -100~100 유지, 하위 8비트만)
  int _toInt8(int value) {
    if (value >= 0) {
      return value & 0xFF;
    } else {
      // 음수를 2의 보수로 변환
      return (256 + value) & 0xFF;
    }
  }

  // 에러 판별
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
    setState(() {
      _debugMessage = '❌ 전송 실패: $detail';
    });
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

  // 조이스틱 핸들러들
  void _onJoystickXZ(double x, double z) {
    x = _applyDeadzone(x);
    z = _applyDeadzone(z);
    final xi = (x * 100).round().clamp(-100, 100);
    final zi = (z * 100).round().clamp(-100, 100);
    setState(() {
      _payload[0] = xi;
      _payload[2] = zi;
    });
  }

  void _onJoystickY(double y) {
    y = _applyDeadzone(y);
    final yi = (y * 100).round().clamp(-100, 100);
    setState(() {
      _payload[1] = yi;
    });
  }

  void _resetXZ() => setState(() {
    _payload[0] = 0;
    _payload[2] = 0;
  });

  void _resetY() => setState(() => _payload[1] = 0);

  double _applyDeadzone(double v, [double t = 0.08]) => v.abs() < t ? 0.0 : v;

  Future<void> _publishAutoMode() async {
    try {
      String topic = '/control_mode';
      try {
        final dev = await DeviceCacheService.loadDeviceForProfile(widget.profileId);
        final serial = dev?['serial'];
        if (serial != null && serial.toString().isNotEmpty) {
          topic = '/control_mode/$serial';
        }
      } catch (_) {}

      final payload = {
        'profile_id': widget.profileId.toString(),
        'previous_mode': ControlMode.manual.name,
        'current_mode': ControlMode.auto.name,
      };

      mqttService.publish(topic, payload);
      setState(() {
        _debugMessage = '📶 MQTT 발행 완료 → $topic $payload';
      });
    } catch (e) {
      setState(() {
        _debugMessage = '❌ MQTT 발행 실패: $e';
      });
    }
  }

  @override
  void dispose() {
    _sendTimer?.cancel();
    _connectionSub?.cancel();

    if (!_publishedOnExit) {
      _publishedOnExit = true;
      _publishAutoMode();
    }

    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.black,
      statusBarIconBrightness: Brightness.light,
    ));
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return WillPopScope(
      onWillPop: () async {
        if (!_publishedOnExit) {
          _publishedOnExit = true;
          await _publishAutoMode();
        }
        return true;
      },
      child: Scaffold(
        backgroundColor: const Color(0xFF0D1117),
        body: SafeArea(
          child: Stack(
            children: [
              // 상단 디버그 박스
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

              // 좌: XZ 조이스틱
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

              // 우: Y 조이스틱
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

              // 뒤로가기
              Positioned(
                top: 16,
                left: 16,
                child: IconButton(
                  icon: const Icon(Icons.arrow_back, color: Colors.white, size: 26),
                  onPressed: () async {
                    if (!_publishedOnExit) {
                      _publishedOnExit = true;
                      await _publishAutoMode();
                    }
                    if (context.mounted) Navigator.pop(context);
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