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
  String _debugMessage = '🔌 연결 상태 확인 중...'; // 화면에 표시 안 함
  String? _lastBleError;

  Timer? _sendTimer;
  StreamSubscription<BluetoothConnectionState>? _connectionSub;

  // (1) 무응답 쓰기 강제
  bool _useWriteWithoutResponse = true;
  // (2) MTU
  int _mtuSize = 23;
  // (3) 우선순위 HIGH 요청 여부(조회 API는 없어 요청 사실만 기록)
  bool _priorityHighRequested = false;

  bool _isConnected = false;
  bool _isWriting = false;
  bool _linkBoosted = false; // 연결마다 1회 부스팅 가드
  DateTime _lastCheckLog = DateTime.fromMillisecondsSinceEpoch(0);

  // pop 시 중복 발행 방지용
  bool _isExiting = false;

  PayloadFmt _fmt = PayloadFmt.i8x3;

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
    _checkCharacteristicCapabilities(); // NR 강제
    _requestMtuSize();                  // 초기 MTU 요청
    _listenConnectionState();
    _ensureInitiallyConnected();        // 초기 연결 확인
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

  Future<void> _ensureInitiallyConnected() async {
    try {
      final s = await widget.writableChar.device.state.first
          .timeout(const Duration(seconds: 2));
      if (s != BluetoothConnectionState.connected) {
        _notifyAndExitWithFix('연결 실패: 다시 연결해 주세요');
      }
    } catch (_) {
      _notifyAndExitWithFix('연결 실패: 다시 연결해 주세요');
    }
  }

  // (2) MTU 크게
  void _requestMtuSize() async {
    try {
      final mtu = await widget.writableChar.device.requestMtu(512);
      _mtuSize = mtu;
      setState(() => _debugMessage = 'ℹ️ MTU 크기: $_mtuSize 바이트');
    } catch (_) {}
  }

  // (1) NR 강제: 리모트가 NR 미지원이면 이후 write 시 예외 → 안내 후 종료
  void _checkCharacteristicCapabilities() {
    final p = widget.writableChar.properties;
    _useWriteWithoutResponse = true; // 항상 NR로 보냄

    final supportsNR = p.writeWithoutResponse;
    setState(() {
      _debugMessage = supportsNR
          ? 'ℹ️ 강제 NR 모드 (리모트 NR 지원)'
          : '⚠️ 강제 NR 모드 (리모트 NR 미지원일 수 있음)';
    });
  }

  // (3) 연결 우선순위 HIGH + MTU 재요청을 연결 시 마다 보장
  Future<void> _boostLinkIfPossible() async {
    if (!_isConnected) return;
    // 우선순위 HIGH
    try {
      await widget.writableChar.device.requestConnectionPriority(
        connectionPriorityRequest: ConnectionPriority.high,
      );
      _priorityHighRequested = true;
    } catch (_) {}
    // MTU 재요청 (일부 단말은 연결 후에만 반영되는 경우가 있어 재요청)
    try {
      final mtu = await widget.writableChar.device.requestMtu(512);
      _mtuSize = mtu;
    } catch (_) {}
    _linkBoosted = true;
  }

  void _listenConnectionState() {
    _connectionSub = widget.writableChar.device.state.listen((state) async {
      _isConnected = (state == BluetoothConnectionState.connected);

      if (_isConnected) {
        _linkBoosted = false;
        await _boostLinkIfPossible();
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
        _notifyAndExitWithFix('연결 실패: 다시 연결해 주세요');
      } else {
        setState(() {
          _debugMessage = 'ℹ️ BLE 상태: $state';
        });
      }
    });
  }

  void _startSending() {
    if (_sendTimer != null) return;
    _sendTimer = Timer.periodic(const Duration(milliseconds: 100), (_) async {
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
    // 연결 성능 튜닝이 아직이면 보장
    if (!_linkBoosted) {
      await _boostLinkIfPossible();
    }
    // 1초마다 상태 체크 로그
    final nowForCheck = DateTime.now();
    if (nowForCheck.difference(_lastCheckLog).inMilliseconds > 1000) {
      _lastCheckLog = nowForCheck;
      debugPrint('BLE LINK CHECK → NR=true, MTU=$_mtuSize, Prio=HIGH(${_priorityHighRequested ? "요청됨" : "미요청"})');
    }

    // ✅ 현재 시각(UTC)
    final nowLocal = DateTime.now();
    final nowUtc = nowLocal.toUtc();

    // 축 값
    final body = _encodeAxes(_payload);

    // 타임스탬프 헤더(7B)
    final hdr = _timestampHeader(nowUtc);

    // 최종 패킷
    final bytes = [...hdr, ...body];

    try {
      await _attemptWrite(bytes);
      _lastBleError = null;

      // 로그
      final hhmm = _fmtLocalHmsMs(nowLocal);
      final secs = nowUtc.millisecondsSinceEpoch ~/ 1000;
      final ms = nowUtc.millisecond;
      final xyzStr = _xyzString(body);
      debugPrint('BLE SEND @ $hhmm | utc=${secs}s+${ms}ms | $xyzStr | len=${bytes.length}B | fmt=$_fmt');
    } catch (e) {
      if (_isLenErr(e)) {
        final success = await _tryNextFormat();
        if (!success) _handleWriteError(e);
      } else {
        _handleWriteError(e);
      }
    }
  }

  // ✅ NR만 사용. 거부되면 안내 후 종료(폴백 없음)
  Future<void> _attemptWrite(List<int> bytes) async {
    try {
      await widget.writableChar.write(bytes, withoutResponse: true);
    } on PlatformException catch (e) {
      if (_looksLikeNoNr(e)) {
        _lastBleError = e.toString();
        await _notifyAndExitWithFix('연결 실패: 기기가 write-without-response 를 지원하지 않습니다');
        return;
      }
      rethrow;
    }
  }

  Future<bool> _tryNextFormat() async {
    if (_currentFormatIndex >= _formatFallback.length - 1) return false;
    _currentFormatIndex++;
    _fmt = _formatFallback[_currentFormatIndex];

    try {
      final nowLocal = DateTime.now();
      final nowUtc = nowLocal.toUtc();
      final body = _encodeAxes(_payload);
      final hdr = _timestampHeader(nowUtc);
      final newBytes = [...hdr, ...body];

      await _attemptWrite(newBytes);
      _lastBleError = null;

      final hhmm = _fmtLocalHmsMs(nowLocal);
      final secs = nowUtc.millisecondsSinceEpoch ~/ 1000;
      final ms = nowUtc.millisecond;
      final xyzStr = _xyzString(body);
      debugPrint('BLE SEND (fallback fmt=$_fmt) @ $hhmm | utc=${secs}s+${ms}ms | $xyzStr | len=${newBytes.length}B');

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

  // ---------- 타임스탬프 헤더(7B) ----------
  /// 7B 헤더: [version=0x01][epoch_sec LE 4B][msec LE 2B]
  List<int> _timestampHeader(DateTime nowUtc) {
    final secs = nowUtc.millisecondsSinceEpoch ~/ 1000; // 32-bit
    final ms = nowUtc.millisecond; // 0..999
    return [
      0x01, // version
      secs & 0xFF,
      (secs >> 8) & 0xFF,
      (secs >> 16) & 0xFF,
      (secs >> 24) & 0xFF,
      ms & 0xFF,
      (ms >> 8) & 0xFF,
    ];
  }

  // ---------- 축 데이터 인코딩 ----------
  // 범위 -127 ~ 127 유지
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

  // ---------- 유틸(로그 포맷) ----------
  int _toInt8(int value) => value >= 0 ? (value & 0xFF) : ((256 + value) & 0xFF);

  String _fmtLocalHmsMs(DateTime t) {
    String two(int v) => v.toString().padLeft(2, '0');
    final ms = t.millisecond.toString().padLeft(3, '0');
    return '${two(t.hour)}:${two(t.minute)}:${two(t.second)}.$ms';
  }

  String _xyzString(List<int> body) {
    if (_fmt == PayloadFmt.i16x3) {
      final x = body[0] | (body[1] << 8);
      final y = body[2] | (body[3] << 8);
      final z = body[4] | (body[5] << 8);
      return 'xyz=($x,$y,$z)';
    } else {
      final x = body[0];
      final y = body[1];
      final z = body[2];
      return 'xyz=($x,$y,$z)';
    }
  }

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
    _debugMessage = '❌ 전송 실패: $detail';

    final lc = detail.toLowerCase();
    if (lc.contains('not connected') ||
        lc.contains('gatt') && (lc.contains('133') || lc.contains('62') || lc.contains('8') || lc.contains('19') || lc.contains('22'))) {
      _notifyAndExitWithFix('연결 실패: 다시 연결해 주세요');
    }
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

  // 🔁 뒤로가기 시 FIX 모드로 전환
  Future<void> _exitWithFix() async {
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
          'current_mode': ControlMode.fix.name,
        };
        mqttService.publish(topic, payload);
        _debugMessage = '📶 MQTT 발행 완료 → $topic $payload';
      } else {
        _debugMessage = '❌ MQTT 미발행: deviceSerial 없음';
      }
    } finally {
      if (mounted) {
        Navigator.pop(context, ControlMode.fix);
      }
    }
  }

  // ✅ 연결 실패/끊김 안내 후 FIX로 종료
  Future<void> _notifyAndExitWithFix(String message) async {
    if (_isExiting || !mounted) return;
    final messenger = ScaffoldMessenger.of(context);
    messenger.hideCurrentSnackBar();
    messenger.showSnackBar(
      SnackBar(
        content: Text(message),
        behavior: SnackBarBehavior.floating,
        backgroundColor: Colors.redAccent,
        duration: const Duration(seconds: 2),
      ),
    );
    await Future.delayed(const Duration(milliseconds: 1600));
    await _exitWithFix();
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
        await _exitWithFix();
      },
      child: Scaffold(
        backgroundColor: const Color(0xFF0D1117),
        body: SafeArea(
          child: Stack(
            children: [
              // 좌측 XZ 조이스틱
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

              // 우측 Y 조이스틱
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

              // 뒤로가기(항상 수동 FIX 종료)
              Positioned(
                top: 16,
                left: 16,
                child: IconButton(
                  icon: const Icon(Icons.arrow_back, color: Colors.white, size: 26),
                  onPressed: () async {
                    await _exitWithFix();
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
