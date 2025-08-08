import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import '../services/mqtt_service.dart';
import '../models/control_mode.dart';

class ManualPage extends StatefulWidget {
  final BluetoothCharacteristic writableChar;

  const ManualPage({Key? key, required this.writableChar}) : super(key: key);

  @override
  State<ManualPage> createState() => _ManualPageState();
}

class _ManualPageState extends State<ManualPage> {
  List<int> _payload = [0, 0, 0];
  String _debugMessage = '🔌 연결 상태 확인 중...';
  Timer? _sendTimer;
  StreamSubscription<BluetoothConnectionState>? _connectionSub;

  @override
  void initState() {
    super.initState();

    // 상태바 투명화
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ));

    // 가로모드 고정 (3초 뒤)
    Future.delayed(const Duration(seconds: 3), () {
      SystemChrome.setPreferredOrientations([
        DeviceOrientation.landscapeLeft,
        DeviceOrientation.landscapeRight,
      ]);
    });

    // 최초 연결
    _connectDevice();

    // BLE 상태 구독: 연결 시 타이머 시작, 끊기면 타이머 취소 + 재연결 시도
    _connectionSub = widget.writableChar.device.state.listen((state) {
      if (state == BluetoothConnectionState.connected) {
        if (_sendTimer == null) {
          _sendTimer = Timer.periodic(
            const Duration(milliseconds: 300),
                (_) => _sendPayload(),
          );
        }
        setState(() => _debugMessage = '✅ BLE 연결됨, 조작 대기 중...');
      } else if (state == BluetoothConnectionState.disconnected) {
        _sendTimer?.cancel();
        _sendTimer = null;
        setState(() => _debugMessage = '❌ BLE 연결 끊어짐, 재연결 시도 중...');
        Future.delayed(const Duration(seconds: 1), () {
          if (mounted) _connectDevice();
        });
      }
    });
  }

  Future<void> _connectDevice() async {
    try {
      await widget.writableChar.device.connect(autoConnect: false);
      await widget.writableChar.device.discoverServices();
    } catch (e) {
      setState(() => _debugMessage = '❌ BLE 연결 실패 또는 재연결 오류: $e');
    }
  }

  @override
  void dispose() {
    _sendTimer?.cancel();
    _connectionSub?.cancel();
    // dispose 시 자동 disconnect 제거: 연결 유지하여 재진입 시 바로 사용 가능

    // 세로모드 복귀 및 Auto 모드 MQTT 발행
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    _publishAutoMode();

    // 상태바 원래대로
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.black,
      statusBarIconBrightness: Brightness.light,
    ));

    super.dispose();
  }

  Future<void> _sendPayload() async {
    try {
      await widget.writableChar
          .write(_payload, withoutResponse: false);
      setState(() => _debugMessage = '📤 BLE 전송: ${_payload.join(', ')}');
    } catch (e) {
      setState(() => _debugMessage = '❌ BLE 전송 실패: $e');
    } finally {
      _payload = [0, 0, 0];
    }
  }

  void _onJoystickXZ(double x, double y) {
    x = _applyDeadzone(x);
    y = _applyDeadzone(y);
    _payload[0] = ((x * 127).round() & 0xFF);
    _payload[2] = ((y * 127).round() & 0xFF);
  }

  void _onJoystickY(double y) {
    y = _applyDeadzone(y);
    _payload[1] = ((y * 127).round() & 0xFF);
  }

  Future<void> _publishAutoMode() async {
    final payload = {
      'profile_id': '1',
      'previous_mode': ControlMode.manual.name,
      'current_mode': ControlMode.auto.name,
    };
    try {
      mqttService.publish('/control_mode/1', payload);
      setState(() => _debugMessage = '📶 MQTT 발행 완료: $payload');
    } catch (e) {
      setState(() => _debugMessage = '❌ MQTT 발행 실패: $e');
    }
  }

  double _applyDeadzone(double value, [double threshold = 0.1]) =>
      value.abs() < threshold ? 0.0 : value;

  Widget _buildJoystickCard(Widget joystick) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(28),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: 15, sigmaY: 15),
        child: Container(
          width: 200,
          height: 200,
          padding: const EdgeInsets.all(20),
          decoration: BoxDecoration(
            color: Colors.white.withOpacity(0.05),
            borderRadius: BorderRadius.circular(28),
            border: Border.all(color: Colors.white.withOpacity(0.15)),
            boxShadow: [
              BoxShadow(
                color: Colors.black.withOpacity(0.4),
                blurRadius: 12,
                offset: const Offset(0, 6),
              ),
            ],
          ),
          child: joystick,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return WillPopScope(
      onWillPop: () async {
        await _publishAutoMode();
        return true;
      },
      child: Scaffold(
        backgroundColor: const Color(0xFF0F1117),
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
                    color: Colors.black.withOpacity(0.6),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Text(
                    _debugMessage,
                    style: const TextStyle(color: Colors.white, fontSize: 14),
                  ),
                ),
              ),
              Positioned(
                bottom: 40,
                left: 40,
                child: _buildJoystickCard(
                  Joystick(
                    mode: JoystickMode.all,
                    listener: (details) {_onJoystickXZ(details.x, -details.y);},
                    onStickDragEnd: () => _onJoystickXZ(0, 0),
                  ),
                ),
              ),
              Positioned(
                bottom: 40,
                right: 40,
                child: _buildJoystickCard(
                  Joystick(
                    mode: JoystickMode.vertical,
                    listener: (details) {_onJoystickY(-details.y);},
                    onStickDragEnd: () => _onJoystickY(0),
                  ),
                ),
              ),
              Positioned(
                top: 10,
                left: 10,
                child: IconButton(
                  icon: const Icon(Icons.arrow_back, color: Colors.white, size: 28),
                  onPressed: () async {
                    await _publishAutoMode();
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
}