import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import '../services/mqtt_service.dart';
import '../models/control_mode.dart';

class ManualPage extends StatefulWidget {
  final BluetoothCharacteristic writableChar; // ✅ BLE 특성 전달

  const ManualPage({super.key, required this.writableChar});

  @override
  State<ManualPage> createState() => _ManualPageState();
}

class _ManualPageState extends State<ManualPage> {
  Offset _xzOffset = Offset.zero; // X/Z 제어
  double _yValue = 0.0; // Y 제어
  List<int> _payload = [127, 127, 127]; // ✅ BLE 전송용 페이로드
  String _debugMessage = '🔌 연결 상태 확인 중...'; // ✅ BLE 상태 디버그 메시지
  Timer? _sendTimer;

  @override
  void initState() {
    super.initState();
    _checkConnection();

    // 상태바 투명화
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ));

    // 3초 뒤 가로모드 전환
    Future.delayed(const Duration(seconds: 3), () {
      SystemChrome.setPreferredOrientations([
        DeviceOrientation.landscapeLeft,
        DeviceOrientation.landscapeRight,
      ]);
    });

    // ✅ 200ms마다 BLE 데이터 전송 (페이로드 기반)
    _sendTimer = Timer.periodic(const Duration(milliseconds: 200), (_) async {
      await _sendPayload();
    });
  }

  /// ✅ BLE 연결 상태 체크
  Future<void> _checkConnection() async {
    final isConnected = await widget.writableChar.device.isConnected;
    setState(() {
      _debugMessage = isConnected
          ? '✅ 연결됨, 조작 대기 중...'
          : '❌ BLE 기기 연결이 끊어졌습니다. 다시 연결해주세요.';
    });
  }

  /// ✅ BLE 데이터 전송
  Future<void> _sendPayload() async {
    try {
      final isConnected = await widget.writableChar.device.isConnected;
      if (!isConnected) {
        setState(() {
          _debugMessage = '❌ 기기 연결이 끊어졌습니다. 다시 연결해주세요.';
        });
        return;
      }
      await widget.writableChar.write(_payload, withoutResponse: false);
      setState(() {
        _debugMessage = '📤 전송됨: ${_payload.join(", ")}';
      });
    } catch (e) {
      setState(() {
        _debugMessage = '❌ 전송 실패: $e';
      });
    }
  }

  /// ✅ XZ 조이스틱 제어 (BLE 데이터 변환)
  void _onJoystickXZ(double x, double y) {
    int bx = ((x + 1) * 127.5).toInt().clamp(0, 255);
    int bz = ((y + 1) * 127.5).toInt().clamp(0, 255);
    _xzOffset = Offset(x, y);
    _payload = [bx, _payload[1], bz];
  }

  /// ✅ Y 조이스틱 제어 (BLE 데이터 변환)
  void _onJoystickY(double y) {
    int by = ((y + 1) * 127.5).toInt().clamp(0, 255);
    _yValue = y;
    _payload = [_payload[0], by, _payload[2]];
  }

  @override
  void dispose() {
    _sendTimer?.cancel();

    // ✅ 세로모드 복귀 및 Auto 전환 MQTT 발행
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    _publishAutoMode();

    // 상태바 초기화
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.black,
      statusBarIconBrightness: Brightness.light,
    ));

    super.dispose();
  }

  /// ✅ Manual 종료 시 Auto 모드로 MQTT 발행
  Future<void> _publishAutoMode() async {
    final payload = {
      "profile_id": "1", // 실제 프로필 ID로 교체
      "previous_mode": ControlMode.manual.name,
      "current_mode": ControlMode.auto.name,
    };
    mqttService.publish('/control_mode/1', payload);
    print("🔄 Manual → Auto 전환 (MQTT 발행): $payload");
  }

  /// Deadzone(중립 영역) 적용
  double _applyDeadzone(double value, [double threshold = 0.1]) {
    return value.abs() < threshold ? 0.0 : value;
  }

  /// 🕹️ 공통 조이스틱 카드 UI (Glass 스타일)
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
              // ✅ BLE 상태 디버그 메시지
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

              // 🕹️ X/Z 조이스틱
              Positioned(
                bottom: 40,
                left: 40,
                child: _buildJoystickCard(
                  Joystick(
                    mode: JoystickMode.all,
                    listener: (details) {
                      double x = _applyDeadzone(details.x);
                      double z = _applyDeadzone(details.y * -1);
                      _onJoystickXZ(x, z);
                    },
                    onStickDragEnd: () => _onJoystickXZ(0, 0),
                  ),
                ),
              ),

              // 🕹️ Y 조이스틱
              Positioned(
                bottom: 40,
                right: 40,
                child: _buildJoystickCard(
                  Joystick(
                    mode: JoystickMode.vertical,
                    listener: (details) {
                      double y = _applyDeadzone(details.y * -1);
                      _onJoystickY(y);
                    },
                    onStickDragEnd: () => _onJoystickY(0),
                  ),
                ),
              ),

              // 🔙 뒤로가기 버튼
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
