import 'dart:async';
import 'dart:convert';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import '../services/mqtt_service.dart'; // ✅ MQTT 서비스 import
import '../models/control_mode.dart'; // ✅ ControlMode enum import

class ManualPage extends StatefulWidget {
  const ManualPage({super.key});

  @override
  State<ManualPage> createState() => _ManualPageState();
}

class _ManualPageState extends State<ManualPage> {
  Offset _xzOffset = Offset.zero; // X/Z 제어
  double _yValue = 0.0; // Y 제어
  Timer? _sendTimer;

  @override
  void initState() {
    super.initState();

    // 상단 상태바 투명화 + 아이콘 색상 밝게
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

    // BLE 데이터 전송 시뮬레이션
    _sendTimer = Timer.periodic(const Duration(milliseconds: 200), (_) {
      final controlData = {
        "x": _xzOffset.dx,
        "z": _xzOffset.dy,
        "y": _yValue,
      };
      print("BLE 전송 데이터: ${jsonEncode(controlData)}");
    });
  }

  @override
  void dispose() {
    _sendTimer?.cancel();

    // ✅ 세로모드 복귀 및 Auto 전환 MQTT 발행
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    _publishAutoMode(); // dispose 시에도 Auto 전환

    // 상태바 초기화
    SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
      statusBarColor: Colors.black,
      statusBarIconBrightness: Brightness.light,
    ));

    super.dispose();
  }

  /// ✅ Auto 모드로 전환 + MQTT 발행
  Future<void> _publishAutoMode() async {
    final payload = {
      "profile_id": "1", // 실제 프로필 ID로 교체
      "previous_mode": ControlMode.manual.name,
      "current_mode": ControlMode.auto.name,
    };
    mqttService.publish('/control_mode/1', payload);
    print("🔄 Manual → Auto 전환 (MQTT 발행): $payload");
  }

  /// Deadzone(중립 영역) 적용 함수
  double _applyDeadzone(double value, [double threshold = 0.1]) {
    return value.abs() < threshold ? 0.0 : value;
  }

  /// 🕹️ 공통 조이스틱 카드 UI (Glass 스타일)
  Widget _buildJoystickCard({required Widget joystick}) {
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
    return WillPopScope( // ✅ 시스템 뒤로가기 인터셉트
      onWillPop: () async {
        await _publishAutoMode(); // 뒤로가기 시 Auto 모드 전환
        return true;
      },
      child: Scaffold(
        backgroundColor: const Color(0xFF0F1117),
        body: SafeArea(
          child: Stack(
            children: [
              // 🕹️ X/Z 조이스틱
              Positioned(
                bottom: 40,
                left: 40,
                child: _buildJoystickCard(
                  joystick: Joystick(
                    mode: JoystickMode.all,
                    listener: (details) {
                      setState(() {
                        _xzOffset = Offset(
                          _applyDeadzone(details.x),
                          _applyDeadzone(details.y * -1),
                        );
                      });
                    },
                    onStickDragEnd: () => setState(() => _xzOffset = Offset.zero),
                  ),
                ),
              ),
              // 🕹️ Y 조이스틱
              Positioned(
                bottom: 40,
                right: 40,
                child: _buildJoystickCard(
                  joystick: Joystick(
                    mode: JoystickMode.vertical,
                    listener: (details) {
                      setState(() {
                        _yValue = _applyDeadzone(details.y * -1);
                      });
                    },
                    onStickDragEnd: () => setState(() => _yValue = 0.0),
                  ),
                ),
              ),

              // 🔙 뒤로가기 버튼 (상단 좌측)
              Positioned(
                top: 10,
                left: 10,
                child: IconButton(
                  icon: const Icon(Icons.arrow_back, color: Colors.white, size: 28),
                  onPressed: () async {
                    await _publishAutoMode(); // 버튼 클릭 시도 Auto 전환
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
