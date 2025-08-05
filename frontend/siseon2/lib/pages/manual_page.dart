  import 'dart:async';
  import 'dart:convert';
  import 'dart:ui';
  import 'package:flutter/material.dart';
  import 'package:flutter/services.dart';
  import 'package:flutter_joystick/flutter_joystick.dart';

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

      // 세로모드 복귀 및 상태바 초기화
      SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
      SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle(
        statusBarColor: Colors.black,
        statusBarIconBrightness: Brightness.light,
      ));

      super.dispose();
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
              color: Colors.white.withOpacity(0.05), // Glass 효과 배경
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
      return Scaffold(
        backgroundColor: const Color(0xFF0F1117), // 다크 모던 배경
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
                    await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
                    if (context.mounted) Navigator.pop(context);
                  },
                ),
              ),
            ],
          ),
        ),
      );
    }
  }
