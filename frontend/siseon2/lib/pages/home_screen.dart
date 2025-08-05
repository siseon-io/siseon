import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:app_settings/app_settings.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:http/http.dart' as http;
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/preset_service.dart';
import 'package:siseon2/pages/settings/preset_page.dart';
import 'package:siseon2/pages/ble_scan_screen.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/models/control_mode.dart';

class HomeScreen extends StatefulWidget {
  final VoidCallback onAiModeSwitch; // RootScreen에서 콜백 전달받음

  const HomeScreen({super.key, required this.onAiModeSwitch});

  @override
  HomeScreenState createState() => HomeScreenState();
}

class HomeScreenState extends State<HomeScreen> {
  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];
  bool _isBluetoothOn = false;
  ControlMode _currentMode = ControlMode.auto; // 초기 모드: auto

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color headerGrey = Color(0xFF161B22);
  static const Color cardGrey = Color(0xFF1E293B);
  static const Color textWhite = Colors.white;
  static const Color textGrey = Colors.white70;

  @override
  void initState() {
    super.initState();
    _loadProfileAndPresets();
    _checkBluetoothState();
  }

  /// ✅ 블루투스 상태 체크
  Future<void> _checkBluetoothState() async {
    final isOn = await FlutterBluePlus.isOn;
    setState(() => _isBluetoothOn = isOn);
  }

  /// ✅ BLE 권한 요청 및 스캔 화면 이동
  Future<void> _handleDisconnectAndScan() async {
    final bluetooth = await Permission.bluetoothConnect.request();
    final scan = await Permission.bluetoothScan.request();
    final location = await Permission.location.request();

    final allGranted = bluetooth.isGranted && scan.isGranted && location.isGranted;
    if (!allGranted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ BLE 권한이 필요합니다')),
      );
      return;
    }

    final isOn = await FlutterBluePlus.isOn;
    if (!isOn) {
      await FlutterBluePlus.turnOn();
      await AppSettings.openAppSettings();
      return;
    }

    if (!mounted) return;
    Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => const BleScanScreen()),
    );
  }

  /// ✅ 프로필 및 프리셋 불러오기
  Future<void> _loadProfileAndPresets() async {
    final profile = await ProfileCacheService.loadProfile();
    if (profile == null) return;

    final profileId = profile['id'];
    final presets = await PresetService.fetchPresets(profileId);

    setState(() {
      _profile = profile;
      _presets = presets.take(3).toList();
    });
  }

  /// ✅ MQTT 발행
  void _publishMode(ControlMode previous, ControlMode current) {
    if (_profile == null) return;

    final profileId = _profile!['id'].toString();
    final payload = {
      "profile_id": profileId,
      "previous_mode": previous.name,
      "current_mode": current.name,
    };

    mqttService.publish('/control_mode/1', payload);
    print("📤 MQTT 발행: $payload");
  }

  /// ✅ RootScreen에서 호출하는 AI 모드 전환
  void switchToAiMode() {
    if (_profile == null) return;

    final prevMode = _currentMode;
    setState(() {
      _currentMode = ControlMode.auto;
    });

    _publishMode(prevMode, ControlMode.auto);
    widget.onAiModeSwitch(); // RootScreen 콜백 실행
  }

  /// ✅ 프리셋 선택 시 API & MQTT 발행
  Future<void> _handlePresetSelect(int presetId) async {
    if (_profile == null) return;
    final profileId = _profile!['id'];
    final prevMode = _currentMode;

    try {
      // API 요청
      final token = await AuthService.getValidAccessToken();
      final response = await http.post(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/preset-coordinate'),
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
        body: jsonEncode({
          "profile_id": profileId,
          "preset_id": presetId,
        }),
      );

      if (response.statusCode == 200) {
        print("✅ 프리셋 좌표 API 호출 성공");
      } else {
        print("❌ 프리셋 좌표 API 실패: ${response.statusCode}, ${response.body}");
      }

      setState(() {
        _currentMode = ControlMode.preset;
      });

      _publishMode(prevMode, ControlMode.preset);
    } catch (e) {
      print("❌ 프리셋 실행 중 오류: $e");
    }
  }

  /// ✅ 프리셋 추가
  Future<void> _addPreset() async {
    if (_presets.length >= 3) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 프리셋은 최대 3개까지 가능합니다')),
      );
      return;
    }

    final profileId = _profile!['id'];
    final dummyName = '프리셋 ${_presets.length + 1}';
    final created = await PresetService.createPreset(dummyName, profileId, 1);

    if (created != null) {
      await _loadProfileAndPresets();
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('✅ $dummyName이 추가되었습니다')),
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 프리셋 추가 실패')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_profile == null) {
      return const Scaffold(
        backgroundColor: backgroundBlack,
        body: Center(child: CircularProgressIndicator(color: Colors.white)),
      );
    }

    // 현재 모드 텍스트 설정
    String modeText;
    switch (_currentMode) {
      case ControlMode.auto:
        modeText = "AI 모드";
        break;
      case ControlMode.preset:
        modeText = "프리셋 모드";
        break;
      case ControlMode.manual:
        modeText = "수동 모드";
        break;
      case ControlMode.fix:
        modeText = "고정 모드";
        break;
      case ControlMode.off:
        modeText = "현재 전원이 꺼져있습니다.";
        break;
      default:
        modeText = "알 수 없는 모드";
    }

    return Scaffold(
      backgroundColor: backgroundBlack,
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            /// 상단 프로필 카드
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
              decoration: BoxDecoration(
                color: headerGrey.withOpacity(0.8),
                borderRadius: BorderRadius.circular(14),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black.withOpacity(0.2),
                    blurRadius: 8,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: Column(
                children: [
                  Text(
                    _currentMode == ControlMode.off
                        ? modeText
                        : '현재 $modeText 입니다.',
                    style: const TextStyle(
                      fontSize: 20,
                      fontWeight: FontWeight.bold,
                      color: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 18),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      ElevatedButton(
                        onPressed: _isBluetoothOn ? null : AppSettings.openAppSettings,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: _isBluetoothOn ? Colors.grey : primaryBlue,
                        ),
                        child: Text(_isBluetoothOn ? 'Connected' : 'Connect Bluetooth'),
                      ),
                      const SizedBox(width: 20),
                      ElevatedButton(
                        onPressed: _handleDisconnectAndScan,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.redAccent,
                        ),
                        child: const Text('Disconnect'),
                      ),
                    ],
                  ),
                ],
              ),
            ),
            const SizedBox(height: 30),

            /// 프리셋 영역
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                const Text(
                  '프리셋',
                  style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: textWhite,
                  ),
                ),
                IconButton(
                  icon: const Icon(Icons.settings, color: textGrey, size: 22),
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute(builder: (_) => const PresetPage()),
                    );
                  },
                ),
              ],
            ),
            const SizedBox(height: 16),
            _buildPresetArea(),
          ],
        ),
      ),
    );
  }

  /// ✅ 프리셋 버튼 UI
  Widget _buildPresetArea() {
    if (_presets.isEmpty) return _addPresetButton();

    return Row(
      children: [
        ..._presets.map((entry) {
          final presetName = entry['name'] ?? '이름 없음';
          final presetId = entry['id'] ?? 0;
          return Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _presetButton(presetName, presetId),
            ),
          );
        }).toList(),
        if (_presets.length < 3)
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _addPresetButton(),
            ),
          ),
      ],
    );
  }

  Widget _addPresetButton() {
    return InkWell(
      onTap: _addPreset,
      borderRadius: BorderRadius.circular(12),
      child: Container(
        height: 60,
        decoration: BoxDecoration(
          color: cardGrey,
          borderRadius: BorderRadius.circular(12),
          border: Border.all(color: primaryBlue, width: 1.5),
        ),
        child: const Center(
          child: Icon(Icons.add_circle_outline, size: 28, color: Colors.white),
        ),
      ),
    );
  }

  Widget _presetButton(String name, int presetId) {
    return InkWell(
      onTap: () => _handlePresetSelect(presetId),
      borderRadius: BorderRadius.circular(12),
      child: Container(
        height: 60,
        decoration: BoxDecoration(
          color: primaryBlue,
          borderRadius: BorderRadius.circular(12),
          boxShadow: [
            BoxShadow(
              color: primaryBlue.withOpacity(0.4),
              blurRadius: 6,
              offset: const Offset(0, 3),
            ),
          ],
        ),
        child: Center(
          child: Text(
            name,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 16,
              fontWeight: FontWeight.w600,
            ),
          ),
        ),
      ),
    );
  }
}
