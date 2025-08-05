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
import 'package:siseon2/pages/manual_page.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/models/control_mode.dart';

class HomeScreen extends StatefulWidget {
  final VoidCallback onAiModeSwitch; // RootScreen에서 콜백 전달받음
  final void Function(BluetoothCharacteristic writableChar)? onConnect; // ✅ RootScreen으로 BLE 전달 콜백

  const HomeScreen({
    super.key,
    required this.onAiModeSwitch,
    this.onConnect,
  });

  @override
  HomeScreenState createState() => HomeScreenState();
}

class HomeScreenState extends State<HomeScreen> {
  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];
  bool _isBluetoothOn = false;
  ControlMode _currentMode = ControlMode.auto; // 초기 모드: auto

  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

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
    _initPermissions();
  }

  /// ✅ BLE 권한 요청
  Future<void> _initPermissions() async {
    final bleScan = await Permission.bluetoothScan.request();
    final bleConnect = await Permission.bluetoothConnect.request();
    final location = await Permission.location.request();

    if (!bleScan.isGranted || !bleConnect.isGranted || !location.isGranted) {
      print('❌ BLE 권한이 부족합니다. 기능이 제한될 수 있습니다.');
      if (await Permission.bluetoothConnect.isPermanentlyDenied) {
        openAppSettings();
      }
    } else {
      print('✅ BLE 권한 모두 허용됨');
    }
  }

  /// ✅ 블루투스 상태 체크
  Future<void> _checkBluetoothState() async {
    final isOn = await FlutterBluePlus.isOn;
    setState(() => _isBluetoothOn = isOn);
  }

  /// ✅ BLE 스캔 및 연결
  Future<void> _handleDisconnectAndScan() async {
    final result = await Navigator.push<Map<String, dynamic>>(
      context,
      MaterialPageRoute(builder: (_) => const BleScanScreen()),
    );
    if (result == null) return;

    setState(() {
      _connectedDevice = result['device'] as BluetoothDevice;
      _writableChar = result['writableChar'] as BluetoothCharacteristic;
    });

    // ✅ RootScreen에 BLE characteristic 전달
    if (widget.onConnect != null && _writableChar != null) {
      widget.onConnect!(_writableChar!);
      print("✅ RootScreen으로 writableChar 전달 완료: $_writableChar");
    }

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('✅ BLE 연결 성공')),
    );
  }

  /// ✅ BLE 연결 해제
  Future<void> _handleDisconnect() async {
    await _connectedDevice?.disconnect();
    setState(() {
      _connectedDevice = null;
      _writableChar = null;
    });
  }

  /// ✅ 수동 조작 화면 이동
  void _goManual() {
    if (_writableChar == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ BLE 기기를 먼저 연결해주세요.')),
      );
      return;
    }
    Navigator.push(
      context,
      MaterialPageRoute(
        builder: (_) => ManualPage(writableChar: _writableChar!),
      ),
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
    widget.onAiModeSwitch();
  }

  /// ✅ 프리셋 선택 시 API & MQTT 발행
  Future<void> _handlePresetSelect(int presetId) async {
    if (_profile == null) return;
    final profileId = _profile!['id'];
    final prevMode = _currentMode;

    try {
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

    final isConnected = _connectedDevice != null;

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
                      ElevatedButton.icon(
                        onPressed: isConnected ? null : _handleDisconnectAndScan,
                        icon: Icon(
                          isConnected ? Icons.bluetooth_connected : Icons.bluetooth_searching,
                          color: Colors.white,
                        ),
                        label: Text(
                          isConnected ? 'Connected' : 'Scan & Connect',
                          style: const TextStyle(color: Colors.white),
                        ),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: isConnected ? Colors.green : primaryBlue,
                        ),
                      ),
                      const SizedBox(width: 16),
                      if (isConnected)
                        ElevatedButton.icon(
                          onPressed: _handleDisconnect,
                          icon: const Icon(Icons.close, color: Colors.white),
                          label: const Text('Disconnect', style: TextStyle(color: Colors.white)),
                          style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
                        ),
                    ],
                  ),
                  if (isConnected) ...[
                    const SizedBox(height: 12),
                    ElevatedButton.icon(
                      onPressed: _goManual,
                      icon: const Icon(Icons.gamepad, color: Colors.white),
                      label: const Text('조작 화면 열기', style: TextStyle(color: Colors.white)),
                      style: ElevatedButton.styleFrom(backgroundColor: Colors.orange),
                    ),
                  ],
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
