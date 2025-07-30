import 'package:flutter/material.dart';
import 'package:app_settings/app_settings.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/preset_service.dart';
import 'package:siseon2/pages/settings/preset_page.dart';
import 'package:siseon2/pages/settings/edit_profile.dart';
import 'package:siseon2/pages/ble_scan_screen.dart'; // ✅ BLE 스캔 화면

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];
  bool _isBluetoothOn = false; // ✅ 블루투스 상태

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
    _checkBluetoothState(); // ✅ 블루투스 상태 확인
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

    final nickname = _profile!['name'] ?? '사용자';
    final avatarImage = _profile!['imageUrl'] != null && _profile!['imageUrl'].isNotEmpty
        ? AssetImage(_profile!['imageUrl'])
        : AssetImage('assets/images/profile_${_profile!['avatar'] ?? 'frog'}.png');

    return Scaffold(
      backgroundColor: backgroundBlack,
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            /// ✅ 상단 프로필 영역
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
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Row(
                    children: [
                      CircleAvatar(
                        radius: 26,
                        backgroundImage: avatarImage,
                      ),
                      const SizedBox(width: 12),
                      Text(
                        '$nickname님 안녕하세요!',
                        style: const TextStyle(
                          fontSize: 17,
                          fontWeight: FontWeight.w600,
                          color: textWhite,
                        ),
                      ),
                    ],
                  ),
                  IconButton(
                    icon: const Icon(Icons.settings, color: Colors.white70),
                    onPressed: () async {
                      await Navigator.push(
                        context,
                        MaterialPageRoute(builder: (_) => const EditProfilePage()),
                      );
                      _loadProfileAndPresets(); // ✅ 돌아오면 프로필 새로고침
                    },
                  ),
                ],
              ),
            ),
            const SizedBox(height: 25),

            /// ✅ AI MODE 상태 카드 + 블루투스 버튼 추가
            Container(
              padding: const EdgeInsets.all(22),
              decoration: BoxDecoration(
                color: cardGrey,
                borderRadius: BorderRadius.circular(16),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black.withOpacity(0.15),
                    blurRadius: 8,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: Column(
                children: [
                  const Text(
                    '현재 AI MODE 입니다.',
                    style: TextStyle(
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

            /// ✅ 프리셋 영역
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

  Widget _buildPresetArea() {
    if (_presets.isEmpty) return _addPresetButton();

    return Row(
      children: [
        ..._presets.map((entry) {
          final presetName = entry['name'] ?? '이름 없음';
          return Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _presetButton(presetName),
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

  Widget _presetButton(String name) {
    return InkWell(
      onTap: () => print('📌 선택된 프리셋: $name'),
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
