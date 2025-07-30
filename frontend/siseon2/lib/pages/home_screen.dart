import 'package:flutter/material.dart';
import 'package:app_settings/app_settings.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/preset_service.dart';
import 'ble_scan_screen.dart'; // ✅ BLE 스캔 화면으로 이동하기 위해 import

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];
  bool _isBluetoothOn = false;

  static const Color primaryBlue = Colors.blue;
  static const Color black = Colors.black;

  @override
  void initState() {
    super.initState();
    _loadProfileAndPresets();
    _checkBluetoothState();
  }

  Future<void> _checkBluetoothState() async {
    final isOn = await FlutterBluePlus.isOn;
    setState(() => _isBluetoothOn = isOn);
  }

  Future<void> _loadProfileAndPresets() async {
    final profile = await ProfileCacheService.loadProfile();
    if (profile == null) return;
    final presets = await PresetService.fetchPresets(profile['id']);

    setState(() {
      _profile = profile;
      _presets = presets.take(3).toList();
    });
  }

  Future<void> _addPreset() async {
    if (_presets.length >= 3) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 프리셋은 최대 3개까지')),
      );
      return;
    }

    final profileId = _profile!['id'];
    final dummyName = '프리셋 ${_presets.length + 1}';
    final created = await PresetService.createPreset(dummyName, profileId, 1);

    if (created != null) {
      await _loadProfileAndPresets();
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('✅ $dummyName 추가됨')),
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 프리셋 추가 실패')),
      );
    }
  }

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
      await AppSettings.openAppSettings(); // ✅ 여기!
      return;
    }

    // ✅ BLE 스캔 화면으로 이동
    if (!mounted) return;
    Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => const BleScanScreen()),
    );
  }
  @override
  Widget build(BuildContext context) {
    if (_profile == null) {
      return const Scaffold(
        backgroundColor: Colors.white,
        body: Center(child: CircularProgressIndicator()),
      );
    }

    final nickname = _profile!['name'] ?? '사용자';
    final avatar = _profile!['avatar'] ?? 'frog';

    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(title: const Text('홈'), backgroundColor: primaryBlue),
      body: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 20),
        child: Column(
          children: [
            const SizedBox(height: 20),
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
            const SizedBox(height: 20),
            Center(
              child: Column(
                children: [
                  Image.asset('assets/images/profile_$avatar.png', width: 100),
                  const SizedBox(height: 12),
                  Text('안녕하세요, $nickname님!', style: const TextStyle(fontSize: 24, color: black)),
                ],
              ),
            ),
            const SizedBox(height: 20),
            const TextField(
              decoration: InputDecoration(
                border: OutlineInputBorder(),
                labelText: '여기에 입력하세요',
                labelStyle: TextStyle(color: black),
              ),
              style: TextStyle(color: black),
            ),
            const SizedBox(height: 32),
            const Align(
              alignment: Alignment.centerLeft,
              child: Text('프리셋', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: black)),
            ),
            const SizedBox(height: 12),
            Expanded(child: Center(child: _buildPresetArea())),
          ],
        ),
      ),
    );
  }

  Widget _buildPresetArea() {
    if (_presets.isEmpty) {
      return IconButton(
        icon: const Icon(Icons.add_circle, size: 60, color: primaryBlue),
        onPressed: _addPreset,
      );
    }

    final buttons = _presets.asMap().entries.map((entry) {
      final index = entry.key;
      final name = '프리셋 ${index + 1}';
      return Expanded(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 6),
          child: _presetButton(name),
        ),
      );
    }).toList();

    if (_presets.length < 3) {
      buttons.add(
        Expanded(
          child: IconButton(
            icon: const Icon(Icons.add_circle, size: 50, color: primaryBlue),
            onPressed: _addPreset,
          ),
        ),
      );
    }

    return Row(mainAxisAlignment: MainAxisAlignment.center, children: buttons);
  }

  Widget _presetButton(String name) {
    return OutlinedButton(
      style: OutlinedButton.styleFrom(
        side: const BorderSide(color: primaryBlue),
        foregroundColor: primaryBlue,
        padding: const EdgeInsets.symmetric(vertical: 14),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      onPressed: () => print('📌 선택된 프리셋: $name'),
      child: Text(name, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
    );
  }
}