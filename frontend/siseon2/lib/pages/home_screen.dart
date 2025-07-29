import 'package:flutter/material.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/preset_service.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];

  static const Color primaryBlue = Colors.blue;
  static const Color black = Colors.black;
  static const Color greyText = Colors.black54;

  @override
  void initState() {
    super.initState();
    _loadProfileAndPresets();
  }

  Future<void> _loadProfileAndPresets() async {
    final profile = await ProfileCacheService.loadProfile();
    if (profile == null) return;

    final profileId = profile['id'];
    final presets = await PresetService.fetchPresets(profileId);

    setState(() {
      _profile = profile;
      _presets = presets.take(3).toList(); // ✅ 프리셋 3개까지만 유지
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
        backgroundColor: Colors.white,
        body: Center(child: CircularProgressIndicator()),
      );
    }

    final nickname = _profile!['name'] ?? '사용자';
    final avatar = _profile!['avatar'] ?? 'frog';

    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('홈'),
        backgroundColor: primaryBlue,
      ),
      body: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 20),
        child: Column(
          children: [
            const SizedBox(height: 20),
            Center(
              child: Column(
                children: [
                  Image.asset('assets/images/profile_$avatar.png', width: 100),
                  const SizedBox(height: 12),
                  Text(
                    '안녕하세요, $nickname님!',
                    style: const TextStyle(fontSize: 24, color: black),
                  ),
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
              child: Text(
                '프리셋',
                style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: black),
              ),
            ),
            const SizedBox(height: 12),
            Expanded(
              child: Center(
                child: _buildPresetArea(),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPresetArea() {
    // ✅ 프리셋이 없는 경우 → 중앙에 + 버튼만
    if (_presets.isEmpty) {
      return IconButton(
        icon: const Icon(Icons.add_circle, size: 60, color: primaryBlue),
        onPressed: _addPreset,
      );
    }

    // ✅ 프리셋이 1~2개인 경우 → 좌측 프리셋, 우측에 +버튼
    if (_presets.length < 3) {
      return Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          ..._presets.asMap().entries.map((entry) {
            final index = entry.key;
            final presetName = '프리셋 ${index + 1}';
            return Expanded(
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 6),
                child: _presetButton(presetName),
              ),
            );
          }),
          Expanded(
            child: IconButton(
              icon: const Icon(Icons.add_circle, size: 50, color: primaryBlue),
              onPressed: _addPreset,
            ),
          ),
        ],
      );
    }

    // ✅ 프리셋이 3개인 경우 → 균등 꽉채움
    return Row(
      children: _presets.asMap().entries.map((entry) {
        final index = entry.key;
        final presetName = '프리셋 ${index + 1}';
        return Expanded(
          child: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 6),
            child: _presetButton(presetName),
          ),
        );
      }).toList(),
    );
  }

  Widget _presetButton(String name) {
    return OutlinedButton(
      style: OutlinedButton.styleFrom(
        side: const BorderSide(color: primaryBlue),
        foregroundColor: primaryBlue,
        padding: const EdgeInsets.symmetric(vertical: 14),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      ),
      onPressed: () {
        print('📌 선택된 프리셋: $name');
      },
      child: Text(
        name,
        style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600),
      ),
    );
  }
}
