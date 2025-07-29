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
      _presets = presets;
    });

    print('📥 불러온 프리셋 목록: $_presets');
  }

  Future<void> _addPreset() async {
    final profileId = _profile!['id'];
    final dummyName = '새 프리셋 ${DateTime.now().millisecondsSinceEpoch}';

    final created = await PresetService.createPreset(
      dummyName,
      profileId,
      1, // deviceId
    );

    print('➕ 프리셋 생성 결과: $created');

    if (created != null) {
      await _loadProfileAndPresets();
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('✅ 프리셋이 추가되었습니다')),
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
        backgroundColor: Colors.blue,
      ),
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Image.asset(
              'assets/images/profile_$avatar.png',
              width: 100,
            ),
            const SizedBox(height: 12),
            Text(
              '안녕하세요, $nickname님!',
              style: const TextStyle(fontSize: 24, color: Colors.black),
            ),
            const SizedBox(height: 20),
            const Padding(
              padding: EdgeInsets.symmetric(horizontal: 20),
              child: TextField(
                decoration: InputDecoration(
                  border: OutlineInputBorder(),
                  labelText: '여기에 입력하세요',
                  labelStyle: TextStyle(color: Colors.black),
                ),
                style: TextStyle(color: Colors.black),
              ),
            ),
            const SizedBox(height: 32),
            if (_presets.length < 3) ...[
              const Text(
                '프리셋을 추가하세요',
                style: TextStyle(fontSize: 16, color: Colors.black54),
              ),
              const SizedBox(height: 8),
              IconButton(
                icon: const Icon(Icons.add_circle, size: 40, color: Colors.blue),
                onPressed: _addPreset,
                tooltip: '프리셋 추가',
              ),
              const SizedBox(height: 12),
              Wrap(
                spacing: 12,
                children: _presets.map((preset) {
                  return ElevatedButton(
                    onPressed: () {
                      print('📌 프리셋 선택됨: ${preset['name']}');
                    },
                    child: Text(preset['name']),
                  );
                }).toList(),
              ),
            ] else ...[
              Wrap(
                spacing: 12,
                children: _presets.map((preset) {
                  return ElevatedButton(
                    onPressed: () {
                      print('📌 프리셋 선택됨: ${preset['name']}');
                    },
                    child: Text(preset['name']),
                  );
                }).toList(),
              ),
            ],
          ],
        ),
      ),
    );
  }
}
