import 'package:flutter/material.dart';
import '../../services/profile_cache_service.dart';
import '../../services/preset_service.dart';

class PresetPage extends StatefulWidget {
  const PresetPage({Key? key}) : super(key: key);

  @override
  State<PresetPage> createState() => _PresetPageState();
}

class _PresetPageState extends State<PresetPage> {
  List<Map<String, dynamic>> _presets = [];
  int? _profileId;

  @override
  void initState() {
    super.initState();
    _loadProfileAndPresets();
  }

  Future<void> _loadProfileAndPresets() async {
    final profile = await ProfileCacheService.loadProfile();
    if (profile == null) return;

    _profileId = profile['id'];
    final result = await PresetService.fetchPresets(_profileId!);
    setState(() => _presets = result);
    print('📥 불러온 프리셋 목록: $_presets');
  }

  void _renamePreset(int index) async {
    final preset = _presets[index];
    print('📝 이름 변경 대상 프리셋 데이터: $preset');

    final controller = TextEditingController(text: preset['name']);
    final newName = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF2E2E30),
        titleTextStyle: const TextStyle(color: Colors.white, fontSize: 18),
        contentTextStyle: const TextStyle(color: Colors.white),
        title: const Text('프리셋 이름 변경'),
        content: TextField(
          controller: controller,
          style: const TextStyle(color: Colors.white),
          decoration: InputDecoration(
            labelText: '새 이름',
            labelStyle: const TextStyle(color: Colors.white60),
            filled: true,
            fillColor: const Color(0xFF3A3A3C),
            border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
          ),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx), child: const Text('취소')),
          TextButton(onPressed: () => Navigator.pop(ctx, controller.text), child: const Text('변경')),
        ],
      ),
    );

    if (newName != null && newName.trim().isNotEmpty) {
      final presetId = preset['presetId'];
      final deviceId = preset['deviceId'];
      final position = preset['position'] ?? {'x': 0, 'y': 0, 'z': 0};

      if (presetId == null || deviceId == null) {
        print('❌ presetId 또는 deviceId가 null입니다. 수정 중단');
        return;
      }

      final updated = await PresetService.updatePreset(
        presetId,
        newName.trim(),
        _profileId!,
        deviceId,
        position,
      );

      print('📌 프리셋 이름 변경 결과: $updated');

      if (updated) {
        await _loadProfileAndPresets();
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('✅ 프리셋 이름이 변경되었습니다')),
        );
      } else {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('❌ 프리셋 이름 변경 실패')),
        );
      }
    }
  }

  void _deletePreset(int index) async {
    final preset = _presets[index];
    final presetId = preset['presetId'];

    print('🗑️ 삭제 대상 프리셋 데이터: $preset');

    if (presetId == null) {
      print('❌ presetId가 null이라 삭제 중단');
      return;
    }

    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF2E2E30),
        titleTextStyle: const TextStyle(color: Colors.white, fontSize: 18),
        contentTextStyle: const TextStyle(color: Colors.white),
        title: const Text('프리셋 삭제'),
        content: Text('“${preset['name']}” 을(를) 삭제하시겠습니까?'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('취소')),
          TextButton(onPressed: () => Navigator.pop(ctx, true), child: const Text('삭제')),
        ],
      ),
    );

    if (confirmed == true) {
      final deleted = await PresetService.deletePreset(presetId);
      print('🗑️ 프리셋 삭제 결과: $deleted');

      if (deleted) {
        await _loadProfileAndPresets();
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('✅ 프리셋이 삭제되었습니다')),
        );
      } else {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('❌ 프리셋 삭제 실패')),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF1C1C1E),
      appBar: AppBar(
        backgroundColor: const Color(0xFF1C1C1E),
        foregroundColor: const Color(0xFF2563FF),
        elevation: 0,
        title: const Text('프리셋', style: TextStyle(fontWeight: FontWeight.bold)),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: Column(
          children: [
            Expanded(
              child: ListView.separated(
                padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
                itemCount: _presets.length,
                separatorBuilder: (_, __) => const SizedBox(height: 16),
                itemBuilder: (ctx, idx) {
                  return Container(
                    padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                    decoration: BoxDecoration(
                      color: const Color(0xFF2E3A59),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Row(
                      children: [
                        Expanded(
                          child: Text(
                            _presets[idx]['name'],
                            style: const TextStyle(color: Colors.white, fontSize: 16),
                          ),
                        ),
                        IconButton(
                          icon: const Icon(Icons.edit, color: Color(0xFF93C5FD)),
                          onPressed: () => _renamePreset(idx),
                        ),
                        IconButton(
                          icon: const Icon(Icons.delete, color: Colors.redAccent),
                          onPressed: () => _deletePreset(idx),
                        ),
                      ],
                    ),
                  );
                },
              ),
            ),
            const SizedBox(height: 12),
            Padding(
              padding: const EdgeInsets.fromLTRB(20, 0, 20, 24),
              child: SizedBox(
                width: double.infinity,
                height: 50,
                child: ElevatedButton(
                  onPressed: () {
                    Navigator.pop(context);
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('✅ 저장되었습니다')),
                    );
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color(0xFF2563FF),
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
                  ),
                  child: const Text('저장', style: TextStyle(fontSize: 16)),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
