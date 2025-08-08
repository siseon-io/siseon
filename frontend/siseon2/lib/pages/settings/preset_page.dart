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

  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color cardGrey = Color(0xFF161B22);
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color textWhite = Colors.white;
  static const Color textGrey = Colors.white70;

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

  Future<void> _addPreset() async {
    if (_presets.length >= 3) {
      _showSnackBar('❌ 프리셋은 최대 3개까지 가능합니다');
      return;
    }

    final dummyName = '프리셋 ${_presets.length + 1}';
    final created = await PresetService.createPreset(dummyName, _profileId!, 1);

    if (created != null) {
      await _loadProfileAndPresets();
      _showSnackBar('✅ $dummyName이 추가되었습니다');
    } else {
      _showSnackBar('❌ 프리셋 추가 실패');
    }
  }

  void _renamePreset(int index) async {
    final preset = _presets[index];
    print('🛠 선택된 프리셋: $preset');

    final controller = TextEditingController(text: preset['name']);

    final newName = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: cardGrey,
        title: const Text('프리셋 이름 변경', style: TextStyle(color: Colors.white)),
        content: TextField(
          controller: controller,
          maxLength: 7, // ✅ 최대 7글자 제한
          style: const TextStyle(color: Colors.white),
          decoration: InputDecoration(
            counterStyle: const TextStyle(color: Colors.white54), // 글자 수 표시 스타일
            labelText: '새 이름 (최대 7글자)',
            labelStyle: const TextStyle(color: Colors.white60),
            filled: true,
            fillColor: const Color(0xFF1E2533),
            border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('취소', style: TextStyle(color: Colors.white70)),
          ),
          TextButton(
            onPressed: () {
              if (controller.text.trim().isEmpty) {
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(content: Text('❌ 이름을 입력해주세요')),
                );
                return;
              }
              if (controller.text.trim().length > 7) {
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(content: Text('❌ 이름은 최대 7글자까지 가능합니다')),
                );
                return;
              }
              Navigator.pop(ctx, controller.text.trim());
            },
            child: const Text('변경', style: TextStyle(color: primaryBlue)),
          ),
        ],
      ),
    );

    if (newName != null && newName.isNotEmpty) {
      print('🛠 이름 변경 요청: id=${preset['id']} → $newName');

      final updated = await PresetService.updatePreset(
        preset['id'],
        newName,
        _profileId!,
        preset['deviceId'],
        preset['position'] ?? {'x': 0, 'y': 0, 'z': 0},
      );

      if (updated) {
        await _loadProfileAndPresets();
        _showSnackBar('✅ 프리셋 이름이 변경되었습니다');
      } else {
        _showSnackBar('❌ 프리셋 이름 변경 실패');
      }
    }
  }


  void _deletePreset(int index) async {
    final preset = _presets[index];
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: cardGrey,
        title: const Text('프리셋 삭제', style: TextStyle(color: Colors.white)),
        content: Text('“${preset['name']}”을(를) 삭제하시겠습니까?',
            style: const TextStyle(color: Colors.white70)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: const Text('취소', style: TextStyle(color: Colors.white70)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            child: const Text('삭제', style: TextStyle(color: Colors.redAccent)),
          ),
        ],
      ),
    );

    if (confirmed == true) {
      print('🛠 삭제 요청 ID: ${preset['id']}'); // ✅ 로그 확인
      final deleted = await PresetService.deletePreset(preset['id']); // ✅ 수정
      if (deleted) {
        await _loadProfileAndPresets();
        _showSnackBar('✅ 프리셋이 삭제되었습니다');
      } else {
        _showSnackBar('❌ 프리셋 삭제 실패');
      }
    }
  }


  void _showSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        behavior: SnackBarBehavior.floating,
        backgroundColor: Colors.black87,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: backgroundBlack,
      appBar: AppBar(
        backgroundColor: backgroundBlack,
        foregroundColor: primaryBlue,
        elevation: 0,
        centerTitle: true,
        title: const Text('프리셋', style: TextStyle(fontWeight: FontWeight.bold)),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: Colors.white),
          onPressed: () => Navigator.pop(context,true),
        ),
      ),
      body: SafeArea(
        child: Column(
          children: [
            Expanded(
              child: ListView(
                padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
                children: [
                  ..._presets.map((preset) => _buildPresetCard(preset)).toList(),
                  if (_presets.length < 3) ...[ // ✅ 3개 미만일 때만 표시
                    const SizedBox(height: 14),
                    _buildSquareAddButton(),
                  ],
                ],
              ),
            ),
            const SizedBox(height: 12),
            Padding(
              padding: const EdgeInsets.fromLTRB(20, 0, 20, 24),
              child: SizedBox(
                width: double.infinity,
                height: 50,
                child: ElevatedButton.icon(
                  icon: const Icon(Icons.save, size: 18, color: Colors.white),
                  onPressed: () {
                    Navigator.pop(context, true);
                    _showSnackBar('✅ 저장되었습니다');
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: primaryBlue,
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(10)),
                  ),
                  label: const Text('저장', style: TextStyle(fontSize: 16, color: Colors.white)),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  /// ✅ 프리셋 카드 위젯
  Widget _buildPresetCard(Map<String, dynamic> preset) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 16),
      margin: const EdgeInsets.only(bottom: 14),
      decoration: BoxDecoration(
        color: cardGrey,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: Colors.white12, width: 1),
      ),
      child: Row(
        children: [
          Expanded(
            child: Text(
              preset['name'],
              style: const TextStyle(color: textWhite, fontSize: 16, fontWeight: FontWeight.w500),
            ),
          ),
          IconButton(
            icon: const Icon(Icons.edit, color: primaryBlue),
            onPressed: () => _renamePreset(_presets.indexOf(preset)),
          ),
          IconButton(
            icon: const Icon(Icons.delete, color: Colors.redAccent),
            onPressed: () => _deletePreset(_presets.indexOf(preset)),
          ),
        ],
      ),
    );
  }

  /// ✅ 정사각형 + 버튼 (3개 미만일 때만 표시)
  Widget _buildSquareAddButton() {
    return Center(
      child: GestureDetector(
        onTap: _addPreset,
        child: Container(
          width: 60,
          height: 60,
          decoration: BoxDecoration(
            color: primaryBlue,
            borderRadius: BorderRadius.circular(14),
          ),
          child: const Icon(Icons.add, color: Colors.white, size: 32),
        ),
      ),
    );
  }

  /// ✅ 저장 버튼
  Widget _buildSaveButton() {
    return Padding(
      padding: const EdgeInsets.only(bottom: 24),
      child: SizedBox(
        width: double.infinity,
        height: 50,
        child: ElevatedButton.icon(
          icon: const Icon(Icons.save, size: 18, color: Colors.white),
          onPressed: () {
            Navigator.pop(context);
            _showSnackBar('✅ 저장되었습니다');
          },
          style: ElevatedButton.styleFrom(
            backgroundColor: primaryBlue,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
          ),
          label: const Text(
            '저장',
            style: TextStyle(fontSize: 16, color: Colors.white),
          ),
        ),
      ),
    );
  }
}
