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

  // 이름 변경 다이얼로그 (왼쪽=변경, 오른쪽=취소)
  void _renamePreset(int index) async {
    final preset = _presets[index];
    final controller = TextEditingController(text: preset['name']);

    final newName = await showDialog<String>(
      context: context,
      barrierDismissible: true,
      barrierColor: Colors.black.withOpacity(0.6),
      builder: (ctx) => Dialog(
        backgroundColor: cardGrey,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Align(
                alignment: Alignment.centerLeft,
                child: Text(
                  '프리셋 이름 변경',
                  style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold, fontSize: 18),
                ),
              ),
              const SizedBox(height: 12),
              TextField(
                controller: controller,
                maxLength: 7,
                style: const TextStyle(color: Colors.white),
                decoration: InputDecoration(
                  counterStyle: const TextStyle(color: Colors.white54),
                  labelText: '새 이름 (최대 7글자)',
                  labelStyle: const TextStyle(color: Colors.white60),
                  filled: true,
                  fillColor: const Color(0xFF1E2533),
                  border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
                ),
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () {
                        final value = controller.text.trim();
                        if (value.isEmpty) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            const SnackBar(content: Text('❌ 이름을 입력해주세요')),
                          );
                          return;
                        }
                        if (value.length > 7) {
                          ScaffoldMessenger.of(context).showSnackBar(
                            const SnackBar(content: Text('❌ 이름은 최대 7글자까지 가능합니다')),
                          );
                          return;
                        }
                        Navigator.pop(ctx, value);
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: primaryBlue,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text('변경', style: TextStyle(color: Colors.white, fontWeight: FontWeight.w600)),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(ctx),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: primaryBlue),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text('취소', style: TextStyle(color: primaryBlue)),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );

    if (newName != null && newName.isNotEmpty) {
      final ok = await PresetService.updatePresetName(
        (preset['id'] is int) ? preset['id'] as int : int.parse('${preset['id']}'),
        newName,
        _profileId!,
      );

      if (ok) {
        await _loadProfileAndPresets();
        _showSnackBar('✅ 프리셋 이름이 변경되었습니다');
      } else {
        _showSnackBar('❌ 프리셋 이름 변경 실패');
      }
    }
  }

  // 삭제 다이얼로그 (왼쪽=삭제, 오른쪽=취소)
  void _deletePreset(int index) async {
    final preset = _presets[index];
    final confirmed = await showDialog<bool>(
      context: context,
      barrierDismissible: true,
      barrierColor: Colors.black.withOpacity(0.6),
      builder: (ctx) => Dialog(
        backgroundColor: cardGrey,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Align(
                alignment: Alignment.centerLeft,
                child: Text(
                  '프리셋 삭제',
                  style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold, fontSize: 18),
                ),
              ),
              const SizedBox(height: 8),
              Text('“${preset['name']}”을(를) 삭제하시겠습니까?', style: const TextStyle(color: Colors.white70)),
              const SizedBox(height: 16),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () => Navigator.pop(ctx, true),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text('삭제', style: TextStyle(color: Colors.white, fontWeight: FontWeight.w600)),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(ctx, false),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: primaryBlue),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text('취소', style: TextStyle(color: primaryBlue)),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );

    if (confirmed == true) {
      final deleted = await PresetService.deletePreset(
        (preset['id'] is int) ? preset['id'] as int : int.parse('${preset['id']}'),
      );
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
          onPressed: () => Navigator.pop(context, true),
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
                  if (_presets.length < 3) ...[
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
                child: ElevatedButton(
                  onPressed: () {
                    Navigator.pop(context, true);
                    _showSnackBar('✅ 저장되었습니다');
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: primaryBlue,
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
                  ),
                  child: const Text('저장', style: TextStyle(fontSize: 16, color: Colors.white)),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  /// 프리셋 카드
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

  /// 정사각형 + 버튼
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
}
