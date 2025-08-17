// 📁 lib/pages/settings/preset_page.dart
import 'package:flutter/material.dart';
import '../../services/profile_cache_service.dart';
import '../../services/preset_service.dart';

class PresetPage extends StatefulWidget {
  // 🔹 FCM에서 바로 진입할 때 전달받을 값들 (선택)
  final int? profileId;
  final bool fromSuggest; // FCM 'preset_suggest'로 진입했는지 표시

  const PresetPage({
    Key? key,
    this.profileId,
    this.fromSuggest = false,
  }) : super(key: key);

  @override
  State<PresetPage> createState() => _PresetPageState();
}

class _PresetPageState extends State<PresetPage> {
  List<Map<String, dynamic>> _presets = [];
  int? _profileId;

  bool _isLoading = false;      // 로딩 스피너 제어
  bool _isConfirming = false;   // 제안 저장 중 표시

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
    setState(() => _isLoading = true);

    // 우선순위: 위젯 인자로 넘어온 profileId → 캐시
    _profileId = widget.profileId;
    if (_profileId == null) {
      final profile = await ProfileCacheService.loadProfile();
      if (!mounted) return;
      if (profile == null) {
        setState(() => _isLoading = false);
        return;
      }
      _profileId = profile['id'];
    }

    final result = await PresetService.fetchPresets(_profileId!);
    if (!mounted) return;
    setState(() {
      _presets = result;
      _isLoading = false;
    });
  }

  // 🔵 FCM 제안: "이 자세로 저장" 실행
  Future<void> _confirmSuggestedPreset() async {
    if (_profileId == null) return;            // (알림 제거) — 조용히 반환
    if (_presets.length >= 3) return;          // (알림 제거)

    final name = '프리셋 ${_presets.length + 1}';
    try {
      setState(() => _isConfirming = true);
      await PresetService.confirm(profileId: _profileId!, name: name);
      await _loadProfileAndPresets();

      // FCM 제안으로 들어온 경우에는 바로 닫기
      if (widget.fromSuggest && mounted) {
        Navigator.pop(context, true);
        return;
      }
      // (성공 알림 제거)
    } on PresetSaveException catch (e) {
      // ✅ “잠시 후 다시 시도” 메시지만 유지
      if (e.code == 'no_raw_posture') {
        _showSnackBar('약 10초 후 다시 시도해주세요.');
      }
      // 그 외 코드는 조용히 무시
    } catch (_) {
      // (알림 제거)
    } finally {
      if (mounted) setState(() => _isConfirming = false);
    }
  }

  Future<void> _addPreset() async {
    if (_profileId == null) return;            // (알림 제거)
    if (_presets.length >= 3) return;          // (알림 제거)

    final dummyName = '프리셋 ${_presets.length + 1}';
    try {
      final created = await PresetService.createPreset(dummyName, _profileId!, 1);
      if (created != null) {
        await _loadProfileAndPresets();
        // (성공 알림 제거)
      }
    } on PresetSaveException catch (e) {
      // ✅ “잠시 후 다시 시도”만 표시
      if (e.code == 'no_raw_posture') {
        _showSnackBar('약 10초 후 다시 시도해주세요.');
      }
    } catch (_) {
      // (알림 제거)
    }
  }

  // 이름 변경
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
                        // (알림 제거) — 유효성 실패 시 아무 동작 안 함
                        if (value.isEmpty) return;
                        if (value.length > 7) return;
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
      } else {
        // (알림 제거)
      }
    }
  }

  // 삭제
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
      } else {
        // (알림 제거)
      }
    }
  }

  // 🔔 유일하게 허용된 알림: "약 10초 후 다시 시도해주세요."
  void _showSnackBar(String message) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        behavior: SnackBarBehavior.floating,
        backgroundColor: Colors.black87,
        duration: const Duration(seconds: 2),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: backgroundBlack,
      appBar: AppBar(
        backgroundColor: backgroundBlack,
        elevation: 0,
        centerTitle: true,
        title: const Text(
          '프리셋',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: Colors.white, // ✅ 흰색 고정
          ),
        ),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: primaryBlue), // 🔵 아이콘만 블루
          onPressed: () => Navigator.pop(context, true),
        ),
      ),
      body: SafeArea(
        child: _isLoading
            ? const Center(child: CircularProgressIndicator())
            : Column(
          children: [
            // 🔹 FCM 제안 배너: fromSuggest=true일 때만 표시
            if (widget.fromSuggest) _suggestBanner(),
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
          ],
        ),
      ),
    );
  }

  /// 🔹 프리셋 제안 배너 (상단)
  Widget _suggestBanner() {
    return Container(
      margin: const EdgeInsets.fromLTRB(20, 16, 20, 0),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: const Color(0xFF0F172A), // 짙은 남색
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: primaryBlue.withOpacity(0.3)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text('이 자세로 프리셋을 저장할까요?',
              style: TextStyle(color: Colors.white, fontWeight: FontWeight.w700, fontSize: 16)),
          const SizedBox(height: 8),
          const Text('최근 1시간 동안 비슷한 자세가 유지됐어요.',
              style: TextStyle(color: Colors.white70, fontSize: 13)),
          const SizedBox(height: 12),
          SizedBox(
            width: double.infinity,
            height: 42,
            child: ElevatedButton(
              onPressed: _isConfirming ? null : _confirmSuggestedPreset,
              style: ElevatedButton.styleFrom(
                backgroundColor: primaryBlue,
                shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
              ),
              child: _isConfirming
                  ? const SizedBox(width: 20, height: 20, child: CircularProgressIndicator(strokeWidth: 2))
                  : const Text('이 자세로 저장', style: TextStyle(color: Colors.white)),
            ),
          ),
        ],
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
            color: Colors.grey[800], // 🔹 회색 버튼
            borderRadius: BorderRadius.circular(14),
          ),
          child: const Icon(Icons.add, color: Colors.white, size: 32),
        ),
      ),
    );
  }
}
