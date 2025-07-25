// 📁 lib/pages/settings/preset_page.dart

import 'package:flutter/material.dart';

class PresetPage extends StatefulWidget {
  const PresetPage({Key? key}) : super(key: key);

  @override
  State<PresetPage> createState() => _PresetPageState();
}

class _PresetPageState extends State<PresetPage> {
  List<String> _presets = ['프리셋 1', '프리셋 2', '프리셋 3'];

  void _renamePreset(int index) async {
    final controller = TextEditingController(text: _presets[index]);
    final newName = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('프리셋 이름 변경'),
        content: TextField(
          controller: controller,
          decoration: const InputDecoration(
            border: OutlineInputBorder(), labelText: '새 이름',
          ),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx), child: const Text('취소')),
          TextButton(onPressed: () => Navigator.pop(ctx, controller.text), child: const Text('변경')),
        ],
      ),
    );
    if (newName != null && newName.trim().isNotEmpty) {
      setState(() => _presets[index] = newName.trim());
    }
  }

  void _deletePreset(int index) {
    showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('프리셋 삭제'),
        content: Text('“${_presets[index]}” 을(를) 삭제하시겠습니까?'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('취소')),
          TextButton(onPressed: () => Navigator.pop(ctx, true), child: const Text('삭제')),
        ],
      ),
    ).then((confirmed) {
      if (confirmed == true) {
        setState(() => _presets.removeAt(index));
      }
    });
  }

  void _saveAll() {
    // TODO: 저장 로직 (서버 전송 등)
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('프리셋이 저장되었습니다.')));
    Navigator.pop(context);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('프리셋'),
        backgroundColor: Colors.white,
        foregroundColor: const Color(0xFF2563FF),
        elevation: 0,
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: Color(0xFF2563FF)),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 20),
          child: Column(
            children: [
              Expanded(
                child: ListView.separated(
                  padding: const EdgeInsets.only(top: 24),
                  itemCount: _presets.length,
                  separatorBuilder: (_, __) => const SizedBox(height: 16),
                  itemBuilder: (ctx, idx) {
                    return Container(
                      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                      decoration: BoxDecoration(
                        color: const Color(0xFFEEF2FF),
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Row(
                        children: [
                          Expanded(
                            child: Text(_presets[idx], style: const TextStyle(fontSize: 16)),
                          ),
                          IconButton(
                            icon: const Icon(Icons.edit, color: Color(0xFF2563FF)),
                            onPressed: () => _renamePreset(idx),
                          ),
                          IconButton(
                            icon: const Icon(Icons.delete, color: Colors.grey),
                            onPressed: () => _deletePreset(idx),
                          ),
                        ],
                      ),
                    );
                  },
                ),
              ),
              const SizedBox(height: 24),
              SizedBox(
                width: double.infinity,
                height: 50,
                child: ElevatedButton(
                  onPressed: _saveAll,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color(0xFF2563FF),
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                  ),
                  child: const Text('저장', style: TextStyle(fontSize: 16)),
                ),
              ),
              const SizedBox(height: 24),
            ],
          ),
        ),
      ),
    );
  }
}
