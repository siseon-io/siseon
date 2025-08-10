import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart';
import 'package:http/http.dart' as http;
import '../services/auth_service.dart';

/// 🎨 공통 색상 팔레트
class AppColors {
  static const background = Color(0xFF0D1117); // 전체 배경
  static const card = Color(0xFF161B22);       // 카드/입력창 배경
  static const border = Color(0xFF334155);     // 테두리
  static const primary = Color(0xFF3B82F6);    // 포인트
  static const text = Colors.white;
  static const textSub = Colors.white70;
  static const textHint = Colors.white38;
}

class ProfileCreateScreen extends StatefulWidget {
  const ProfileCreateScreen({Key? key}) : super(key: key);

  @override
  State<ProfileCreateScreen> createState() => _ProfileCreateScreenState();
}

class _ProfileCreateScreenState extends State<ProfileCreateScreen> {
  String? _selectedAvatar;
  String _name = '';
  String _height = '';
  DateTime? _birthDate;
  String _visionLeft = '';
  String _visionRight = '';

  final Map<String, String> _avatarNames = {
    'profile_frog': '개구리',
    'profile_cat': '고양이',
    'profile_dog': '강아지',
    'profile_lion': '사자',
    'profile_mouse': '쥐',
    'profile_rabbit': '토끼',
  };

  final List<String?> _avatarOptions = [
    null,
    'assets/images/profile_frog.png',
    'assets/images/profile_cat.png',
    'assets/images/profile_dog.png',
    'assets/images/profile_lion.png',
    'assets/images/profile_mouse.png',
    'assets/images/profile_rabbit.png',
  ];

  void _pickDate() {
    int selectedYear = _birthDate?.year ?? 2000;
    int selectedMonth = _birthDate?.month ?? 1;
    int selectedDay = _birthDate?.day ?? 1;

    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: AppColors.card,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (_) {
        return Padding(
          padding: MediaQuery.of(context).viewInsets,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Text(
                  '생년월일 선택',
                  style: TextStyle(color: AppColors.text, fontSize: 18, fontWeight: FontWeight.w600),
                ),
                const SizedBox(height: 12),
                SizedBox(
                  height: 200,
                  child: Row(
                    children: [
                      Expanded(
                        child: CupertinoPicker(
                          scrollController: FixedExtentScrollController(initialItem: selectedYear - 1900),
                          itemExtent: 40,
                          onSelectedItemChanged: (index) => selectedYear = 1900 + index,
                          children: List.generate(
                            126,
                                (i) => Center(
                              child: Text('${1900 + i}년', style: const TextStyle(color: AppColors.text)),
                            ),
                          ),
                        ),
                      ),
                      Expanded(
                        child: CupertinoPicker(
                          scrollController: FixedExtentScrollController(initialItem: selectedMonth - 1),
                          itemExtent: 40,
                          onSelectedItemChanged: (index) => selectedMonth = index + 1,
                          children: List.generate(
                            12,
                                (i) => Center(
                              child: Text('${i + 1}월', style: const TextStyle(color: AppColors.text)),
                            ),
                          ),
                        ),
                      ),
                      Expanded(
                        child: CupertinoPicker(
                          scrollController: FixedExtentScrollController(initialItem: selectedDay - 1),
                          itemExtent: 40,
                          onSelectedItemChanged: (index) => selectedDay = index + 1,
                          children: List.generate(
                            31,
                                (i) => Center(
                              child: Text('${i + 1}일', style: const TextStyle(color: AppColors.text)),
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 16),
                SafeArea(
                  child: ElevatedButton(
                    onPressed: () {
                      setState(() {
                        _birthDate = DateTime(selectedYear, selectedMonth, selectedDay);
                      });
                      Navigator.pop(context);
                    },
                    style: ElevatedButton.styleFrom(
                      backgroundColor: AppColors.primary,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                    ),
                    child: const Text('확인', style: TextStyle(color: Colors.white)),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }

  void _showAvatarPicker() {
    showModalBottomSheet(
      context: context,
      backgroundColor: AppColors.card,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (ctx) {
        return Container(
          padding: const EdgeInsets.all(20),
          height: 360,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text(
                '아바타 선택',
                style: TextStyle(
                  fontFamily: 'Pretendard',
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: AppColors.text,
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: GridView.builder(
                  shrinkWrap: true,
                  gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
                    crossAxisCount: 3,
                    crossAxisSpacing: 16,
                    mainAxisSpacing: 16,
                  ),
                  itemCount: _avatarOptions.length,
                  itemBuilder: (context, index) {
                    final path = _avatarOptions[index];
                    final isSelected = path == _selectedAvatar;

                    return GestureDetector(
                      onTap: () {
                        setState(() => _selectedAvatar = path);
                        Navigator.pop(ctx);
                      },
                      child: Column(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Container(
                            decoration: BoxDecoration(
                              shape: BoxShape.circle,
                              border: Border.all(
                                color: isSelected ? AppColors.primary : AppColors.border,
                                width: isSelected ? 3 : 1,
                              ),
                            ),
                            child: CircleAvatar(
                              radius: 40,
                              backgroundColor: const Color(0xFF1F2937),
                              backgroundImage: path != null ? AssetImage(path) : null,
                              child: path == null
                                  ? const Icon(Icons.person_off, size: 30, color: Colors.grey)
                                  : null,
                            ),
                          ),
                          const SizedBox(height: 6),
                          Flexible(
                            child: Text(
                              path == null
                                  ? ''
                                  : _avatarNames[path.split('/').last.split('.').first] ?? '아바타',
                              style: const TextStyle(color: AppColors.textSub, fontSize: 13),
                              overflow: TextOverflow.ellipsis,
                              maxLines: 1,
                              textAlign: TextAlign.center,
                            ),
                          ),
                        ],
                      ),
                    );
                  },
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Future<void> _submitProfile() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('로그인이 필요합니다.')),
      );
      return;
    }

    final response = await http.post(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile'),
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer $token',
      },
      body: jsonEncode({
        "name": _name,
        "height": int.tryParse(_height),
        "birthDate": _birthDate?.toIso8601String(),
        "leftVision": double.tryParse(_visionLeft),
        "rightVision": double.tryParse(_visionRight),
        "imageUrl": _selectedAvatar,
      }),
    );

    if (response.statusCode == 200 || response.statusCode == 201) {
      if (!mounted) return;
      Navigator.pop(context, true);
    } else {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('생성 실패: ${response.statusCode}')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: AppColors.background,
      appBar: AppBar(
        backgroundColor: AppColors.background,
        elevation: 0,
        iconTheme: const IconThemeData(color: AppColors.text),
        title: const Text('프로필 추가', style: TextStyle(color: AppColors.text)),
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 32),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Center(
              child: InkWell(
                onTap: _showAvatarPicker,
                borderRadius: BorderRadius.circular(64),
                child: CircleAvatar(
                  radius: 50,
                  backgroundColor: const Color(0xFF1F2937),
                  backgroundImage: _selectedAvatar != null ? AssetImage(_selectedAvatar!) : null,
                  child: _selectedAvatar == null
                      ? const Icon(Icons.person, size: 50, color: Colors.white30)
                      : null,
                ),
              ),
            ),
            const SizedBox(height: 32),
            _buildField(
              label: '이름',
              hint: '프로필 이름을 입력하세요',
              icon: Icons.person,
              keyboardType: TextInputType.text,
              onChanged: (v) => setState(() => _name = v),
            ),
            const SizedBox(height: 16),
            _buildField(
              label: '키 (cm)',
              hint: '예: 170',
              icon: Icons.straighten,
              keyboardType: TextInputType.number,
              onChanged: (v) => setState(() => _height = v),
            ),
            const SizedBox(height: 16),
            GestureDetector(
              onTap: _pickDate,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text('생년월일', style: TextStyle(color: AppColors.textSub, fontSize: 12)),
                  const SizedBox(height: 6),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 18),
                    decoration: BoxDecoration(
                      color: AppColors.card,
                      borderRadius: BorderRadius.circular(12),
                      border: Border.all(color: AppColors.border, width: 1),
                    ),
                    child: Row(
                      children: [
                        const Icon(Icons.calendar_today, color: AppColors.textSub),
                        const SizedBox(width: 12),
                        Text(
                          _birthDate == null
                              ? '생년월일 선택'
                              : DateFormat('yyyy-MM-dd').format(_birthDate!),
                          style: TextStyle(
                            color: _birthDate == null ? AppColors.textHint : AppColors.text,
                            fontSize: 16,
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
            Row(
              children: [
                Expanded(
                  child: _buildField(
                    label: '좌안 시력',
                    hint: '예: 1.0',
                    icon: Icons.visibility,
                    keyboardType: const TextInputType.numberWithOptions(decimal: true),
                    onChanged: (v) => setState(() => _visionLeft = v),
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: _buildField(
                    label: '우안 시력',
                    hint: '예: 1.0',
                    icon: Icons.visibility,
                    keyboardType: const TextInputType.numberWithOptions(decimal: true),
                    onChanged: (v) => setState(() => _visionRight = v),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
      bottomNavigationBar: SafeArea(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(24, 0, 24, 12),
          child: ElevatedButton(
            onPressed: _name.trim().isEmpty ? null : _submitProfile,
            style: ElevatedButton.styleFrom(
              backgroundColor: AppColors.primary,            // 활성: 파랑
              foregroundColor: Colors.white,                 // 활성 텍스트: 하양
              disabledBackgroundColor: AppColors.primary,    // 비활성: 파랑(동일)
              disabledForegroundColor: Colors.white,         // 비활성 텍스트: 하양(동일)
              padding: const EdgeInsets.symmetric(vertical: 16),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(30)),
              elevation: 0,
            ),
            child: const Text(
              '저장',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
                color: Colors.white,
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildField({
    required String label,
    required String hint,
    required IconData icon,
    required TextInputType keyboardType,
    required ValueChanged<String> onChanged,
  }) {
    return TextField(
      style: const TextStyle(color: AppColors.text),
      keyboardType: keyboardType,
      decoration: InputDecoration(
        filled: true,
        fillColor: AppColors.card,
        prefixIcon: Icon(icon, color: AppColors.textSub),
        labelText: label,
        labelStyle: const TextStyle(color: AppColors.textSub),
        hintText: hint,
        hintStyle: const TextStyle(color: AppColors.textHint),
        enabledBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.border),
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.primary, width: 1.5),
        ),
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.border),
        ),
      ),
      onChanged: onChanged,
    );
  }
}
