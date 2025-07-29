import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart';
import 'package:http/http.dart' as http;
import '../services/auth_service.dart';

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
      backgroundColor: Colors.black,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (_) {
        return SizedBox(
          height: 300,
          child: Column(
            children: [
              const Padding(
                padding: EdgeInsets.all(12),
                child: Text('생년월일 선택',
                    style: TextStyle(color: Colors.white, fontSize: 18)),
              ),
              Expanded(
                child: Row(
                  children: [
                    Expanded(
                      child: CupertinoPicker(
                        scrollController: FixedExtentScrollController(
                            initialItem: selectedYear - 1900),
                        itemExtent: 40,
                        onSelectedItemChanged: (index) {
                          selectedYear = 1900 + index;
                        },
                        children: List.generate(
                          126, // 1900 ~ 2025
                              (index) => Center(
                            child: Text('${1900 + index}년',
                                style: TextStyle(color: Colors.white)),
                          ),
                        ),
                      ),
                    ),
                    Expanded(
                      child: CupertinoPicker(
                        scrollController: FixedExtentScrollController(
                            initialItem: selectedMonth - 1),
                        itemExtent: 40,
                        onSelectedItemChanged: (index) {
                          selectedMonth = index + 1;
                        },
                        children: List.generate(
                          12,
                              (index) => Center(
                            child: Text('${index + 1}월',
                                style: TextStyle(color: Colors.white)),
                          ),
                        ),
                      ),
                    ),
                    Expanded(
                      child: CupertinoPicker(
                        scrollController: FixedExtentScrollController(
                            initialItem: selectedDay - 1),
                        itemExtent: 40,
                        onSelectedItemChanged: (index) {
                          selectedDay = index + 1;
                        },
                        children: List.generate(
                          31,
                              (index) => Center(
                            child: Text('${index + 1}일',
                                style: TextStyle(color: Colors.white)),
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              ElevatedButton(
                onPressed: () {
                  setState(() {
                    _birthDate = DateTime(selectedYear, selectedMonth, selectedDay);
                  });
                  Navigator.pop(context);
                },
                style: ElevatedButton.styleFrom(
                  backgroundColor: const Color(0xFF3B82F6),
                ),
                child: const Text('확인'),
              ),
              const SizedBox(height: 10),
            ],
          ),
        );
      },
    );
  }

  void _showAvatarPicker() {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('아바타 선택'),
        content: SizedBox(
          width: double.maxFinite,
          child: SingleChildScrollView(
            scrollDirection: Axis.horizontal,
            child: Row(
              children: _avatarOptions.map((path) {
                final label = path == null ? '없음' : path.split('/').last.split('.').first;
                return Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 8),
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      InkWell(
                        onTap: () {
                          setState(() => _selectedAvatar = path);
                          Navigator.pop(ctx);
                        },
                        borderRadius: BorderRadius.circular(40),
                        child: CircleAvatar(
                          radius: 40,
                          backgroundColor: Colors.grey[200],
                          backgroundImage: path != null ? AssetImage(path) : null,
                          child: path == null
                              ? const Icon(Icons.person_off, size: 30, color: Colors.grey)
                              : null,
                        ),
                      ),
                      const SizedBox(height: 6),
                      Text(label),
                    ],
                  ),
                );
              }).toList(),
            ),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('닫기'),
          ),
        ],
      ),
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
      Navigator.pop(context, true);
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('생성 실패: ${response.statusCode}')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final dateText = _birthDate == null
        ? '생년월일 선택'
        : DateFormat('yyyy-MM-dd').format(_birthDate!);

    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        backgroundColor: Colors.black,
        elevation: 0,
        iconTheme: const IconThemeData(color: Colors.white),
        title: const Text('프로필 추가', style: TextStyle(color: Colors.white)),
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
                  backgroundColor: Colors.grey[900],
                  backgroundImage: _selectedAvatar != null
                      ? AssetImage(_selectedAvatar!)
                      : null,
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
                  const Text(
                    '생년월일',
                    style: TextStyle(color: Colors.white70, fontSize: 12),
                  ),
                  const SizedBox(height: 6),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 18),
                    decoration: BoxDecoration(
                      color: Colors.grey[900],
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Row(
                      children: [
                        const Icon(Icons.calendar_today, color: Colors.white70),
                        const SizedBox(width: 12),
                        Text(
                          _birthDate == null
                              ? '생년월일 선택'
                              : DateFormat('yyyy-MM-dd').format(_birthDate!),
                          style: TextStyle(
                            color: _birthDate == null ? Colors.white38 : Colors.white,
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
              backgroundColor: const Color(0xFF3B82F6),
              disabledBackgroundColor: Colors.grey[800],
              padding: const EdgeInsets.symmetric(vertical: 16),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(30),
              ),
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
      style: const TextStyle(color: Colors.white),
      keyboardType: keyboardType,
      decoration: InputDecoration(
        filled: true,
        fillColor: Colors.grey[900],
        prefixIcon: Icon(icon, color: Colors.white70),
        labelText: label,
        labelStyle: const TextStyle(color: Colors.white70),
        hintText: hint,
        hintStyle: const TextStyle(color: Colors.white38),
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: BorderSide.none,
        ),
      ),
      onChanged: onChanged,
    );
  }
}
