// EditProfilePage.dart - 전체 수정 버전 (선택된 프로필 기준)
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart';
import 'package:http/http.dart' as http;
import '../../services/auth_service.dart';
import '../../profile_select_screen.dart';
import '../../services/profile_cache_service.dart';

class EditProfilePage extends StatefulWidget {
  const EditProfilePage({Key? key}) : super(key: key);

  @override
  State<EditProfilePage> createState() => _EditProfilePageState();
}

class _EditProfilePageState extends State<EditProfilePage> {
  final _nameController = TextEditingController();
  final _heightController = TextEditingController();
  final _visionLeftController = TextEditingController();
  final _visionRightController = TextEditingController();
  final _birthDateController = TextEditingController();
  DateTime? _birthDate;
  int? _profileId;
  bool _isLoading = true;
  String? _selectedImage;

  final List<String> _availableImages = [
    'assets/images/profile_frog.png',
    'assets/images/profile_cat.png',
    'assets/images/profile_dog.png',
    'assets/images/profile_lion.png',
    'assets/images/profile_mouse.png',
    'assets/images/profile_rabbit.png',
  ];

  @override
  void initState() {
    super.initState();
    _fetchProfile();
  }

  Future<void> _fetchProfile() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      setState(() => _isLoading = false);
      return;
    }

    try {
      final cached = await ProfileCacheService.loadProfile();
      if (cached == null || cached['id'] == null) {
        setState(() => _isLoading = false);
        return;
      }
      final selectedId = cached['id'];

      final response = await http.get(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile'),
        headers: {'Authorization': 'Bearer $token'},
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(utf8.decode(response.bodyBytes));
        if (data is List && data.isNotEmpty) {
          final user = data.firstWhere(
                (e) => e['id'] == selectedId,
            orElse: () => data.first,
          );

          setState(() {
            _profileId = user['id'];
            _nameController.text = user['name'] ?? '';
            _heightController.text = user['height']?.toString() ?? '';
            _visionLeftController.text = user['leftVision']?.toString() ?? '';
            _visionRightController.text = user['rightVision']?.toString() ?? '';
            if (user['birthDate'] != null) {
              _birthDate = DateTime.tryParse(user['birthDate']);
              if (_birthDate != null) {
                _birthDateController.text = DateFormat('yyyy-MM-dd').format(_birthDate!);
              }
            }
            _selectedImage = user['imageUrl'];
            _isLoading = false;
          });
        } else {
          setState(() => _isLoading = false);
        }
      } else {
        setState(() => _isLoading = false);
      }
    } catch (e) {
      setState(() => _isLoading = false);
    }
  }

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
                          126,
                              (index) => Center(
                            child: Text('${1900 + index}년',
                                style: const TextStyle(color: Colors.white)),
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
                                style: const TextStyle(color: Colors.white)),
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
                                style: const TextStyle(color: Colors.white)),
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
                    _birthDateController.text =
                        DateFormat('yyyy-MM-dd').format(_birthDate!);
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

  Future<void> _save() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null || _profileId == null) return;

    final payload = {
      "name": _nameController.text.trim(),
      "height": double.tryParse(_heightController.text.trim()),
      "birthDate": _birthDate?.toIso8601String(),
      "leftVision": double.tryParse(_visionLeftController.text.trim()),
      "rightVision": double.tryParse(_visionRightController.text.trim()),
      "imageUrl": _selectedImage,
    };

    final response = await http.put(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile/$_profileId'),
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer $token',
      },
      body: jsonEncode(payload),
    );

    if (response.statusCode == 200 && mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('프로필이 저장되었습니다.')),
      );
      Navigator.pop(context);
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('수정 실패: ${response.statusCode}')),
      );
    }
  }

  Future<void> _deleteProfile() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null || _profileId == null) return;

    final response = await http.delete(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile/$_profileId'),
      headers: {'Authorization': 'Bearer $token'},
    );

    if (response.statusCode == 204 && mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('프로필이 삭제 요청되었습니다.')),
      );
      Navigator.pushAndRemoveUntil(
        context,
        MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
            (route) => false,
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('삭제 실패: ${response.statusCode}')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        backgroundColor: Colors.black,
        elevation: 0,
        iconTheme: const IconThemeData(color: Colors.white),
        title: const Text('프로필 수정', style: TextStyle(color: Colors.white)),
        actions: [
          TextButton(
            onPressed: _deleteProfile,
            child: const Text('삭제', style: TextStyle(color: Colors.red)),
          )
        ],
      ),
      body: _isLoading
          ? const Center(child: CircularProgressIndicator(color: Colors.white))
          : SingleChildScrollView(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Center(
              child: GestureDetector(
                onTap: _showImagePicker,
                child: CircleAvatar(
                  radius: 48,
                  backgroundImage: AssetImage(
                    (_selectedImage != null && _selectedImage!.isNotEmpty)
                        ? _selectedImage!
                        : 'assets/images/user_placeholder.png',
                  ),
                  backgroundColor: Colors.grey[700],
                ),
              ),
            ),
            const SizedBox(height: 16),
            _buildField(
              controller: _nameController,
              label: '이름',
              hint: '이름을 입력하세요',
              icon: Icons.person,
              keyboardType: TextInputType.text,
            ),
            const SizedBox(height: 16),
            _buildField(
              controller: _heightController,
              label: '키 (cm)',
              hint: '예: 170.5',
              icon: Icons.straighten,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
            ),
            const SizedBox(height: 16),
            GestureDetector(
              onTap: _pickDate,
              child: AbsorbPointer(
                child: _buildField(
                  controller: _birthDateController,
                  label: '생년월일',
                  hint: '날짜 선택',
                  icon: Icons.calendar_today,
                  keyboardType: TextInputType.datetime,
                ),
              ),
            ),
            const SizedBox(height: 24),
            Row(
              children: [
                Expanded(
                  child: _buildField(
                    controller: _visionLeftController,
                    label: '좌안 시력',
                    hint: '예: 1.0',
                    icon: Icons.remove_red_eye,
                    keyboardType: const TextInputType.numberWithOptions(decimal: true),
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: _buildField(
                    controller: _visionRightController,
                    label: '우안 시력',
                    hint: '예: 1.0',
                    icon: Icons.remove_red_eye_outlined,
                    keyboardType: const TextInputType.numberWithOptions(decimal: true),
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
            onPressed: _isLoading ? null : _save,
            style: ElevatedButton.styleFrom(
              backgroundColor: const Color(0xFF3B82F6),
              padding: const EdgeInsets.symmetric(vertical: 16),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(30),
              ),
              elevation: 0,
            ),
            child: const Text(
              '저장',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Colors.white),
            ),
          ),
        ),
      ),
    );
  }

  void _showImagePicker() {
    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.grey[900],
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (context) {
        return GridView.count(
          crossAxisCount: 3,
          shrinkWrap: true,
          padding: const EdgeInsets.all(16),
          children: _availableImages.map((path) {
            return GestureDetector(
              onTap: () {
                setState(() => _selectedImage = path);
                Navigator.pop(context);
              },
              child: Padding(
                padding: const EdgeInsets.all(8.0),
                child: CircleAvatar(
                  backgroundImage: AssetImage(path),
                  radius: 30,
                ),
              ),
            );
          }).toList(),
        );
      },
    );
  }

  Widget _buildField({
    required TextEditingController controller,
    required String label,
    required String hint,
    required IconData icon,
    required TextInputType keyboardType,
  }) {
    return TextField(
      controller: controller,
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
    );
  }
}