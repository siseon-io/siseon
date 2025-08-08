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

  final Map<String, String> _avatarNames = {
    'profile_frog': '개구리',
    'profile_cat': '고양이',
    'profile_dog': '강아지',
    'profile_lion': '사자',
    'profile_mouse': '쥐',
    'profile_rabbit': '토끼',
  };
  final List<String?> _availableImages = [
    null,
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
    if (token == null) return setState(() => _isLoading = false);

    try {
      final cached = await ProfileCacheService.loadProfile();
      if (cached == null || cached['id'] == null)
        return setState(() => _isLoading = false);
      final id = cached['id'];

      final res = await http.get(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile'),
        headers: {'Authorization': 'Bearer $token'},
      );
      if (res.statusCode == 200) {
        final js = jsonDecode(utf8.decode(res.bodyBytes));
        if (js is List && js.isNotEmpty) {
          final user = js.firstWhere((e) => e['id'] == id, orElse: () => js.first);
          setState(() {
            _profileId = user['id'];
            _nameController.text = user['name'] ?? '';
            _heightController.text = user['height']?.toString() ?? '';
            _visionLeftController.text = user['leftVision']?.toString() ?? '';
            _visionRightController.text = user['rightVision']?.toString() ?? '';
            if (user['birthDate'] != null) {
              _birthDate = DateTime.tryParse(user['birthDate']);
              _birthDateController.text = DateFormat('yyyy-MM-dd').format(_birthDate!);
            }
            _selectedImage = user['imageUrl'];
          });
        }
      }
    } catch (_) {}
    setState(() => _isLoading = false);
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
        return Container(
          height: 300,                // 고정 높이
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
          child: Column(
            children: [
              const Text('생년월일 선택', style: TextStyle(color: Colors.white, fontSize: 18)),
              const SizedBox(height: 12),
              Expanded(
                child: Row(
                  children: [
                    Expanded(
                      child: CupertinoPicker(
                        scrollController:
                        FixedExtentScrollController(initialItem: selectedYear - 1900),
                        itemExtent: 40,
                        onSelectedItemChanged: (i) => selectedYear = 1900 + i,
                        children: List.generate(
                          126,
                              (i) => Center(
                            child: Text('${1900 + i}년', style: const TextStyle(color: Colors.white)),
                          ),
                        ),
                      ),
                    ),
                    Expanded(
                      child: CupertinoPicker(
                        scrollController:
                        FixedExtentScrollController(initialItem: selectedMonth - 1),
                        itemExtent: 40,
                        onSelectedItemChanged: (i) => selectedMonth = i + 1,
                        children: List.generate(
                          12,
                              (i) => Center(
                            child: Text('${i + 1}월', style: const TextStyle(color: Colors.white)),
                          ),
                        ),
                      ),
                    ),
                    Expanded(
                      child: CupertinoPicker(
                        scrollController:
                        FixedExtentScrollController(initialItem: selectedDay - 1),
                        itemExtent: 40,
                        onSelectedItemChanged: (i) => selectedDay = i + 1,
                        children: List.generate(
                          31,
                              (i) => Center(
                            child: Text('${i + 1}일', style: const TextStyle(color: Colors.white)),
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              SafeArea(
                child: ElevatedButton(
                  onPressed: () {
                    setState(() {
                      _birthDate = DateTime(selectedYear, selectedMonth, selectedDay);
                      _birthDateController.text =
                          DateFormat('yyyy-MM-dd').format(_birthDate!);
                    });
                    Navigator.pop(context);
                  },
                  style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFF3B82F6)),
                  child: const Text('확인'),
                ),
              ),
            ],
          ),
        );
      },
    );
  }
  Widget _yearPicker(ValueChanged<int> onChanged, int init) =>
      CupertinoPicker(
        scrollController: FixedExtentScrollController(initialItem: init),
        itemExtent: 40,
        onSelectedItemChanged: onChanged,
        children: List.generate(
          126,
              (i) => Center(
            child: Text('${1900 + i}년',
                style: const TextStyle(color: Colors.white)),
          ),
        ),
      );

  Widget _monthPicker(ValueChanged<int> onChanged, int init) =>
      CupertinoPicker(
        scrollController: FixedExtentScrollController(initialItem: init),
        itemExtent: 40,
        onSelectedItemChanged: onChanged,
        children: List.generate(
          12,
              (i) => Center(
            child: Text('${i + 1}월',
                style: const TextStyle(color: Colors.white)),
          ),
        ),
      );

  Widget _dayPicker(ValueChanged<int> onChanged, int init) =>
      CupertinoPicker(
        scrollController: FixedExtentScrollController(initialItem: init),
        itemExtent: 40,
        onSelectedItemChanged: onChanged,
        children: List.generate(
          31,
              (i) => Center(
            child: Text('${i + 1}일',
                style: const TextStyle(color: Colors.white)),
          ),
        ),
      );

  void _showImagePicker() {
    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: Colors.black,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (ctx) {
        return FractionallySizedBox(
          heightFactor: 0.5,
          child: SafeArea(
            child: Container(
              padding: const EdgeInsets.all(20),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    '아바타 선택',
                    style: TextStyle(
                      fontFamily: 'Pretendard',
                      fontSize: 18,
                      fontWeight: FontWeight.bold,
                      color: Colors.white,
                    ),
                  ),
                  const SizedBox(height: 16),
                  Expanded(
                    child: GridView.builder(
                      gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
                        crossAxisCount: 3,
                        crossAxisSpacing: 16,
                        mainAxisSpacing: 16,
                        childAspectRatio: 0.9,
                      ),
                      itemCount: _availableImages.length,
                      itemBuilder: (context, idx) {
                        final path = _availableImages[idx];
                        final isSel = path == _selectedImage;
                        return GestureDetector(
                          onTap: () {
                            setState(() => _selectedImage = path);
                            Navigator.pop(ctx);
                          },
                          child: Column(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              Container(
                                decoration: BoxDecoration(
                                  shape: BoxShape.circle,
                                  border: isSel
                                      ? Border.all(color: const Color(0xFF3B82F6), width: 3)
                                      : null,
                                ),
                                child: CircleAvatar(
                                  radius: 40,
                                  backgroundColor: Colors.grey[900],
                                  backgroundImage: path != null ? AssetImage(path) : null,
                                  child: path == null
                                      ? const Icon(Icons.person_off, size: 30, color: Colors.grey)
                                      : null,
                                ),
                              ),
                              const SizedBox(height: 6),
                              SizedBox(
                                width: 70,
                                child: Text(
                                  path == null
                                      ? ''
                                      : _avatarNames[path.split('/').last.split('.').first] ?? '아바타',
                                  style: const TextStyle(color: Colors.white70, fontSize: 13),
                                  overflow: TextOverflow.ellipsis,
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
            ),
          ),
        );
      },
    );
  }


  Future<void> _save() async {
    final t = await AuthService.getValidAccessToken();
    if (t == null || _profileId == null) return;
    final body = jsonEncode({
      "name": _nameController.text.trim(),
      "height": double.tryParse(_heightController.text.trim()),
      "birthDate": _birthDate?.toIso8601String(),
      "leftVision": double.tryParse(_visionLeftController.text.trim()),
      "rightVision": double.tryParse(_visionRightController.text.trim()),
      "imageUrl": _selectedImage,
    });
    final res = await http.put(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile/$_profileId'),
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer $t',
      },
      body: body,
    );
    if (res.statusCode == 200 && mounted) {
      ScaffoldMessenger.of(context)
          .showSnackBar(const SnackBar(content: Text('프로필이 저장되었습니다.')));
      Navigator.pop(context);
    } else {
      ScaffoldMessenger.of(context)
          .showSnackBar(SnackBar(content: Text('수정 실패: ${res.statusCode}')));
    }
  }

  Future<void> _confirmDelete() async {
    final name = _nameController.text.isNotEmpty ? _nameController.text : '이';
    final ok = await showDialog<bool>(
      context: context,  // ← 여기도 context
      builder: (context) => AlertDialog(
        title: const Text('프로필 삭제'),
        content: Text('$name님의 프로필을 삭제하시겠습니까?'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context, true),  // ← context
            child: const Text('예', style: TextStyle(color: Colors.red)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, false), // ← context
            child: const Text('아니오'),
          ),
        ],
      ),
    );
    if (ok == true) {
      await _deleteProfile();
    }
  }

  Future<void> _deleteProfile() async {
    final t = await AuthService.getValidAccessToken();
    if (t == null || _profileId == null) return;
    final res = await http.delete(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile/$_profileId'),
      headers: {'Authorization': 'Bearer $t'},
    );
    if (res.statusCode == 204 && mounted) {
      ScaffoldMessenger.of(context)
          .showSnackBar(const SnackBar(content: Text('프로필이 삭제되었습니다.')));
      Navigator.pushAndRemoveUntil(
          context,
          MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
              (r) => false);
    } else {
      ScaffoldMessenger.of(context)
          .showSnackBar(SnackBar(content: Text('삭제 실패: ${res.statusCode}')));
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
            onPressed: _confirmDelete,
            child: const Text('삭제', style: TextStyle(color: Colors.red)),
          ),
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
                  backgroundColor: Colors.grey[700],
                  backgroundImage:
                  _selectedImage != null ? AssetImage(_selectedImage!) : null,
                  child: _selectedImage == null
                      ? const Icon(Icons.person, size: 50, color: Colors.white30)
                      : null,
                ),
              ),
            ),
            const SizedBox(height: 16),
            _buildField(
              _nameController,
              '이름',
              '이름을 입력하세요',
              Icons.person,
              TextInputType.text,
            ),
            const SizedBox(height: 16),
            _buildField(
              _heightController,
              '키 (cm)',
              '예: 170.5',
              Icons.straighten,
              const TextInputType.numberWithOptions(decimal: true),
            ),
            const SizedBox(height: 16),
            GestureDetector(
              onTap: _pickDate,
              child: AbsorbPointer(
                child: _buildField(
                  _birthDateController,
                  '생년월일',
                  '날짜 선택',
                  Icons.calendar_today,
                  TextInputType.datetime,
                ),
              ),
            ),
            const SizedBox(height: 24),
            Row(
              children: [
                Expanded(
                  child: _buildField(
                    _visionLeftController,
                    '좌안 시력',
                    '예: 1.0',
                    Icons.remove_red_eye,
                    const TextInputType.numberWithOptions(decimal: true),
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: _buildField(
                    _visionRightController,
                    '우안 시력',
                    '예: 1.0',
                    Icons.remove_red_eye_outlined,
                    const TextInputType.numberWithOptions(decimal: true),
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
  Widget _buildField(TextEditingController c, String label, String hint,
      IconData icon, TextInputType kt) =>
      TextField(
        controller: c,
        style: const TextStyle(color: Colors.white),
        keyboardType: kt,
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