import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:intl/intl.dart';
import 'package:http/http.dart' as http;
import '../../services/auth_service.dart';
import '../../profile_select_screen.dart';
import '../../services/profile_cache_service.dart';

/// 🎨 공통 색상
class AppColors {
  static const background = Color(0xFF0D1117); // 전체 배경
  static const card = Color(0xFF161B22);       // 카드/입력창 배경
  static const border = Color(0xFF334155);     // 테두리
  static const primary = Color(0xFF3B82F6);    // 포인트
  static const text = Colors.white;
  static const textSub = Colors.white70;
  static const textHint = Colors.white38;
}

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
      if (cached == null || cached['id'] == null) {
        setState(() => _isLoading = false);
        return;
      }
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
              if (_birthDate != null) {
                _birthDateController.text = DateFormat('yyyy-MM-dd').format(_birthDate!);
              }
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
      backgroundColor: AppColors.card,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (_) {
        return Container(
          height: 300,
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
          child: Column(
            children: [
              const Text('생년월일 선택',
                  style: TextStyle(color: AppColors.text, fontSize: 18, fontWeight: FontWeight.w600)),
              const SizedBox(height: 12),
              Expanded(
                child: Row(
                  children: [
                    Expanded(
                      child: CupertinoPicker(
                        scrollController: FixedExtentScrollController(initialItem: selectedYear - 1900),
                        itemExtent: 40,
                        onSelectedItemChanged: (i) => selectedYear = 1900 + i,
                        children: List.generate(
                          126,
                              (i) => const Center(child: Text('', style: TextStyle(color: AppColors.text)))
                              .copyWithText('${1900 + i}년'),
                        ),
                      ),
                    ),
                    Expanded(
                      child: CupertinoPicker(
                        scrollController: FixedExtentScrollController(initialItem: selectedMonth - 1),
                        itemExtent: 40,
                        onSelectedItemChanged: (i) => selectedMonth = i + 1,
                        children: List.generate(
                          12,
                              (i) => const Center(child: Text('', style: TextStyle(color: AppColors.text)))
                              .copyWithText('${i + 1}월'),
                        ),
                      ),
                    ),
                    Expanded(
                      child: CupertinoPicker(
                        scrollController: FixedExtentScrollController(initialItem: selectedDay - 1),
                        itemExtent: 40,
                        onSelectedItemChanged: (i) => selectedDay = i + 1,
                        children: List.generate(
                          31,
                              (i) => const Center(child: Text('', style: TextStyle(color: AppColors.text)))
                              .copyWithText('${i + 1}일'),
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
                      _birthDateController.text = DateFormat('yyyy-MM-dd').format(_birthDate!);
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
        );
      },
    );
  }

  void _showImagePicker() {
    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: AppColors.card,
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
                      color: AppColors.text,
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
                                  border: Border.all(
                                    color: isSel ? AppColors.primary : Colors.transparent,
                                    width: 3,
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
                              SizedBox(
                                width: 70,
                                child: Text(
                                  path == null
                                      ? ''
                                      : _avatarNames[path.split('/').last.split('.').first] ?? '아바타',
                                  style: const TextStyle(color: AppColors.textSub, fontSize: 13),
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
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('프로필이 저장되었습니다.')));
      Navigator.pop(context);
    } else {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('수정 실패: ${res.statusCode}')));
    }
  }

  // ✅ 삭제 다이얼로그: 왼쪽 빨간 '삭제', 오른쪽 '취소' (다른 곳과 통일)
  Future<void> _confirmDelete() async {
    final name = _nameController.text.isNotEmpty ? _nameController.text : '이';
    final ok = await showDialog<bool>(
      context: context,
      barrierDismissible: true,
      barrierColor: Colors.black.withOpacity(0.6),
      builder: (_) => Dialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        backgroundColor: AppColors.card,
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text(
                '프로필 삭제',
                style: TextStyle(
                  color: AppColors.text,
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  fontFamily: 'Pretendard',
                ),
              ),
              const SizedBox(height: 12),
              Text(
                '$name님의 프로필을 삭제하시겠습니까?',
                style: const TextStyle(color: AppColors.textSub, fontFamily: 'Pretendard'),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 24),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () => Navigator.pop(context, true),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '삭제',
                        style: TextStyle(
                          color: Colors.white,
                          fontSize: 16,
                          fontWeight: FontWeight.w600,
                          fontFamily: 'Pretendard',
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(context, false),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: AppColors.primary),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '취소',
                        style: TextStyle(
                          color: AppColors.primary,
                          fontSize: 16,
                          fontFamily: 'Pretendard',
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
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
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('프로필이 삭제되었습니다.')));
      Navigator.pushAndRemoveUntil(
        context,
        MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
            (r) => false,
      );
    } else {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('삭제 실패: ${res.statusCode}')));
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
        title: const Text('프로필 수정', style: TextStyle(color: AppColors.text)),
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
                  backgroundColor: const Color(0xFF1F2937),
                  backgroundImage: _selectedImage != null ? AssetImage(_selectedImage!) : null,
                  child: _selectedImage == null
                      ? const Icon(Icons.person, size: 50, color: Colors.white30)
                      : null,
                ),
              ),
            ),
            const SizedBox(height: 16),
            _buildField(_nameController, '이름', '이름을 입력하세요', Icons.person, TextInputType.text),
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
              backgroundColor: AppColors.primary,
              padding: const EdgeInsets.symmetric(vertical: 16),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(30)),
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

  Widget _buildField(
      TextEditingController c,
      String label,
      String hint,
      IconData icon,
      TextInputType kt,
      ) =>
      TextField(
        controller: c,
        style: const TextStyle(color: AppColors.text),
        keyboardType: kt,
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
      );
}

/// 작은 헬퍼: Center(child: Text('')) 패턴에 텍스트만 쉽게 바꾸기
extension _CopyWithText on Widget {
  Widget copyWithText(String text) {
    return Builder(builder: (_) {
      if (this is Center && (this as Center).child is Text) {
        final base = (this as Center);
        final baseText = base.child as Text;
        return Center(child: Text(text, style: baseText.style));
      }
      return this;
    });
  }
}
