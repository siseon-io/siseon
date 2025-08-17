import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:flutter/services.dart'; // ✅ AssetManifest, TextInputFormatter
import 'package:intl/intl.dart';
import 'package:http/http.dart' as http;
import '../../services/auth_service.dart';
import '../../profile_select_screen.dart';
import '../../services/profile_cache_service.dart';
import 'dart:math' as math;

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

/// ✅ 시력 입력 전용 포매터
/// - 맨 앞의 '-' 0~1개 허용
/// - 숫자, '.' 0~1개 허용
/// - 편집 중 공란/단일 '-'도 허용(타이핑 자연스럽게)
class _SignedDecimalFormatter extends TextInputFormatter {
  final RegExp _regex = RegExp(r'^-?\d*\.?\d*$');
  @override
  TextEditingValue formatEditUpdate(
      TextEditingValue oldValue, TextEditingValue newValue) {
    final t = newValue.text;
    if (t.isEmpty || t == '-' || _regex.hasMatch(t)) {
      return newValue;
    }
    return oldValue;
  }
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

  String? _selectedImage; // asset path (또는 null)

  // ✅ avatars 폴더 자동 스캔
  List<String> _avatarAssets = [];
  bool _avatarsLoaded = false;

  // ✅ 폴더 비어있을 때 폴백(생성 화면과 동일 셋)
  static const List<String> _fallbackAvatars = [
    'assets/images/profile_blueman.png',
    'assets/images/profile_glassblueman.png',
    'assets/images/profile_purpleman.png',
    'assets/images/profile_purplegirl.png',
    'assets/images/profile_lightblueman.png',
    'assets/images/profile_circlepurplegirl.png',
    'assets/images/profile_lightpurplegirl.png',
    'assets/images/profile_bluegirl.png',
  ];

  @override
  void initState() {
    super.initState();
    _loadAvatarAssets();
    _fetchProfile();
  }

  Future<void> _loadAvatarAssets() async {
    try {
      final manifestJson = await rootBundle.loadString('AssetManifest.json');
      final Map<String, dynamic> manifest = jsonDecode(manifestJson);

      final candidates = manifest.keys.where((k) {
        final lower = k.toLowerCase();
        final isUnderAvatars = lower.startsWith('assets/images/avatars/');
        final isImage = lower.endsWith('.png') ||
            lower.endsWith('.jpg') ||
            lower.endsWith('.jpeg') ||
            lower.endsWith('.webp');
        return isUnderAvatars && isImage;
      }).toList()
        ..sort();

      setState(() {
        _avatarAssets = candidates.isNotEmpty ? candidates : _fallbackAvatars;
        _avatarsLoaded = true;
      });
    } catch (_) {
      setState(() {
        _avatarAssets = _fallbackAvatars;
        _avatarsLoaded = true;
      });
    }
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
      final id = cached['id'];

      final res = await http.get(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/profile'),
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
    } catch (_) {
      // ignore
    }
    setState(() => _isLoading = false);
  }

  void _pickDate() {
    final now = DateTime.now();
    const startYear = 1900;
    final endYear = now.year;

    int selectedYear = (_birthDate?.year ?? 2000).clamp(startYear, endYear);
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
          child: StatefulBuilder(
            builder: (ctx, setModal) {
              final yearCount = endYear - startYear + 1;
              final daysInMonth = DateTime(
                selectedMonth == 12 ? selectedYear + 1 : selectedYear,
                selectedMonth == 12 ? 1 : selectedMonth + 1,
                0,
              ).day;
              if (selectedDay > daysInMonth) selectedDay = daysInMonth;

              return Container(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Text('생년월일 선택',
                        style: TextStyle(color: AppColors.text, fontSize: 18, fontWeight: FontWeight.w600)),
                    const SizedBox(height: 12),
                    SizedBox(
                      height: 200,
                      child: Row(
                        children: [
                          Expanded(
                            child: CupertinoPicker(
                              scrollController: FixedExtentScrollController(
                                initialItem: (selectedYear - startYear).clamp(0, yearCount - 1),
                              ),
                              itemExtent: 40,
                              onSelectedItemChanged: (i) => setModal(() => selectedYear = startYear + i),
                              children: List.generate(
                                yearCount,
                                    (i) => Center(
                                  child: Text('${startYear + i}년', style: const TextStyle(color: AppColors.text)),
                                ),
                              ),
                            ),
                          ),
                          Expanded(
                            child: CupertinoPicker(
                              scrollController: FixedExtentScrollController(initialItem: selectedMonth - 1),
                              itemExtent: 40,
                              onSelectedItemChanged: (i) => setModal(() => selectedMonth = i + 1),
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
                              scrollController: FixedExtentScrollController(
                                initialItem: (selectedDay - 1).clamp(0, daysInMonth - 1),
                              ),
                              itemExtent: 40,
                              onSelectedItemChanged: (i) => setModal(() => selectedDay = i + 1),
                              children: List.generate(
                                daysInMonth,
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
          ),
        );
      },
    );
  }

  // ✅ 생성 화면과 동일한 아바타 선택 UI
  void _showAvatarPicker() {
    if (!_avatarsLoaded) return;

    showModalBottomSheet(
      context: context,
      backgroundColor: AppColors.card,
      isScrollControlled: true,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (ctx) {
        final media = MediaQuery.of(ctx);
        final sheetHeight = math.min(media.size.height * 0.40, 400.0);

        return SizedBox(
          height: sheetHeight,
          child: Padding(
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
                  child: LayoutBuilder(
                    builder: (context, constraints) {
                      const int crossAxisCount = 3;
                      const double spacing = 12;
                      final double maxW = constraints.maxWidth;

                      final double tileWidth =
                          (maxW - spacing * (crossAxisCount - 1)) / crossAxisCount;

                      final double avatarOuter = math.min(tileWidth, 84);
                      const double borderSelected = 3;
                      const double borderNormal = 1;
                      final double tileExtent = avatarOuter + 4;

                      return GridView.builder(
                        padding: EdgeInsets.zero,
                        physics: const ClampingScrollPhysics(),
                        primary: false,
                        gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                          crossAxisCount: crossAxisCount,
                          crossAxisSpacing: spacing,
                          mainAxisSpacing: spacing,
                          mainAxisExtent: tileExtent,
                        ),
                        itemCount: 1 + _avatarAssets.length, // +1: '없음'
                        itemBuilder: (context, index) {
                          final String? path = (index == 0) ? null : _avatarAssets[index - 1];
                          final bool isSelected = path == _selectedImage;
                          final double borderWidth = isSelected ? borderSelected : borderNormal;
                          final double radius = avatarOuter / 2 - borderWidth;

                          return GestureDetector(
                            onTap: () {
                              setState(() => _selectedImage = path);
                              Navigator.pop(ctx);
                            },
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                Container(
                                  width: avatarOuter,
                                  height: avatarOuter,
                                  decoration: BoxDecoration(
                                    shape: BoxShape.circle,
                                    border: Border.all(
                                      color: isSelected ? AppColors.primary : AppColors.border,
                                      width: borderWidth,
                                    ),
                                  ),
                                  child: CircleAvatar(
                                    radius: radius,
                                    backgroundColor: const Color(0xFF1F2937),
                                    backgroundImage: path != null ? AssetImage(path) : null,
                                    child: path == null
                                        ? const Icon(Icons.person_off, size: 24, color: Colors.grey)
                                        : null,
                                  ),
                                ),
                              ],
                            ),
                          );
                        },
                      );
                    },
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }

  Future<void> _save() async {
    final t = await AuthService.getValidAccessToken();
    if (t == null || _profileId == null) return;

    final payload = {
      "name": _nameController.text.trim(),
      "height": double.tryParse(_heightController.text.trim()),
      "birthDate": _birthDate?.toIso8601String(),
      "leftVision": double.tryParse(_visionLeftController.text.trim()),
      "rightVision": double.tryParse(_visionRightController.text.trim()),
      "imageUrl": _selectedImage, // assets 경로 또는 null
    };

    final res = await http.put(
      Uri.parse('https://i13b101.p.ssafy.io/siseon/api/profile/$_profileId'),
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer $t',
      },
      body: jsonEncode(payload),
    );

    if (!mounted) return;

    if (res.statusCode == 200) {
      Map<String, dynamic> fromServer = {};
      try {
        final js = jsonDecode(utf8.decode(res.bodyBytes));
        if (js is Map<String, dynamic>) fromServer = js;
      } catch (_) {}

      final cached = await ProfileCacheService.loadProfile() ?? {};
      final updated = <String, dynamic>{
        ...cached,
        ...fromServer,
        'id': _profileId,
        'profileId': _profileId,
        'name': payload['name'],
        'height': payload['height'],
        'leftVision': payload['leftVision'],
        'rightVision': payload['rightVision'],
        'birthDate': _birthDate != null
            ? DateFormat('yyyy-MM-dd').format(_birthDate!)
            : (fromServer['birthDate'] ?? cached['birthDate']),
        'imageUrl': payload['imageUrl'],
      };

      await ProfileCacheService.saveProfile(updated);

      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('프로필이 저장되었습니다.')),
      );
      Navigator.pop(context, updated);
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('수정 실패: ${res.statusCode}')),
      );
    }
  }

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
      Uri.parse('https://i13b101.p.ssafy.io/siseon/api/profile/$_profileId'),
      headers: {'Authorization': 'Bearer $t'},
    );
    if (!mounted) return;

    if (res.statusCode == 204) {
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
              child: InkWell(
                onTap: _avatarsLoaded ? _showAvatarPicker : null,
                borderRadius: BorderRadius.circular(64),
                child: Stack(
                  alignment: Alignment.center,
                  children: [
                    CircleAvatar(
                      radius: 50,
                      backgroundColor: const Color(0xFF1F2937),
                      backgroundImage: _selectedImage != null ? AssetImage(_selectedImage!) : null,
                      child: _selectedImage == null
                          ? const Icon(Icons.person, size: 50, color: Colors.white30)
                          : null,
                    ),
                    if (!_avatarsLoaded)
                      const SizedBox(
                        width: 100,
                        height: 100,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          color: AppColors.primary,
                        ),
                      ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 16),
            _buildField(_nameController, '이름', '이름을 입력하세요', Icons.person, TextInputType.text),
            const SizedBox(height: 16),
            _buildField(
              _heightController,
              '키 (cm)',
              '예: 170',
              Icons.straighten,
              TextInputType.number,
              inputFormatters: [FilteringTextInputFormatter.digitsOnly],
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
                    '예: -0.75',
                    Icons.remove_red_eye,
                    const TextInputType.numberWithOptions(signed: true, decimal: true), // ✅
                    inputFormatters: [_SignedDecimalFormatter()], // ✅
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: _buildField(
                    _visionRightController,
                    '우안 시력',
                    '예: -0.75',
                    Icons.remove_red_eye_outlined,
                    const TextInputType.numberWithOptions(signed: true, decimal: true), // ✅
                    inputFormatters: [_SignedDecimalFormatter()], // ✅
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
      TextInputType kt, {
        List<TextInputFormatter>? inputFormatters,
      }) {
    return TextField(
      controller: c,
      style: const TextStyle(color: AppColors.text),
      keyboardType: kt,
      inputFormatters: inputFormatters,
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
}
