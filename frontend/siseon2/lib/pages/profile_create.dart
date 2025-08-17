// lib/pages/profile_create_screen.dart
import 'dart:convert';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter/cupertino.dart';
import 'package:flutter/services.dart'; // rootBundle, InputFormatters
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

/// ✅ 시력 입력 전용 포매터 (음수/소수 허용, 자연스러운 타이핑)
/// - 맨 앞 '-' 0~1개 허용
/// - 숫자와 '.' 0~1개 허용
/// - 편집 중 공란/단일 '-'도 허용
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

class ProfileCreateScreen extends StatefulWidget {
  const ProfileCreateScreen({Key? key}) : super(key: key);

  @override
  State<ProfileCreateScreen> createState() => _ProfileCreateScreenState();
}

class _ProfileCreateScreenState extends State<ProfileCreateScreen> {
  String? _selectedAvatar; // asset path (또는 null)
  String _name = '';
  String _height = '';
  DateTime? _birthDate;
  String _visionLeft = '';
  String _visionRight = '';

  // ✅ 아바타 자산 자동 로딩 (assets/images/avatars/*)
  List<String> _avatarAssets = [];
  bool _avatarsLoaded = false;

  // 폴백(예전 이미지 셋) — avatars 폴더가 비어있을 때만 사용
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
  }

  Future<void> _loadAvatarAssets() async {
    try {
      final manifestJson = await rootBundle.loadString('AssetManifest.json');
      final Map<String, dynamic> manifest = jsonDecode(manifestJson);

      // assets/images/avatars/ 아래의 이미지 파일 자동 수집
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
      // manifest 읽기 실패 시 폴백 사용
      setState(() {
        _avatarAssets = _fallbackAvatars;
        _avatarsLoaded = true;
      });
    }
  }

  void _pickDate() {
    final now = DateTime.now();
    const startYear = 1900;
    final endYear = now.year;

    int selectedYear = _birthDate?.year.clamp(startYear, endYear) ?? 2000;
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
            child: StatefulBuilder(
              builder: (ctx, setModalState) {
                final yearCount = endYear - startYear + 1;
                final daysInMonth = DateTime(
                  selectedMonth == 12 ? selectedYear + 1 : selectedYear,
                  selectedMonth == 12 ? 1 : selectedMonth + 1,
                  0,
                ).day;
                if (selectedDay > daysInMonth) {
                  selectedDay = daysInMonth;
                }

                return Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Text(
                      '생년월일 선택',
                      style: TextStyle(
                        color: AppColors.text,
                        fontSize: 18,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                    const SizedBox(height: 12),
                    SizedBox(
                      height: 200,
                      child: Row(
                        children: [
                          // Year
                          Expanded(
                            child: CupertinoPicker(
                              scrollController: FixedExtentScrollController(
                                initialItem: (selectedYear - startYear)
                                    .clamp(0, yearCount - 1),
                              ),
                              itemExtent: 40,
                              onSelectedItemChanged: (index) {
                                setModalState(() {
                                  selectedYear = startYear + index;
                                });
                              },
                              children: List.generate(
                                yearCount,
                                    (i) => Center(
                                  child: Text(
                                    '${startYear + i}년',
                                    style: const TextStyle(
                                      color: AppColors.text,
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          // Month
                          Expanded(
                            child: CupertinoPicker(
                              scrollController: FixedExtentScrollController(
                                initialItem: selectedMonth - 1,
                              ),
                              itemExtent: 40,
                              onSelectedItemChanged: (index) {
                                setModalState(() {
                                  selectedMonth = index + 1;
                                });
                              },
                              children: List.generate(
                                12,
                                    (i) => Center(
                                  child: Text(
                                    '${i + 1}월',
                                    style: const TextStyle(
                                      color: AppColors.text,
                                    ),
                                  ),
                                ),
                              ),
                            ),
                          ),
                          // Day
                          Expanded(
                            child: CupertinoPicker(
                              scrollController: FixedExtentScrollController(
                                initialItem: (selectedDay - 1)
                                    .clamp(0, daysInMonth - 1),
                              ),
                              itemExtent: 40,
                              onSelectedItemChanged: (index) {
                                setModalState(() {
                                  selectedDay = index + 1;
                                });
                              },
                              children: List.generate(
                                daysInMonth,
                                    (i) => Center(
                                  child: Text(
                                    '${i + 1}일',
                                    style: const TextStyle(
                                      color: AppColors.text,
                                    ),
                                  ),
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
                            _birthDate = DateTime(
                              selectedYear,
                              selectedMonth,
                              selectedDay,
                            );
                          });
                          Navigator.pop(context);
                        },
                        style: ElevatedButton.styleFrom(
                          backgroundColor: AppColors.primary,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(12),
                          ),
                        ),
                        child: const Text(
                          '확인',
                          style: TextStyle(color: Colors.white),
                        ),
                      ),
                    ),
                  ],
                );
              },
            ),
          ),
        );
      },
    );
  }

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
        // 기존: 0.65, 480
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
                const SizedBox(height: 12),
                // 👉 반응형 그리드
                Expanded(
                  child: LayoutBuilder(
                    builder: (context, constraints) {
                      const int crossAxisCount = 3;     // 항상 3칸
                      const double spacing = 12;        // 동일 간격
                      final double maxW = constraints.maxWidth;

                      final double tileWidth =
                          (maxW - spacing * (crossAxisCount - 1)) / crossAxisCount;

                      // 동일 지름 cap
                      final double avatarOuter = math.min(tileWidth, 84);
                      const double borderSelected = 3;
                      const double borderNormal = 1;

                      // 동일 셀 세로 높이
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
                        itemCount: 1 + _avatarAssets.length,
                        itemBuilder: (context, index) {
                          final String? path = (index == 0) ? null : _avatarAssets[index - 1];
                          final bool isSelected = path == _selectedAvatar;
                          final double borderWidth = isSelected ? borderSelected : borderNormal;
                          final double radius = avatarOuter / 2 - borderWidth;

                          return GestureDetector(
                            onTap: () {
                              setState(() => _selectedAvatar = path);
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
                )

              ],
            ),
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
      Uri.parse('https://i13b101.p.ssafy.io/siseon/api/profile'),
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
        // 서버가 실제 URL을 요구한다면, 에셋 경로 대신 별도 키를 보내거나 업로드 후 URL을 보내도록 백엔드와 합의 필요
        "imageUrl": _selectedAvatar,
      }),
    );

    if (!mounted) return;

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
                onTap: _avatarsLoaded ? _showAvatarPicker : null,
                borderRadius: BorderRadius.circular(64),
                child: Stack(
                  alignment: Alignment.center,
                  children: [
                    CircleAvatar(
                      radius: 50,
                      backgroundColor: const Color(0xFF1F2937),
                      backgroundImage: _selectedAvatar != null ? AssetImage(_selectedAvatar!) : null,
                      child: _selectedAvatar == null
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
              inputFormatters: [FilteringTextInputFormatter.digitsOnly],
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
                    hint: '예: -0.75', // ✅ 힌트 업데이트
                    icon: Icons.visibility,
                    keyboardType: const TextInputType.numberWithOptions(
                      signed: true,
                      decimal: true,
                    ), // ✅ 안드로이드 마이너스 키 등장
                    onChanged: (v) => setState(() => _visionLeft = v),
                    inputFormatters: [_SignedDecimalFormatter()], // ✅ 음수/소수 포매터
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: _buildField(
                    label: '우안 시력',
                    hint: '예: -0.75', // ✅ 힌트 업데이트
                    icon: Icons.visibility,
                    keyboardType: const TextInputType.numberWithOptions(
                      signed: true,
                      decimal: true,
                    ), // ✅
                    onChanged: (v) => setState(() => _visionRight = v),
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
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Colors.white),
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
    List<TextInputFormatter>? inputFormatters,
  }) {
    return TextField(
      style: const TextStyle(color: AppColors.text),
      keyboardType: keyboardType,
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
      onChanged: onChanged,
    );
  }
}
