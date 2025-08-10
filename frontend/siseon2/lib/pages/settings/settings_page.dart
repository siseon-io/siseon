import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

import '../../../login_screen.dart';
import '../../../profile_select_screen.dart';
import '../../services/auth_service.dart';
import '../../services/profile_cache_service.dart';

import 'edit_profile.dart';
import 'preset_page.dart';
import 'device_info.dart';
import 'package:siseon2/pages/settings/stats_page.dart'; // ✅ 통계 페이지 import

class SettingsPage extends StatefulWidget {
  const SettingsPage({super.key});

  @override
  State<SettingsPage> createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  String? _name;
  String? _imageUrl;
  int? _selectedProfileId;
  bool _isLoading = true;

  // 🎨 색상
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color cardGrey = Color(0xFF161B22);
  static const Color primaryBlue = Color(0xFF3B82F6);

  @override
  void initState() {
    super.initState();
    _loadCachedProfile();
    _fetchProfile();
  }

  // 👉 공통 슬라이드 전환
  Route<T> _slideRightToLeft<T>(Widget page) {
    return PageRouteBuilder<T>(
      pageBuilder: (_, __, ___) => page,
      transitionDuration: const Duration(milliseconds: 280),
      reverseTransitionDuration: const Duration(milliseconds: 240),
      transitionsBuilder: (_, animation, __, child) {
        final curved = CurvedAnimation(parent: animation, curve: Curves.easeInOut);
        return SlideTransition(
          position: Tween<Offset>(begin: const Offset(1, 0), end: Offset.zero).animate(curved),
          child: child,
        );
      },
    );
  }

  Future<void> _loadCachedProfile() async {
    final cached = await ProfileCacheService.loadProfile();
    if (cached != null) {
      setState(() {
        _name = cached['name'];
        _imageUrl = cached['imageUrl'];
        _selectedProfileId = cached['id'];
        _isLoading = false;
      });
    }
  }

  Future<void> _fetchProfile() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return;

    try {
      final response = await http.get(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile'),
        headers: {'Authorization': 'Bearer $token'},
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(utf8.decode(response.bodyBytes));
        if (data is List && data.isNotEmpty) {
          final selected = data.firstWhere(
                (e) => e['id'] == _selectedProfileId,
            orElse: () => data.first,
          );

          setState(() {
            _name = selected['name'];
            _imageUrl = selected['imageUrl'];
            _isLoading = false;
          });

          await ProfileCacheService.saveProfile(selected);
        } else {
          setState(() => _isLoading = false);
        }
      } else {
        setState(() => _isLoading = false);
      }
    } catch (_) {
      setState(() => _isLoading = false);
    }
  }

  // 🔐 회원 탈퇴 API 호출
  Future<void> _deleteUser() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return;

    try {
      final response = await http.delete(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/user'),
        headers: {'Authorization': 'Bearer $token'},
      );

      if (response.statusCode == 200 || response.statusCode == 204) {
        await AuthService.clearTokens();
        await ProfileCacheService.clearProfile();

        if (!mounted) return;
        Navigator.pushAndRemoveUntil(
          context,
          _slideRightToLeft(const LoginScreen()),
              (route) => false,
        );
      }
    } catch (_) {}
  }

  // ❗ 회원 탈퇴 확인 다이얼로그
  void _showDeleteUserDialog() {
    showDialog(
      context: context,
      barrierDismissible: true,
      barrierColor: Colors.black.withOpacity(0.6),
      builder: (_) => Dialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        backgroundColor: cardGrey,
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text(
                '정말 탈퇴하시겠습니까?',
                style: TextStyle(
                  fontFamily: 'Pretendard',
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: Colors.white,
                ),
              ),
              const SizedBox(height: 12),
              const Text(
                '탈퇴 시 모든 데이터가 삭제됩니다.',
                style: TextStyle(fontFamily: 'Pretendard', fontSize: 15, color: Colors.white70),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 24),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () {
                        Navigator.pop(context);
                        _deleteUser();
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '탈퇴',
                        style: TextStyle(
                          fontFamily: 'Pretendard',
                          fontSize: 16,
                          fontWeight: FontWeight.w600,
                          color: Colors.white,
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(context),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: primaryBlue),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '취소',
                        style: TextStyle(
                          fontFamily: 'Pretendard',
                          fontSize: 16,
                          color: primaryBlue,
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
  }

  // 🚪 로그아웃 다이얼로그
  void _showLogoutDialog() {
    showDialog(
      context: context,
      barrierDismissible: true,
      barrierColor: Colors.black.withOpacity(0.6),
      builder: (_) => Dialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        backgroundColor: cardGrey,
        child: Padding(
          padding: const EdgeInsets.all(20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text(
                '로그아웃 하시겠습니까?',
                style: TextStyle(
                  fontFamily: 'Pretendard',
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: Colors.white,
                ),
              ),
              const SizedBox(height: 24),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () {
                        Navigator.pop(context);
                        Navigator.pushAndRemoveUntil(
                          context,
                          _slideRightToLeft(const LoginScreen()),
                              (route) => false,
                        );
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '로그아웃',
                        style: TextStyle(
                          fontFamily: 'Pretendard',
                          fontSize: 16,
                          fontWeight: FontWeight.w600,
                          color: Colors.white,
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(context),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: primaryBlue),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '취소',
                        style: TextStyle(
                          fontFamily: 'Pretendard',
                          fontSize: 16,
                          color: primaryBlue,
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
  }

  Widget _buildMenuItem(IconData icon, String text, VoidCallback onTap) {
    return Container(
      margin: const EdgeInsets.symmetric(vertical: 6, horizontal: 16),
      decoration: BoxDecoration(
        color: cardGrey,
        borderRadius: BorderRadius.circular(16), // 홈과 비슷한 둥글기
        border: Border.all(                       // ✅ 연한 테두리 추가
          color: Colors.white.withOpacity(0.16),
          width: 1,
        ),
        boxShadow: [
          BoxShadow(                              // (옵션) 아주 약한 그림자
            color: Colors.black.withOpacity(0.18),
            blurRadius: 10,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: ListTile(
        leading: Icon(icon, color: primaryBlue),
        title: Text(
          text,
          style: const TextStyle(
            fontSize: 16,
            fontFamily: 'Pretendard',
            color: Colors.white,
          ),
        ),
        trailing: const Icon(Icons.chevron_right, color: Colors.white38),
        onTap: onTap,
        contentPadding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
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
        automaticallyImplyLeading: false,
        actions: [
          TextButton(
            onPressed: _showDeleteUserDialog,
            child: const Text(
              '회원 탈퇴',
              style: TextStyle(
                fontFamily: 'Pretendard',
                fontSize: 14,
                color: Colors.red,
              ),
            ),
          ),
        ],
      ),
      body: _isLoading
          ? const Center(child: CircularProgressIndicator(color: Colors.white))
          : ListView(
        children: [
          const SizedBox(height: 16),
          Center(
            child: CircleAvatar(
              radius: 50,
              backgroundColor: Colors.grey[800],
              backgroundImage:
              (_imageUrl != null && _imageUrl!.isNotEmpty)
                  ? AssetImage(_imageUrl!)
                  : null,
              child: (_imageUrl == null || _imageUrl!.isEmpty)
                  ? const Icon(Icons.person, size: 50, color: Colors.white)
                  : null,
            ),
          ),
          const SizedBox(height: 12),
          Center(
            child: Text(
              _name ?? '사용자',
              style: const TextStyle(
                fontFamily: 'Pretendard',
                fontSize: 20,
                fontWeight: FontWeight.bold,
                color: Colors.white,
              ),
            ),
          ),
          const SizedBox(height: 24),
          _buildMenuItem(Icons.account_circle, '프로필 변경', () {
            Navigator.pushReplacement(
                context, _slideRightToLeft(const ProfileSelectScreen()));
          }),
          _buildMenuItem(Icons.edit, '프로필 수정', () {
            Navigator.push(context, _slideRightToLeft(const EditProfilePage()))
                .then((_) => _fetchProfile());
          }),
          _buildMenuItem(Icons.bar_chart, '통계 보기', () {
            Navigator.push(context, _slideRightToLeft(const StatsPage()));
          }),
          _buildMenuItem(Icons.favorite, '프리셋', () {
            Navigator.push(context, _slideRightToLeft(const PresetPage()));
          }),
          _buildMenuItem(Icons.system_update_alt, '펌웨어 업데이트', () {
            // 파라미터 필요 없도록 DeviceInfoPage 기본 생성자 사용
            Navigator.push(context, _slideRightToLeft(const DeviceInfoPage()));
          }),
          _buildMenuItem(Icons.logout, '로그아웃', _showLogoutDialog),
        ],
      ),
    );
  }
}
