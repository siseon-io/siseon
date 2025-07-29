import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import '../../../login_screen.dart';
import '../../../profile_select_screen.dart';
import '../../services/auth_service.dart';
import '../../services/profile_cache_service.dart';
import 'edit_profile.dart';
import 'preset_page.dart';
import 'update_page.dart';

class ProfilePage extends StatefulWidget {
  const ProfilePage({super.key});

  @override
  State<ProfilePage> createState() => _ProfilePageState();
}

class _ProfilePageState extends State<ProfilePage> {
  String? _name;
  String? _imageUrl;
  int? _selectedProfileId;
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _loadCachedProfile();
    _fetchProfile();
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
        }
      } else {
        print('❌ 프로필 요청 실패: ${response.statusCode}');
        setState(() => _isLoading = false);
      }
    } catch (e) {
      print('❌ 예외 발생: $e');
      setState(() => _isLoading = false);
    }
  }

  /// 회원 탈퇴 API 호출
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

        Navigator.pushAndRemoveUntil(
          context,
          MaterialPageRoute(builder: (_) => const LoginScreen()),
              (route) => false,
        );
      } else {
        print('❌ 회원 탈퇴 실패: ${response.statusCode}');
      }
    } catch (e) {
      print('❌ 예외 발생: $e');
    }
  }

  /// 회원 탈퇴 팝업
  void _showDeleteUserDialog() {
    showDialog(
      context: context,
      barrierDismissible: true, // 바깥 클릭 시 취소
      builder: (_) => Dialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        child: Container(
          padding: const EdgeInsets.all(20),
          decoration: BoxDecoration(
            color: Colors.white,
            borderRadius: BorderRadius.circular(16),
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text(
                '정말 탈퇴하시겠습니까?',
                style: TextStyle(
                  fontFamily: 'Pretendard',
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: Colors.black,
                ),
              ),
              const SizedBox(height: 12),
              const Text(
                '탈퇴 시 모든 데이터가 삭제됩니다.',
                style: TextStyle(
                  fontFamily: 'Pretendard',
                  fontSize: 15,
                  color: Colors.black54,
                ),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 24),
              Row(
                children: [
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(context),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: Color(0xFF3B82F6)),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '취소',
                        style: TextStyle(
                          fontFamily: 'Pretendard',
                          fontSize: 16,
                          color: Color(0xFF3B82F6),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () {
                        Navigator.pop(context);
                        _deleteUser();
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
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
                ],
              )
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildMenuItem(IconData icon, String text, VoidCallback onTap) {
    return ListTile(
      leading: CircleAvatar(
        radius: 20,
        backgroundColor: const Color(0xFFE5E7EB),
        child: Icon(icon, color: const Color(0xFF2563FF)),
      ),
      title: Text(text, style: const TextStyle(fontSize: 16, fontFamily: 'Pretendard')),
      trailing: const Icon(Icons.chevron_right, color: Colors.grey),
      onTap: onTap,
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Colors.white,
        elevation: 0,
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
      body: SafeArea(
        child: Material(
          color: Colors.white,
          child: _isLoading
              ? const Center(child: CircularProgressIndicator())
              : ListView(
            padding: const EdgeInsets.symmetric(vertical: 24),
            children: [
              Center(
                child: CircleAvatar(
                  radius: 50,
                  backgroundColor: const Color(0xFFE5E7EB),
                  backgroundImage: (_imageUrl != null && _imageUrl!.isNotEmpty)
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
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: Colors.black,
                  ),
                ),
              ),
              const SizedBox(height: 32),
              _buildMenuItem(Icons.account_circle, '프로필 변경', () {
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
                );
              }),
              _buildMenuItem(Icons.edit, '프로필 수정', () {
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (_) => const EditProfilePage()),
                ).then((_) => _fetchProfile());
              }),
              _buildMenuItem(Icons.favorite, '프리셋', () {
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (_) => const PresetPage()),
                );
              }),
              _buildMenuItem(Icons.system_update_alt, '펌웨어 업데이트', () {
                Navigator.push(
                  context,
                  MaterialPageRoute(builder: (_) => const UpdatePage()),
                );
              }),
              _buildMenuItem(Icons.logout, '로그아웃', () {
                _showLogoutDialog();
              }),
            ],
          ),
        ),
      ),
    );
  }

  void _showLogoutDialog() {
    showDialog(
      context: context,
      barrierDismissible: true,
      builder: (_) => Dialog(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        child: Container(
          padding: const EdgeInsets.all(20),
          decoration: BoxDecoration(
            color: Colors.white,
            borderRadius: BorderRadius.circular(16),
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const Text(
                '로그아웃 하시겠습니까?',
                style: TextStyle(
                  fontFamily: 'Pretendard',
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: Colors.black,
                ),
              ),
              const SizedBox(height: 24),
              Row(
                children: [
                  Expanded(
                    child: OutlinedButton(
                      onPressed: () => Navigator.pop(context),
                      style: OutlinedButton.styleFrom(
                        side: const BorderSide(color: Color(0xFF3B82F6)),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                        padding: const EdgeInsets.symmetric(vertical: 14),
                      ),
                      child: const Text(
                        '취소',
                        style: TextStyle(
                          fontFamily: 'Pretendard',
                          fontSize: 16,
                          color: Color(0xFF3B82F6),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: ElevatedButton(
                      onPressed: () {
                        Navigator.pop(context);
                        Navigator.pushAndRemoveUntil(
                          context,
                          MaterialPageRoute(builder: (_) => const LoginScreen()),
                              (route) => false,
                        );
                      },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: const Color(0xFF3B82F6),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
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
                ],
              )
            ],
          ),
        ),
      ),
    );
  }
}
