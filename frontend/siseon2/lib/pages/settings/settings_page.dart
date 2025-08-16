import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:http/http.dart' as http;

import '../../../login_screen.dart';
import '../../../profile_select_screen.dart';
import '../../services/auth_service.dart';
import '../../services/profile_cache_service.dart';

import 'edit_profile.dart';
import 'preset_page.dart';
import 'device_info.dart';
import 'package:siseon2/pages/settings/stats_page.dart';

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

  // ✅ 네트워크 아바타 캐시버스트용
  int _avatarBust = 0;

  // ✅ 화면에 보일 때 캐시 변경 한 번만 반영
  bool _profileCheckScheduled = false;

  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color cardGrey = Color(0xFF161B22);
  static const Color primaryBlue = Color(0xFF3B82F6);

  @override
  void initState() {
    super.initState();
    _loadCachedProfile();
    _fetchProfile();
  }

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

  // ✅ 아바타 Provider (assets + http 지원, 캐시버스트)
  ImageProvider? _avatarProvider(String? s) {
    final v = (s ?? '').trim();
    if (v.isEmpty) return null;
    if (v.startsWith('http://') || v.startsWith('https://')) {
      final sep = v.contains('?') ? '&' : '?';
      final withBust = _avatarBust > 0 ? '$v${sep}v=$_avatarBust' : v;
      return NetworkImage(withBust);
    }
    if (v.startsWith('assets/')) {
      return AssetImage(v);
    }
    return null;
  }

  Future<void> _loadCachedProfile() async {
    final cached = await ProfileCacheService.loadProfile();
    if (cached != null) {
      setState(() {
        _name = cached['name'];
        _imageUrl = cached['imageUrl'];
        _selectedProfileId = cached['id'] ?? cached['profileId'];
        _isLoading = false;
        _avatarBust = DateTime.now().millisecondsSinceEpoch; // 새로고침
      });
    } else {
      setState(() => _isLoading = false);
    }
  }

  Future<void> _fetchProfile() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return;

    try {
      final response = await http.get(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/profile'),
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
            _selectedProfileId = selected['id'] ?? selected['profileId'];
            _isLoading = false;
            _avatarBust = DateTime.now().millisecondsSinceEpoch;
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

  // ✅ 화면 보일 때 캐시 변경 반영(한 프레임 뒤 1회)
  void _checkCachedProfileOnce() {
    if (_profileCheckScheduled) return;
    _profileCheckScheduled = true;
    SchedulerBinding.instance.addPostFrameCallback((_) async {
      _profileCheckScheduled = false;
      final cached = await ProfileCacheService.loadProfile();
      if (!mounted || cached == null) return;

      final newUrl = (cached['imageUrl'] ?? '').toString();
      final oldUrl = (_imageUrl ?? '');
      final changed = newUrl != oldUrl || (cached['name'] != _name);

      if (changed) {
        setState(() {
          _name = cached['name'];
          _imageUrl = cached['imageUrl'];
          _selectedProfileId = cached['id'] ?? cached['profileId'];
          _avatarBust = DateTime.now().millisecondsSinceEpoch; // 강제 새로고침
        });
      }
    });
  }

  // ✅ 푸시 토큰 해제 API 호출
  Future<void> _unregisterPush() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null || _selectedProfileId == null) return;

    try {
      final uri = Uri.parse(
        'https://i13b101.p.ssafy.io/siseon/api/push/unregister?profileId=$_selectedProfileId',
      );

      final response = await http.post(
        uri,
        headers: {
          'Authorization': 'Bearer $token',
          'Content-Type': 'application/json',
        },
      );

      if (response.statusCode >= 300) {
        debugPrint('🔥 unregister 실패: ${response.statusCode} ${response.body}');
      }
    } catch (e) {
      debugPrint('🔥 unregister 예외 발생: $e');
    }
  }

  Future<void> _deleteUser() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return;

    try {
      final response = await http.delete(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/user'),
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
                      onPressed: () async {
                        Navigator.pop(context);

                        await _unregisterPush(); // ✅ 푸시 등록 해제
                        await AuthService.clearTokens();
                        await ProfileCacheService.clearProfile();

                        if (!mounted) return;
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
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.white.withOpacity(0.16), width: 1),
        boxShadow: [
          BoxShadow(
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
    // ✅ 화면 보일 때 캐시 변경 한 번 반영
    _checkCachedProfileOnce();

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
        padding: EdgeInsets.only(
          bottom: 24 + MediaQuery.of(context).padding.bottom,
        ),
        children: [
          const SizedBox(height: 16),
          Center(
            child: CircleAvatar(
              radius: 50,
              backgroundColor: Colors.grey[800],
              // ✅ 캐시/네트워크/에셋 모두 지원 (+버스트)
              foregroundImage: _avatarProvider(_imageUrl),
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

          // 프로필 변경(선택)
          _buildMenuItem(Icons.account_circle, '프로필 변경', () async {
            await Navigator.push(
              context,
              _slideRightToLeft(const ProfileSelectScreen(allowBack: true)),
            );
            // ✅ 선택 후 캐시에서 즉시 반영
            await _loadCachedProfile();
          }),

          // 프로필 수정
          _buildMenuItem(Icons.edit, '프로필 수정', () async {
            final result = await Navigator.push<Map<String, dynamic>?>(
              context,
              _slideRightToLeft(const EditProfilePage()),
            );

            if (result != null) {
              // ✅ 수정 결과 즉시 반영 + 캐시 저장
              setState(() {
                _name = result['name'];
                _imageUrl = result['imageUrl'];
                _selectedProfileId = result['id'] ?? result['profileId'];
                _avatarBust = DateTime.now().millisecondsSinceEpoch;
              });
              await ProfileCacheService.saveProfile(result);
            } else {
              // 수정 페이지가 result를 안 주는 경우 서버/캐시 재동기화
              await _fetchProfile();
            }
          }),

          _buildMenuItem(Icons.bar_chart, '통계 보기', () {
            Navigator.push(
              context,
              _slideRightToLeft(const StatsPage()),
            );
          }),
          _buildMenuItem(Icons.favorite, '프리셋', () {
            Navigator.push(
              context,
              _slideRightToLeft(const PresetPage()),
            );
          }),
          _buildMenuItem(Icons.system_update_alt, '기기 정보', () {
            Navigator.push(
              context,
              _slideRightToLeft(const DeviceInfoPage()),
            );
          }),
          _buildMenuItem(Icons.logout, '로그아웃', _showLogoutDialog),

          const SizedBox(height: 24),
        ],
      ),
    );
  }
}
