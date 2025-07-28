// 📁 lib/pages/settings/profile_page.dart

import 'package:flutter/material.dart';
import '../../../login_screen.dart';
import '../../../profile_select_screen.dart';
import 'edit_profile.dart';
import 'preset_page.dart';
import 'update_page.dart';

class ProfilePage extends StatelessWidget {
  const ProfilePage({super.key});

  Widget _buildMenuItem(IconData icon, String text, VoidCallback onTap) {
    return ListTile(
      leading: CircleAvatar(
        radius: 20,
        backgroundColor: const Color(0xFFE5E7EB),
        child: Icon(icon, color: const Color(0xFF2563FF)),
      ),
      title: Text(text, style: const TextStyle(fontSize: 16)),
      trailing: const Icon(Icons.chevron_right, color: Colors.grey),
      onTap: onTap,
    );
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Material(
        color: Colors.white,
        child: ListView(
          padding: const EdgeInsets.symmetric(vertical: 24),
          children: [
            // 프로필 사진 + 편집 아이콘
            Center(
              child: Stack(
                clipBehavior: Clip.none,
                children: [
                  const CircleAvatar(
                    radius: 50,
                    backgroundColor: Color(0xFFE5E7EB),
                    child: Icon(Icons.person, size: 50, color: Colors.white),
                  ),
                  Positioned(
                    bottom: -4,
                    right: -4,
                    child: GestureDetector(
                      onTap: () {
                        // TODO: 프로필 사진 편집 로직
                      },
                      child: CircleAvatar(
                        radius: 16,
                        backgroundColor: Colors.white,
                        child: Container(
                          decoration: const BoxDecoration(
                            color: Color(0xFF2563FF),
                            shape: BoxShape.circle,
                          ),
                          padding: const EdgeInsets.all(2),
                          child: const Icon(Icons.edit, size: 16, color: Colors.white),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),

            const SizedBox(height: 12),
            // 사용자 이름
            const Center(
              child: Text(
                '최인혁',
                style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              ),
            ),

            const SizedBox(height: 32),
            // 프로필 변경
            _buildMenuItem(Icons.account_circle, '프로필 변경', () {
              Navigator.pushReplacement(
                context,
                MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
              );
            }),
            // 프로필 수정
            _buildMenuItem(Icons.edit, '프로필 수정', () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (_) => const EditProfilePage()),
              );
            }),
            // 프리셋
            _buildMenuItem(Icons.favorite, '프리셋', () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (_) => const PresetPage()),
              );
            }),
            // 펌웨어 업데이트
            _buildMenuItem(Icons.system_update_alt, '펌웨어 업데이트', () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (_) => const UpdatePage()),
              );
            }),
            // 로그아웃
            _buildMenuItem(Icons.logout, '로그아웃', () {
              showDialog(
                context: context,
                builder: (_) => AlertDialog(
                  title: const Text('로그아웃'),
                  content: const Text('로그아웃 되었습니다.'),
                  actions: [
                    TextButton(
                      onPressed: () {
                        Navigator.pop(context); // 닫기
                        Navigator.pushAndRemoveUntil(
                          context,
                          MaterialPageRoute(builder: (_) => const LoginScreen()),
                              (route) => false,
                        );
                      },
                      child: const Text('확인'),
                    ),
                  ],
                ),
              );
            }),
          ],
        ),
      ),
    );
  }
}
