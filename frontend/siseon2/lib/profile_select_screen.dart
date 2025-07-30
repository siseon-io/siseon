import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import '/root_screen.dart';
import '/pages/profile_create.dart';

class ProfileSelectScreen extends StatefulWidget {
  const ProfileSelectScreen({super.key});

  @override
  State<ProfileSelectScreen> createState() => _ProfileSelectScreenState();
}

class _ProfileSelectScreenState extends State<ProfileSelectScreen> {
  List<Map<String, dynamic>> _profiles = [];
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    fetchProfiles();
  }

  Future<void> fetchProfiles() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      showError('로그인이 필요합니다.');
      return;
    }

    try {
      final response = await http.get(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile'),
        headers: {'Authorization': 'Bearer $token'},
      );

      print('📦 [응답 상태코드] ${response.statusCode}');
      print('📦 [utf8 디코딩 결과] ${utf8.decode(response.bodyBytes)}');

      if (response.statusCode == 200) {
        final List data = jsonDecode(utf8.decode(response.bodyBytes));
        setState(() {
          _profiles = List<Map<String, dynamic>>.from(data);
          _isLoading = false;
        });
      } else {
        showError('프로필 조회 실패 (${response.statusCode})');
      }
    } catch (e) {
      print('❌ [예외 발생] $e');
      showError('예외 발생: $e');
    }
  }

  void showError(String message) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(message)));
  }

  void onAddPressed() async {
    final result = await Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => const ProfileCreateScreen()),
    );
    if (result == true) {
      fetchProfiles();
    }
  }

  ImageProvider? _getImageProvider(String? imageUrl) {
    if (imageUrl == null) return null;
    if (imageUrl.startsWith('http')) {
      return NetworkImage(imageUrl);
    } else if (imageUrl.startsWith('assets/')) {
      return AssetImage(imageUrl);
    }
    return null;
  }

  Future<void> onProfileSelected(Map<String, dynamic> profile) async {
    await ProfileCacheService.saveProfile(profile);
    Navigator.pushReplacement(
      context,
      MaterialPageRoute(builder: (_) => const RootScreen()),
    );
  }

  @override
  Widget build(BuildContext context) {
    final profilesWithAdd = [..._profiles];
    if (_profiles.length < 4) {
      profilesWithAdd.add({'isAddButton': true});
    }

    return Scaffold(
      backgroundColor: const Color(0xFF1C1C1E),
      appBar: AppBar(
        title: const Text('프로필 선택'),
        backgroundColor: const Color(0xFF1C1C1E),
        elevation: 0,
        centerTitle: true,
      ),
      body: _isLoading
          ? const Center(child: CircularProgressIndicator())
          : Padding(
        padding: const EdgeInsets.all(20),
        child: GridView.builder(
          itemCount: profilesWithAdd.length,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: 2,
            crossAxisSpacing: 24,
            mainAxisSpacing: 24,
            childAspectRatio: 0.85,
          ),
          itemBuilder: (context, index) {
            final profile = profilesWithAdd[index];
            final isAddButton = profile['isAddButton'] == true;

            return GestureDetector(
              onTap: () {
                if (isAddButton) {
                  onAddPressed();
                } else {
                  onProfileSelected(profile);
                }
              },
              child: AnimatedContainer(
                duration: const Duration(milliseconds: 200),
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  color: Colors.white10,
                  borderRadius: BorderRadius.circular(24),
                  border: Border.all(color: Colors.white24, width: 1),
                ),
                child: Column(
                  mainAxisSize: MainAxisSize.min, // ✅ overflow 방지 핵심
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    CircleAvatar(
                      radius: 36, // ✅ 약간 줄이기
                      backgroundColor: Colors.grey[800],
                      backgroundImage: _getImageProvider(profile['imageUrl']),
                      child: _getImageProvider(profile['imageUrl']) == null
                          ? const Icon(Icons.person, color: Colors.white30, size: 36)
                          : null,
                    ),
                    const SizedBox(height: 10),
                    Text(
                      isAddButton ? '프로필 추가' : (profile['name'] ?? ''),
                      style: const TextStyle(
                        fontFamily: 'Pretendard',
                        fontSize: 16, // ✅ 조금 줄임
                        color: Colors.white,
                        fontWeight: FontWeight.w600,
                      ),
                      overflow: TextOverflow.ellipsis,
                    ),
                    if (isAddButton)
                      const Padding(
                        padding: EdgeInsets.only(top: 6),
                        child: Icon(Icons.add_circle_outline, color: Colors.white70, size: 24),
                      ),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );
  }
}
