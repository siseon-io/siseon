import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:siseon2/services/auth_service.dart';
import 'root_screen.dart';
import 'pages/profile_create.dart';

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

    final response = await http.get(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/profile'),
      headers: {'Authorization': 'Bearer $token'},
    );

    if (response.statusCode == 200) {
      final List data = jsonDecode(response.body);
      setState(() {
        _profiles = List<Map<String, dynamic>>.from(data);
        _isLoading = false;
      });
    } else {
      showError('프로필 조회 실패 (${response.statusCode})');
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
                  final name = profile['name'] ?? '이름 없음';
                  Navigator.pushReplacement(
                    context,
                    MaterialPageRoute(
                      builder: (_) => RootScreen(userName: name),
                    ),
                  );
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
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    CircleAvatar(
                      radius: 40,
                      backgroundColor: Colors.grey[800],
                      backgroundImage: _getImageProvider(profile['imageUrl']),
                      child: _getImageProvider(profile['imageUrl']) == null
                          ? const Icon(Icons.person, color: Colors.white30, size: 40)
                          : null,
                    ),
                    const SizedBox(height: 12),
                    Text(
                      isAddButton ? '프로필 추가' : profile['name'] ?? '',
                      style: const TextStyle(
                        fontSize: 18,
                        color: Colors.white,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                    if (isAddButton)
                      const Padding(
                        padding: EdgeInsets.only(top: 8),
                        child: Icon(Icons.add_circle_outline, color: Colors.white70, size: 32),
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
