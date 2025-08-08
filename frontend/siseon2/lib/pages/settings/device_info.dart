import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:http/http.dart' as http;

class DeviceInfoPage extends StatefulWidget {
  const DeviceInfoPage({super.key});

  @override
  State<DeviceInfoPage> createState() => _DeviceInfoPageState();
}

class _DeviceInfoPageState extends State<DeviceInfoPage> {
  Map<String, dynamic>? _profile;
  String? _deviceSerial;
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _loadProfileAndDevice();
  }

  /// ✅ 프로필과 기기 정보 불러오기
  Future<void> _loadProfileAndDevice() async {
    final profile = await ProfileCacheService.loadProfile();
    final prefs = await SharedPreferences.getInstance();
    final serial = prefs.getString('deviceSerial');

    setState(() {
      _profile = profile;
      _deviceSerial = serial;
      _isLoading = false;
    });
    print("📦 불러온 프로필: $_profile");
  }

  /// ✅ 기기 삭제 처리
  Future<void> _deleteDevice() async {
    if (_profile == null) return;
    final profileId = _profile!['id'];

    final confirm = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text("기기 삭제"),
        content: const Text("등록된 기기를 삭제하시겠습니까?\n삭제 후에는 다시 등록해야 합니다."),
        actions: [
          ElevatedButton(
            onPressed: () => Navigator.pop(context, true),
            style: ElevatedButton.styleFrom(backgroundColor: Colors.redAccent),
            child: const Text("기기 삭제"),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text("취소"),
          ),
        ],
      ),
    );

    if (confirm != true) return;

    final token = await AuthService.getValidAccessToken();
    final res = await http.delete(
      Uri.parse('http://i13b101.p.ssafy.io:8080/api/device/profile/$profileId'),
      headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer $token',
      },
    );

    if (res.statusCode == 200 || res.statusCode == 204) {
      final prefs = await SharedPreferences.getInstance();
      await prefs.remove('deviceSerial');
      await prefs.setBool('isDeviceRegistered', false);

      if (mounted) {
        Navigator.pop(context, true); // ✅ 홈 화면으로 돌아가면서 true 전달
      }
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('❌ 삭제 실패: ${res.statusCode}')),
      );
    }
  }

  /// ✅ 프로필 이미지 위젯 빌더 (Asset / Network 자동 처리)
  ImageProvider _buildProfileImage() {
    final imageUrl = _profile?['imageUrl'];

    if (imageUrl != null && imageUrl.toString().isNotEmpty) {
      if (imageUrl.toString().startsWith('http')) {
        return NetworkImage(imageUrl);
      } else {
        return AssetImage(imageUrl);
      }
    }
    return const AssetImage('assets/images/profile_cat.png');
  }

  @override
  Widget build(BuildContext context) {
    if (_isLoading) {
      return const Scaffold(
        backgroundColor: Color(0xFF161B22),
        body: Center(child: CircularProgressIndicator(color: Colors.white)),
      );
    }

    return Scaffold(
      backgroundColor: const Color(0xFF161B22),
      appBar: AppBar(
        title: const Text("기기 정보"),
        backgroundColor: const Color(0xFF161B22),
        foregroundColor: Colors.white,
        actions: [
          if (_deviceSerial != null)
            IconButton(
              icon: const Icon(Icons.delete_forever, color: Colors.redAccent),
              onPressed: _deleteDevice,
            ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            /// 프로필 이미지
            CircleAvatar(
              radius: 50,
              backgroundImage: _buildProfileImage(),
            ),
            const SizedBox(height: 16),

            /// 사용자 이름
            Text(
              _profile?['name'] ?? '사용자 이름',
              style: const TextStyle(color: Colors.white, fontSize: 22, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 32),

            /// 기기 정보 카드
            Container(
              width: double.infinity,
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: Colors.white.withOpacity(0.05),
                borderRadius: BorderRadius.circular(16),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text("시리얼 넘버", style: TextStyle(color: Colors.white54, fontSize: 12)),
                  const SizedBox(height: 6),
                  Text(
                    _deviceSerial ?? '등록된 기기가 없습니다.',
                    style: const TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.w600),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
