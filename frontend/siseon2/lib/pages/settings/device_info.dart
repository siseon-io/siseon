import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;

import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/device_cache_service.dart';
import 'package:siseon2/widgets/rect_card.dart';

class DeviceInfoPage extends StatefulWidget {
  const DeviceInfoPage({super.key});

  @override
  State<DeviceInfoPage> createState() => _DeviceInfoPageState();
}

class _DeviceInfoPageState extends State<DeviceInfoPage> {
  // Theme
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color cardGrey       = Color(0xFF161B22);
  static const Color primaryBlue    = Color(0xFF3B82F6);
  static const Color okGreen        = Color(0xFF22C55E);

  // 하드코딩된 현재 앱/디바이스 펌웨어 버전
  static const String _currentFwVersion = '1.3.5';

  Map<String, dynamic>? _profile;
  String? _deviceSerial;
  bool _isLoading = true;

  int? get _profileIdSafe => _profile?['profileId'] ?? _profile?['id'];

  @override
  void initState() {
    super.initState();
    _loadProfileAndDevice();
  }

  Future<void> _loadProfileAndDevice() async {
    setState(() => _isLoading = true);

    final profile = await ProfileCacheService.loadProfile();
    String? serial;

    if (profile != null) {
      final pid = profile['profileId'] ?? profile['id'];
      if (pid != null) {
        // 서버 우선 조회
        try {
          final token = await AuthService.getValidAccessToken();
          final res = await http.get(
            Uri.parse('https://i13b101.p.ssafy.io/siseon/api/device/profile/$pid'),
            headers: {
              'Authorization': 'Bearer $token',
              'Accept': 'application/json',
            },
          );

          if (res.statusCode == 200 && res.body.isNotEmpty) {
            final decoded = jsonDecode(res.body);
            serial = _extractSerial(decoded);
          } else if (res.statusCode == 404 || res.statusCode == 204) {
            serial = null;
          }
        } catch (_) {/* 네트워크 실패 시 아래 캐시 폴백 */}

        // 캐시 폴백
        if (serial == null) {
          final device = await DeviceCacheService.loadDeviceForProfile(pid);
          final s = (device?['serial'] as String?)?.trim();
          if (s != null && s.isNotEmpty) serial = s;
        }
      }
    }

    // 레거시 폴백
    if (serial == null) {
      final prefs = await SharedPreferences.getInstance();
      serial = prefs.getString('deviceSerial');
    }

    if (!mounted) return;
    setState(() {
      _profile = profile;
      _deviceSerial = serial;
      _isLoading = false;
    });
  }

  String? _extractSerial(dynamic json) {
    if (json == null) return null;
    if (json is Map) {
      for (final k in ['serial', 'deviceSerial', 'device_serial']) {
        final v = json[k];
        if (v is String && v.trim().isNotEmpty) return v.trim();
      }
      if (json.containsKey('data')) return _extractSerial(json['data']);
    }
    if (json is List) {
      for (final item in json) {
        final s = _extractSerial(item);
        if (s != null) return s;
      }
    }
    return null;
  }

  Future<void> _deleteDevice() async {
    if (_profileIdSafe == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('프로필 ID를 찾을 수 없습니다.')),
      );
      return;
    }

    final confirm = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: cardGrey,
        titleTextStyle: const TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.w700),
        contentTextStyle: const TextStyle(color: Colors.white70, fontSize: 14),
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
            style: TextButton.styleFrom(foregroundColor: primaryBlue),
            child: const Text("취소"),
          ),
        ],
      ),
    );

    if (confirm != true) return;

    try {
      final token = await AuthService.getValidAccessToken();
      final res = await http.delete(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/device/profile/${_profileIdSafe!}'),
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
          'Accept': 'application/json',
        },
      );

      if (res.statusCode == 200 || res.statusCode == 204) {
        // 캐시 정리
        await DeviceCacheService.clearDeviceForProfile(_profileIdSafe!);

        final prefs = await SharedPreferences.getInstance();
        await prefs.remove('deviceSerial');
        await prefs.remove('isDeviceRegistered');

        if (!mounted) return;
        setState(() => _deviceSerial = null);
        Navigator.pop(context, true);
      } else {
        if (!mounted) return;
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('❌ 삭제 실패(${res.statusCode}) : ${res.body}')),
        );
      }
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('네트워크 오류: $e')),
      );
    }
  }

  ImageProvider _buildProfileImage() {
    final imageUrl = _profile?['imageUrl'];
    if (imageUrl != null && imageUrl.toString().isNotEmpty) {
      if (imageUrl.toString().startsWith('http')) return NetworkImage(imageUrl);
      return AssetImage(imageUrl);
    }
    return const AssetImage('assets/images/profile_cat.png');
  }

  @override
  Widget build(BuildContext context) {
    if (_isLoading) {
      return const Scaffold(
        backgroundColor: backgroundBlack,
        body: Center(child: CircularProgressIndicator(color: Colors.white)),
      );
    }

    return Scaffold(
      backgroundColor: backgroundBlack,
      appBar: AppBar(
        title: const Text("기기 정보"),
        backgroundColor: backgroundBlack,
        foregroundColor: Colors.white,
        elevation: 0,
        actions: [
          IconButton(
            tooltip: _deviceSerial == null ? '등록된 기기 없음' : '기기 삭제',
            onPressed: _deviceSerial == null ? null : _deleteDevice,
            icon: Icon(
              Icons.delete_forever,
              color: _deviceSerial == null ? Colors.white24 : Colors.redAccent,
            ),
          ),
        ],
      ),
      body: RefreshIndicator(
        color: Colors.white,
        backgroundColor: backgroundBlack,
        onRefresh: _loadProfileAndDevice,
        child: ListView(
          physics: const AlwaysScrollableScrollPhysics(),
          padding: const EdgeInsets.fromLTRB(16, 8, 16, 24),
          children: [
            const SizedBox(height: 8),
            // 프로필 아바타 + 이름
            Center(
              child: CircleAvatar(
                radius: 50,
                backgroundColor: Colors.grey[800],
                backgroundImage: _buildProfileImage(),
              ),
            ),
            const SizedBox(height: 12),
            Center(
              child: Text(
                _profile?['name'] ?? '사용자',
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),

            const SizedBox(height: 20),

            // 시리얼 넘버 카드
            RectCard(
              bgColor: cardGrey,
              outlineColor: Colors.white.withOpacity(0.16),
              elevated: false,
              child: Row(
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Container(
                    width: 42,
                    height: 42,
                    decoration: BoxDecoration(
                      color: primaryBlue.withOpacity(0.15),
                      borderRadius: BorderRadius.circular(10),
                      border: Border.all(color: primaryBlue.withOpacity(0.35), width: 1),
                    ),
                    child: const Icon(Icons.confirmation_number, color: primaryBlue),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text('시리얼 넘버',
                            style: TextStyle(color: Colors.white54, fontSize: 12)),
                        const SizedBox(height: 4),
                        Text(
                          _deviceSerial ?? '등록된 기기가 없습니다.',
                          style: const TextStyle(
                            color: Colors.white,
                            fontSize: 16,
                            fontWeight: FontWeight.w600,
                          ),
                          overflow: TextOverflow.ellipsis,
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            const SizedBox(height: 12),

            // 펌웨어 버전 카드 (현재는 최신 버전으로 고정)
            RectCard(
              bgColor: cardGrey,
              outlineColor: Colors.white.withOpacity(0.16),
              elevated: false,
              child: Row(
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Container(
                    width: 42,
                    height: 42,
                    decoration: BoxDecoration(
                      color: okGreen.withOpacity(0.15),
                      borderRadius: BorderRadius.circular(10),
                      border: Border.all(color: okGreen.withOpacity(0.35), width: 1),
                    ),
                    child: const Icon(Icons.system_update, color: okGreen),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text('펌웨어 버전',
                            style: TextStyle(color: Colors.white54, fontSize: 12)),
                        const SizedBox(height: 4),
                        Text(
                          _currentFwVersion,
                          style: const TextStyle(
                            color: Colors.white,
                            fontSize: 16,
                            fontWeight: FontWeight.w700,
                          ),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(width: 12),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
                    decoration: BoxDecoration(
                      color: okGreen.withOpacity(0.15),
                      borderRadius: BorderRadius.circular(999),
                      border: Border.all(color: okGreen.withOpacity(0.45), width: 1),
                    ),
                    child: Row(
                      children: const [
                        Icon(Icons.check_circle, size: 14, color: okGreen),
                        SizedBox(width: 6),
                        Text(
                          '최신 버전입니다',
                          style: TextStyle(color: okGreen, fontWeight: FontWeight.w700, fontSize: 12),
                        ),
                      ],
                    ),
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
