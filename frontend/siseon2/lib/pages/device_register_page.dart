import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;

import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/device_cache_service.dart';

class DeviceRegisterPage extends StatefulWidget {
  const DeviceRegisterPage({super.key});

  @override
  State<DeviceRegisterPage> createState() => _DeviceRegisterPageState();
}

class _DeviceRegisterPageState extends State<DeviceRegisterPage> {
  final TextEditingController _serialController = TextEditingController();
  bool _isLoading = false;

  static const Color bg = Color(0xFF1E293B);
  static const Color primary = Colors.redAccent;

  Future<void> _registerDevice() async {
    final raw = _serialController.text.trim();
    final serial = raw.toUpperCase();
    if (serial.isEmpty) {
      _toast('시리얼 넘버를 입력해주세요.');
      return;
    }

    setState(() => _isLoading = true);

    try {
      final token = await AuthService.getValidAccessToken();
      if (token == null) {
        _toast('로그인이 필요합니다.');
        setState(() => _isLoading = false);
        return;
      }

      final profile = await ProfileCacheService.loadProfile();
      final pid = profile?['id'] as int?;
      if (pid == null) {
        _toast('프로필 정보를 불러올 수 없습니다.');
        setState(() => _isLoading = false);
        return;
      }

      // 1) 서버 등록
      final res = await http.post(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/device'),
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
        body: jsonEncode({
          'profileId': pid,
          'serialNumber': serial,
        }),
      );

      // 2) 결과 처리
      if (res.statusCode == 200 || res.statusCode == 201) {
        // (중요) 즉시 로컬 캐시에 반영 → 홈으로 돌아가자마자 "등록됨" 표시
        await DeviceCacheService.saveDeviceForProfile(pid, {'serial': serial});

        // 서버 데이터 형식과 싱크 맞추기(배경 동기화: 실패해도 UX 영향 X)
        unawaited(DeviceCacheService.fetchAndCacheDevice(profileId: pid));

        if (!mounted) return;
        Navigator.pop(context, true);
      } else if (res.statusCode == 409) {
        // 서버가 중복 등록을 409로 주는 경우가 많음
        _toast('이미 등록된 시리얼입니다.\n(${res.statusCode})');
      } else {
        _toast('기기 등록 실패: ${res.statusCode}\n${res.body}');
      }
    } catch (e) {
      _toast('네트워크 오류: $e');
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  void _toast(String msg) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(msg, style: const TextStyle(color: Colors.white)),
        backgroundColor: Colors.red,
      ),
    );
  }

  @override
  void dispose() {
    _serialController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: bg,
      appBar: AppBar(
        title: const Text('기기 등록'),
        backgroundColor: primary,
      ),
      body: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              '기기의 시리얼 넘버를 입력해주세요:',
              style: TextStyle(color: Colors.white, fontSize: 16),
            ),
            const SizedBox(height: 16),
            TextField(
              controller: _serialController,
              style: const TextStyle(color: Colors.white),
              textCapitalization: TextCapitalization.characters,
              inputFormatters: [
                FilteringTextInputFormatter.allow(RegExp(r'[A-Za-z0-9\-]')),
              ],
              decoration: InputDecoration(
                hintText: '예: ABCD1234',
                hintStyle: const TextStyle(color: Colors.white38),
                filled: true,
                fillColor: Colors.black12,
                border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
              ),
            ),
            const SizedBox(height: 24),
            _isLoading
                ? const Center(child: CircularProgressIndicator(color: primary))
                : ElevatedButton(
              onPressed: _registerDevice,
              style: ElevatedButton.styleFrom(
                backgroundColor: primary,
                minimumSize: const Size(double.infinity, 48),
              ),
              child: const Text('등록 완료'),
            ),
          ],
        ),
      ),
    );
  }
}

// Dart <= 3.3 호환용(Top-level await 불가하므로 무시 가능한 Future로)
void unawaited(Future<void> f) {}
