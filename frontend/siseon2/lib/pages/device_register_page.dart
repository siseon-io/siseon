import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;

import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/device_cache_service.dart';

/// BleScanScreen 과 같은 팔레트
class AppColors {
  static const backgroundBlack = Color(0xFF0D1117);
  static const cardGrey       = Color(0xFF161B22);
  static const cardBorder     = Color(0xFF334155);
  static const primaryBlue    = Color(0xFF3B82F6);
  static const text           = Colors.white;
  static const textSub        = Colors.white70;
}

class DeviceRegisterPage extends StatefulWidget {
  const DeviceRegisterPage({super.key});

  @override
  State<DeviceRegisterPage> createState() => _DeviceRegisterPageState();
}

class _DeviceRegisterPageState extends State<DeviceRegisterPage> {
  final TextEditingController _serialController = TextEditingController();
  final _formKey = GlobalKey<FormState>();
  bool _isLoading = false;

  Future<void> _registerDevice() async {
    FocusScope.of(context).unfocus();

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

      // 서버 등록
      final res = await http.post(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/device'),
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
        body: jsonEncode({
          'profileId': pid,
          'serialNumber': serial,
        }),
      );

      if (res.statusCode == 200 || res.statusCode == 201) {
        // 캐시에 즉시 반영
        await DeviceCacheService.saveDeviceForProfile(pid, {'serial': serial});
        unawaited(DeviceCacheService.fetchAndCacheDevice(profileId: pid));

        if (!mounted) return;
        Navigator.pop(context, true);
      } else if (res.statusCode == 409) {
        _toast('이미 등록된 시리얼입니다. (409)');
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
        backgroundColor: Colors.black.withOpacity(0.85),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
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
    final canSubmit = _serialController.text.trim().isNotEmpty && !_isLoading;

    return GestureDetector(
      onTap: () => FocusScope.of(context).unfocus(),
      child: Scaffold(
        backgroundColor: AppColors.backgroundBlack,
        appBar: AppBar(
          backgroundColor: AppColors.backgroundBlack,
          elevation: 0,
          centerTitle: true,
          title: const Text(
            '기기 등록',
            style: TextStyle(color: AppColors.text, fontWeight: FontWeight.w700),
          ),
          iconTheme: const IconThemeData(color: Colors.white),
          foregroundColor: Colors.white,
        ),
        body: SafeArea(
          child: SingleChildScrollView(
            padding: const EdgeInsets.fromLTRB(16, 20, 16, 20),
            child: Form(
              key: _formKey,
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  // 설명 카드
                  Container(
                    padding: const EdgeInsets.all(14),
                    decoration: BoxDecoration(
                      color: AppColors.cardGrey,
                      borderRadius: BorderRadius.circular(14),
                      border: Border.all(color: AppColors.cardBorder.withOpacity(0.5)),
                    ),
                    child: const Text(
                      '기기의 시리얼 넘버를 입력해주세요.',
                      style: TextStyle(color: AppColors.text, fontSize: 15),
                    ),
                  ),
                  const SizedBox(height: 14),

                  // 입력 필드 카드
                  Container(
                    padding: const EdgeInsets.all(14),
                    decoration: BoxDecoration(
                      color: AppColors.cardGrey,
                      borderRadius: BorderRadius.circular(14),
                      border: Border.all(color: Colors.white.withOpacity(0.12)),
                    ),
                    child: TextFormField(
                      controller: _serialController,
                      onChanged: (_) => setState(() {}),
                      style: const TextStyle(color: AppColors.text, fontSize: 16),
                      textCapitalization: TextCapitalization.characters,
                      inputFormatters: [
                        FilteringTextInputFormatter.allow(RegExp(r'[A-Za-z0-9\-]')),
                      ],
                      decoration: InputDecoration(
                        hintText: '예: ABCD1234',
                        hintStyle: const TextStyle(color: Colors.white38),
                        isDense: true,
                        filled: true,
                        fillColor: const Color(0xFF0F172A), // 살짝 어두운 인풋 배경
                        contentPadding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
                        enabledBorder: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(10),
                          borderSide: BorderSide(color: AppColors.cardBorder.withOpacity(0.5)),
                        ),
                        focusedBorder: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(10),
                          borderSide: const BorderSide(color: AppColors.primaryBlue, width: 1.2),
                        ),
                        suffixIcon: IconButton(
                          tooltip: '지우기',
                          icon: const Icon(Icons.clear, color: Colors.white38),
                          onPressed: () {
                            _serialController.clear();
                            setState(() {});
                          },
                        ),
                      ),
                    ),
                  ),

                  const SizedBox(height: 22),

                  // 제출 버튼
                  SizedBox(
                    height: 48,
                    width: double.infinity,
                    child: ElevatedButton.icon(
                      onPressed: canSubmit ? _registerDevice : null,
                      icon: _isLoading
                          ? const SizedBox(
                        width: 18,
                        height: 18,
                        child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white),
                      )
                          : const Icon(Icons.verified, color: Colors.white),
                      label: Text(
                        _isLoading ? '등록 중...' : '등록 완료',
                        style: const TextStyle(
                          color: Colors.white,
                          fontWeight: FontWeight.w700,
                        ),
                      ),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: AppColors.primaryBlue,
                        disabledBackgroundColor: AppColors.primaryBlue.withOpacity(0.35),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

// Dart <= 3.3 호환용(Top-level await 불가하므로 무시 가능한 Future로)
void unawaited(Future<void> f) {}
