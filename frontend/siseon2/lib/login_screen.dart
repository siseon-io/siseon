import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:google_sign_in/google_sign_in.dart';
import 'package:http/http.dart' as http;
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:siseon2/services/auth_service.dart'; // AuthService 경로
import 'profile_select_screen.dart';

/// 🎨 프로젝트 다크 팔레트 (필요 시 공용 팔레트로 대체 가능)
class AppColors {
  static const background = Color(0xFF0D1117); // 전체 배경
  static const card       = Color(0xFF161B22); // 카드/버튼 배경
  static const border     = Color(0xFF334155); // 보더
  static const primary    = Color(0xFF3B82F6); // 포인트
  static const text       = Colors.white;      // 본문
  static const textSub    = Colors.white70;    // 보조 텍스트
  static const textHint   = Colors.white38;    // 힌트
}

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  bool _isLoading = false;

  /// Google accessToken을 백엔드에 전송
  Future<void> sendAccessTokenToBackend(String accessToken) async {
    final url = Uri.parse('https://i13b101.p.ssafy.io/siseon/api/auth/google');
    final headers = {'Content-Type': 'application/json'};
    final body = jsonEncode({'accessToken': accessToken});

    try {
      final response = await http.post(url, headers: headers, body: body);

      if (response.statusCode == 200 && mounted) {
        final data = jsonDecode(response.body);
        final jwtAccessToken = data['accessToken'];
        final jwtRefreshToken = data['refreshToken'];

        // ✅ 로그인 후 JWT 저장
        await AuthService.saveTokens(jwtAccessToken, jwtRefreshToken);

        Navigator.pushReplacement(
          context,
          MaterialPageRoute(builder: (_) => const ProfileSelectScreen()),
        );
      } else {
        showErrorDialog('로그인 실패: ${response.statusCode}');
      }
    } catch (e) {
      showErrorDialog('서버 오류: $e');
    }
  }

  void showErrorDialog(String message) {
    if (!mounted) return;
    showDialog(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: AppColors.card,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(14),
          side: const BorderSide(color: AppColors.border),
        ),
        titleTextStyle: const TextStyle(
          color: AppColors.text,
          fontSize: 18,
          fontWeight: FontWeight.w700,
        ),
        contentTextStyle: const TextStyle(
          color: AppColors.textSub,
          fontSize: 14,
        ),
        title: const Text('에러'),
        content: Text(message),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            style: TextButton.styleFrom(foregroundColor: AppColors.primary),
            child: const Text('확인'),
          ),
        ],
      ),
    );
  }

  Future<void> handleGoogleSignIn() async {
    try {
      setState(() => _isLoading = true);

      final GoogleSignIn googleSignIn = GoogleSignIn(scopes: ['email', 'profile']);
      await googleSignIn.signOut(); // 기존 세션 제거
      final account = await googleSignIn.signIn();
      final auth = await account?.authentication;
      final accessToken = auth?.accessToken;

      if (accessToken != null) {
        await sendAccessTokenToBackend(accessToken);
      } else {
        showErrorDialog('AccessToken 없음');
      }
    } catch (e) {
      showErrorDialog('Google 로그인 실패: $e');
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: AppColors.background,
      body: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 420),
          child: AnimatedSwitcher(
            duration: const Duration(milliseconds: 250),
            child: _isLoading
                ? const SpinKitFadingCircle(
              color: AppColors.primary,
              size: 56.0,
            )
                : Padding(
              padding: const EdgeInsets.symmetric(horizontal: 28),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  // 로고/타이틀
                  const Text(
                    'SISEON',
                    style: TextStyle(
                      fontSize: 36,
                      color: AppColors.text,
                      fontWeight: FontWeight.bold,
                      letterSpacing: 1.5,
                    ),
                  ),
                  const SizedBox(height: 10),
                  const Text(
                    '당신의 자세와 시선을 감지해,\n모니터 위치를 스스로 조정합니다',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: AppColors.textSub,
                      fontSize: 14,
                      height: 1.4,
                    ),
                  ),
                  const SizedBox(height: 36),

                  // 일러스트
                  Image.asset(
                    'assets/images/eye_open.png',
                    width: 120,
                    height: 120,
                  ),
                  const SizedBox(height: 40),

                  // Google 로그인 버튼 (다크 스타일)
                  SizedBox(
                    width: double.infinity,
                    child: ElevatedButton(
                      onPressed: _isLoading ? null : handleGoogleSignIn,
                      style: ElevatedButton.styleFrom(
                        elevation: 0,
                        backgroundColor: AppColors.card,
                        disabledBackgroundColor: AppColors.card.withOpacity(0.6),
                        foregroundColor: AppColors.text,
                        padding: const EdgeInsets.symmetric(
                            horizontal: 18, vertical: 14),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14),
                          side: const BorderSide(color: AppColors.border),
                        ),
                      ),
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Image.asset(
                            'assets/images/google_logo.png',
                            width: 22,
                            height: 22,
                          ),
                          const SizedBox(width: 10),
                          const Text(
                            'Google로 로그인',
                            style: TextStyle(
                              fontSize: 16,
                              color: AppColors.text,
                              fontWeight: FontWeight.w600,
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),

                  const SizedBox(height: 16),
                  const Text(
                    '계정 선택 팝업이 보이지 않으면 이전 Google 세션을 종료해 주세요.',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: AppColors.textHint,
                      fontSize: 12,
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
