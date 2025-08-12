import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:google_sign_in/google_sign_in.dart';
import 'package:http/http.dart' as http;
import 'package:flutter_spinkit/flutter_spinkit.dart';
import 'package:siseon2/services/auth_service.dart'; // AuthService 경로
import 'profile_select_screen.dart';

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
        title: const Text('에러'),
        content: Text(message),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('확인'),
          )
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
      setState(() => _isLoading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: Center(
        child: AnimatedSwitcher(
          duration: const Duration(milliseconds: 300),
          child: _isLoading
              ? const SpinKitFadingCircle(
            color: Colors.black54,
            size: 50.0,
          )
              : Padding(
            padding: const EdgeInsets.symmetric(horizontal: 32),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Text(
                  'SISEON',
                  style: TextStyle(
                    fontSize: 36,
                    color: Colors.black,
                    fontWeight: FontWeight.bold,
                    letterSpacing: 1.5,
                  ),
                ),
                const SizedBox(height: 12),
                const Text(
                  '당신의 자세와 시선을 감지해,\n모니터 위치를 스스로 조정합니다',
                  textAlign: TextAlign.center,
                  style: TextStyle(
                    color: Colors.black54,
                    fontSize: 14,
                  ),
                ),
                const SizedBox(height: 40),
                Image.asset(
                  'assets/images/eye_open.png',
                  width: 120,
                  height: 120,
                ),
                const SizedBox(height: 50),
                ElevatedButton(
                  onPressed: handleGoogleSignIn,
                  style: ElevatedButton.styleFrom(
                    elevation: 4,
                    shadowColor: Colors.black12,
                    backgroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 12),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(30),
                      side: BorderSide(color: Colors.grey.shade300),
                    ),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Image.asset('assets/images/google_logo.png', width: 24),
                      const SizedBox(width: 12),
                      const Text(
                        'Google로 로그인',
                        style: TextStyle(color: Colors.black87, fontSize: 16),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
