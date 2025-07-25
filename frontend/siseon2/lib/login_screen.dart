import 'package:flutter/material.dart';
import 'package:google_sign_in/google_sign_in.dart';
import 'profile_select_screen.dart';

class LoginScreen extends StatelessWidget {
  const LoginScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: Center(
        child: SingleChildScrollView(
          padding: const EdgeInsets.symmetric(horizontal: 32.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const Text(
                '로그인',
                style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              ),
              const SizedBox(height: 32),
              const Image(
                image: AssetImage('assets/images/eye_open.png'),
                width: 120,
                height: 120,
              ),
              const SizedBox(height: 24),
              const Text(
                '환영합니다',
                style: TextStyle(
                  fontSize: 20,
                  color: Color(0xFF2563FF),
                  fontWeight: FontWeight.bold,
                ),
              ),
              const SizedBox(height: 12),
              const Text(
                'SISEON은 AIoT 기반 시선·자세 분석으로\n최적의 디스플레이 위치를 제안해 줍니다.',
                textAlign: TextAlign.center,
                style: TextStyle(color: Colors.black54),
              ),
              const SizedBox(height: 40),

              /// 👇 구글 로그인 버튼
              GestureDetector(
                onTap: () async {
                  try {
                    final GoogleSignIn _googleSignIn = GoogleSignIn(
                      scopes: ['email', 'profile'],
                    );

                    await _googleSignIn.signOut(); // ✅ 기존 세션 제거

                    final account = await _googleSignIn.signIn();
                    final auth = await account?.authentication;
                    final accessToken = auth?.accessToken;

                    if (accessToken != null) {
                      print('🔑 accessToken: $accessToken');

                      if (context.mounted) {
                        showDialog(
                          context: context,
                          builder: (context) => AlertDialog(
                            title: const Text('로그인 완료!'),
                            content: Text('${account?.displayName ?? "사용자"}님 환영합니다.'),
                            actions: [
                              TextButton(
                                onPressed: () {
                                  Navigator.pop(context);
                                  Navigator.pushReplacement(
                                    context,
                                    MaterialPageRoute(builder: (context) => const ProfileSelectScreen()),
                                  );

                                },
                                child: const Text('확인'),
                              ),
                            ],
                          ),
                        );
                      }
                    }
                  } catch (e) {
                    print('❌ 로그인 에러: $e');
                  }
                },
                child: Container(
                  padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 20),
                  decoration: BoxDecoration(
                    border: Border.all(color: Colors.grey.shade300),
                    borderRadius: BorderRadius.circular(40),
                    color: Colors.white,
                    boxShadow: [
                      BoxShadow(
                        color: Colors.black12,
                        blurRadius: 4,
                        offset: Offset(0, 2),
                      )
                    ],
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Image.asset('assets/images/google_logo.png', width: 24),
                      const SizedBox(width: 12),
                      const Text(
                        'Sign in with Google',
                        style: TextStyle(fontSize: 16),
                      ),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 20),
              const Text(
                '계정이 없으신가요? 회원가입',
                style: TextStyle(color: Colors.black54, fontSize: 13),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
