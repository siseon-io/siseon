import 'package:flutter/material.dart';
import 'package:siseon2/services/auth_service.dart';

class UpdatePage extends StatefulWidget {
  const UpdatePage({Key? key}) : super(key: key);

  @override
  State<UpdatePage> createState() => _UpdatePageState();
}

class _UpdatePageState extends State<UpdatePage>
    with SingleTickerProviderStateMixin {
  static const currentVersion = '1.2.0';
  static const newVersion = '1.2.1';
  static const Color primaryBlue = Color(0xFF2563FF);
  static const Color background = Color(0xFF161B22);

  late AnimationController _controller;
  late Animation<double> _arrowAnimation;

  @override
  void initState() {
    super.initState();
    _controller =
    AnimationController(vsync: this, duration: const Duration(seconds: 1))
      ..repeat(reverse: true);
    _arrowAnimation =
        Tween<double>(begin: 0, end: 10).animate(CurvedAnimation(
          parent: _controller,
          curve: Curves.easeInOut,
        ));
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Future<void> _showAccessToken() async {
    final token = await AuthService.getValidAccessToken();
    print("🔑 Access Token: $token"); // 콘솔 로그

    if (!mounted) return;
    showDialog(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text('Access Token'),
        content: SingleChildScrollView(
          child: Text(
            token ?? '토큰 없음 (로그인 필요)',
            style: const TextStyle(fontSize: 14),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('닫기'),
          )
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: background,
      appBar: AppBar(
        title: const Text('펌웨어 업데이트'),
        backgroundColor: background,
        foregroundColor: primaryBlue,
        elevation: 0,
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: primaryBlue),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 32),
          child: Column(
            children: [
              _buildVersionCard('현재 버전', currentVersion, Colors.grey[400]!),
              const SizedBox(height: 20),
              // 애니메이션 화살표
              AnimatedBuilder(
                animation: _controller,
                builder: (_, __) {
                  return Padding(
                    padding: EdgeInsets.only(top: _arrowAnimation.value),
                    child: const Icon(Icons.arrow_downward,
                        size: 40, color: Colors.grey),
                  );
                },
              ),
              const SizedBox(height: 20),
              _buildVersionCard('새 버전', newVersion, primaryBlue),
              const Spacer(),
              // ✅ Access Token 출력 버튼
              SizedBox(
                width: double.infinity,
                height: 50,
                child: ElevatedButton(
                  onPressed: _showAccessToken,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12),
                    ),
                  ),
                  child: const Text(
                    'Access Token 보기',
                    style: TextStyle(fontSize: 16, color: Colors.white),
                  ),
                ),
              ),
              const SizedBox(height: 12),
              // 기존 업데이트 버튼
              SizedBox(
                width: double.infinity,
                height: 50,
                child: ElevatedButton(
                  onPressed: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('업데이트 기능은 준비 중입니다.')),
                    );
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: primaryBlue,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12),
                    ),
                  ),
                  child: const Text(
                    '업데이트',
                    style: TextStyle(fontSize: 16, color: Colors.white),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildVersionCard(String title, String version, Color highlightColor) {
    return Card(
      color: Colors.white.withOpacity(0.05),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      elevation: 0,
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 24, horizontal: 20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              title,
              style: const TextStyle(fontSize: 13, color: Colors.grey),
            ),
            const SizedBox(height: 6),
            Text(
              version,
              style: TextStyle(
                fontSize: 24,
                fontWeight: FontWeight.bold,
                color: highlightColor,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
