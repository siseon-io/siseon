import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'pages/home_screen.dart';
import 'pages/manual_page.dart';
import 'pages/stats_page.dart';
import 'pages/settings/profile_page.dart';

class RootScreen extends StatefulWidget {
  const RootScreen({super.key});

  @override
  State<RootScreen> createState() => _RootScreenState();
}

class _RootScreenState extends State<RootScreen> {
  late final List<Widget> _pages;
  int _currentIndex = 0;

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  @override
  void initState() {
    super.initState();
    _pages = const [
      HomeScreen(),
      StatsPage(),   // ✅ ManualPage 제거
      ProfilePage(),
    ];
  }

  Future<void> _selectTab(int idx) async {
    if (idx == 1) { // 통계 화면
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    } else if (idx == 2) { // 프로필 화면
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    }
    setState(() => _currentIndex = idx);
  }

  /// ✅ ManualPage는 탭 대신 Navigator.push로 이동
  Future<void> _openManualPage() async {
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('약 3초 뒤 가로모드로 전환됩니다.'),
        duration: Duration(seconds: 3),
      ),
    );
    await Future.delayed(const Duration(seconds: 3));
    await SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeRight,
      DeviceOrientation.landscapeLeft,
    ]);

    if (!mounted) return;
    Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => const ManualPage()),
    ).then((_) async {
      // ManualPage에서 돌아오면 세로모드로 복귀
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBody: true,
      backgroundColor: rootBackground,
      body: _pages[_currentIndex],

      floatingActionButton: Container(
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.4),
              blurRadius: 10,
              spreadRadius: 2,
              offset: const Offset(0, 4),
            ),
          ],
        ),
        child: FloatingActionButton(
          onPressed: () => _selectTab(1), // 🔥 FAB → 통계로 이동 유지
          backgroundColor: primaryBlue,
          elevation: 0,
          child: const Icon(Icons.remove_red_eye, size: 30, color: Colors.white),
        ),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,

      bottomNavigationBar: Container(
        height: 85,
        decoration: const BoxDecoration(
          border: Border(top: BorderSide(color: Colors.white12, width: 1)),
          color: rootBackground,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            _buildTabItem(Icons.home, '홈', 0),
            _buildManualTabItem(Icons.menu_book_rounded, '매뉴얼'), // ✅ ManualPage 버튼 분리
            const SizedBox(width: 60),
            _buildTabItem(Icons.bar_chart_rounded, '통계', 1),
            _buildTabItem(Icons.person, '프로필', 2),
          ],
        ),
      ),
    );
  }

  Widget _buildTabItem(IconData icon, String label, int idx) {
    final isSelected = _currentIndex == idx;
    final color = isSelected ? primaryBlue : inactiveGrey;

    return GestureDetector(
      onTap: () => _selectTab(idx),
      behavior: HitTestBehavior.opaque,
      child: SizedBox(
        width: 65,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: color, size: 28),
            const SizedBox(height: 3),
            Text(
              label,
              style: TextStyle(
                color: color,
                fontSize: 12,
                fontWeight: isSelected ? FontWeight.w600 : FontWeight.w400,
              ),
            ),
          ],
        ),
      ),
    );
  }

  /// ✅ 매뉴얼 버튼은 push로 이동
  Widget _buildManualTabItem(IconData icon, String label) {
    return GestureDetector(
      onTap: _openManualPage,
      behavior: HitTestBehavior.opaque,
      child: SizedBox(
        width: 65,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: inactiveGrey, size: 28),
            const SizedBox(height: 3),
            Text(
              label,
              style: const TextStyle(
                color: inactiveGrey,
                fontSize: 12,
                fontWeight: FontWeight.w400,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
