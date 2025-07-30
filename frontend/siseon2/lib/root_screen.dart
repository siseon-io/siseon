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
  static const Color rootBackground = Color(0xFF161B22); // ✅ 홈보다 살짝 밝게
  static const Color inactiveGrey = Colors.grey;

  @override
  void initState() {
    super.initState();
    _pages = const [
      HomeScreen(),
      ManualPage(),
      StatsPage(),
      ProfilePage(),
    ];
  }

  Future<void> _selectTab(int idx) async {
    if (idx == 1) {
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
    } else {
      await SystemChrome.setPreferredOrientations([
        DeviceOrientation.portraitUp,
      ]);
    }
    setState(() => _currentIndex = idx);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBody: true, // ✅ Body가 BottomAppBar 영역까지 확장되도록 설정
      backgroundColor: rootBackground,
      body: _pages[_currentIndex],

      // 🔥 FAB (중앙 버튼)
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
          onPressed: () => _selectTab(2),
          backgroundColor: primaryBlue,
          elevation: 0,
          child: const Icon(Icons.remove_red_eye, size: 30, color: Colors.white), // 아이콘 살짝 키움
        ),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,

      // 🔥 BottomAppBar


      bottomNavigationBar: Padding(
        padding: const EdgeInsets.only(bottom: 0), // ✅ 여백 제거
        child: Container(
          height: 85, // ✅ 원하는 높이
          decoration: const BoxDecoration(
            border: Border(
              top: BorderSide(color: Colors.white12, width: 1),
            ),
            color: rootBackground,
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildTabItem(Icons.home, '홈', 0),
              _buildTabItem(Icons.menu_book_rounded, '매뉴얼', 1),
              const SizedBox(width: 60),
              _buildTabItem(Icons.bar_chart_rounded, '통계', 2),
              _buildTabItem(Icons.person, '프로필', 3),
            ],
          ),
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
        width: 65, // 버튼 영역 약간 키움
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: color, size: 28), // 아이콘 크기 ↑ (26 → 28)
            const SizedBox(height: 3),
            Text(
              label,
              style: TextStyle(
                color: color,
                fontSize: 12, // 글씨 크기 ↑ (11 → 12)
                fontWeight: isSelected ? FontWeight.w600 : FontWeight.w400,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
