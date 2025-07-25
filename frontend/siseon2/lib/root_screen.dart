import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'pages/home_screen.dart';
import 'pages/manual_page.dart';
import 'pages/stats_page.dart';
import 'pages/settings/profile_page.dart';

class RootScreen extends StatefulWidget {
  final String userName;
  const RootScreen({super.key, required this.userName});

  @override
  State<RootScreen> createState() => _RootScreenState();
}

class _RootScreenState extends State<RootScreen> {
  late final List<Widget> _pages;
  int _currentIndex = 0;

  @override
  void initState() {
    super.initState();
    _pages = [
      HomeScreen(userName: widget.userName),
      const ManualPage(),
      const StatsPage(),
      const ProfilePage(),
    ];
  }

  Future<void> _selectTab(int idx) async {
    if (idx == 1) {
      // ManualPage 진입 시: 안내 → 지연 → 가로모드 전환
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('약 3초 뒤에 핸드폰이 가로모드로 변경됩니다.'),
          duration: Duration(seconds: 3),
        ),
      );

      await Future.delayed(const Duration(seconds: 3));

      await SystemChrome.setPreferredOrientations([
        DeviceOrientation.landscapeRight,
        DeviceOrientation.landscapeLeft,
      ]);
    } else {
      // 다른 탭: 세로모드로 복구
      await SystemChrome.setPreferredOrientations([
        DeviceOrientation.portraitUp,
      ]);
    }

    setState(() => _currentIndex = idx);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: _pages[_currentIndex],
      floatingActionButton: FloatingActionButton(
        onPressed: () => _selectTab(2), // StatsPage로 이동
        backgroundColor: Colors.purple,
        child: const Icon(Icons.remove_red_eye, size: 32),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,
      bottomNavigationBar: BottomAppBar(
        shape: const CircularNotchedRectangle(),
        notchMargin: 8,
        child: SizedBox(
          height: 70,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildTabItem(Icons.home, 'Home', 0),
              _buildTabItem(Icons.menu_book, 'Manual', 1),
              const SizedBox(width: 60),
              _buildTabItem(Icons.bar_chart, 'Stats', 2),
              _buildTabItem(Icons.person, 'Profile', 3),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildTabItem(IconData icon, String label, int idx) {
    final isSelected = _currentIndex == idx;
    final color = isSelected ? Colors.purple : Colors.grey;
    return GestureDetector(
      onTap: () => _selectTab(idx),
      behavior: HitTestBehavior.opaque,
      child: SizedBox(
        width: 60,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: color),
            const SizedBox(height: 4),
            Text(label, style: TextStyle(color: color, fontSize: 12)),
          ],
        ),
      ),
    );
  }
}
