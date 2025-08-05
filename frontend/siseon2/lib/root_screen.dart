import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
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
  final GlobalKey<HomeScreenState> _homeKey = GlobalKey<HomeScreenState>(); // ✅ HomeScreen State 접근용 키
  late List<Widget> _pages;
  int _currentIndex = 0;

  BluetoothCharacteristic? _writableChar; // ✅ BLE 연결 여부 판단용

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  @override
  void initState() {
    super.initState();
    _pages = [
      HomeScreen(
        key: _homeKey,
        onAiModeSwitch: _handleAiModeFromHome,
        onConnect: (char) {
          setState(() {
            _writableChar = char; // ✅ BLE 연결 시 characteristic 저장
          });
        },
      ),
      const StatsPage(),
      const ProfilePage(),
    ];
  }

  /// ✅ RootScreen → HomeScreen의 AI 모드 전환 메서드 호출
  void _switchToAIMode() {
    _homeKey.currentState?.switchToAiMode(); // ✅ MQTT 발행은 HomeScreen이 전담

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('🤖 AI 모드로 전환됩니다.'),
        duration: Duration(seconds: 2),
      ),
    );
  }

  /// ✅ HomeScreen에서 콜백으로 전달된 AI 모드 전환 처리
  void _handleAiModeFromHome() {
    debugPrint("🔄 RootScreen에서 HomeScreen의 AI 모드 콜백 실행됨");
  }

  /// ✅ 탭 선택
  Future<void> _selectTab(int idx) async {
    if (idx == 1 || idx == 2) {
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    }
    setState(() => _currentIndex = idx);
  }

  /// ✅ BLE 연결됐으면 ManualPage로 이동, 아니면 스낵바
  Future<void> _handleManualTap() async {
    if (_writableChar != null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('3초 뒤 매뉴얼 화면으로 전환됩니다.'), duration: Duration(seconds: 2)),
      );
      await Future.delayed(const Duration(seconds: 3));
      await SystemChrome.setPreferredOrientations([
        DeviceOrientation.landscapeLeft,
        DeviceOrientation.landscapeRight,
      ]);

      if (!mounted) return;
      Navigator.push(
        context,
        MaterialPageRoute(builder: (_) => ManualPage(writableChar: _writableChar!)),
      ).then((_) async {
        // 돌아오면 세로 모드 복귀
        await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
      });
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('⚠️ 먼저 BLE 기기를 연결해주세요.'),
          duration: Duration(seconds: 2),
        ),
      );
    }
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
          onPressed: _switchToAIMode, // ✅ AI 모드 버튼
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
            _buildManualTabItem(Icons.menu_book_rounded, '매뉴얼'),
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

  /// ✅ BLE 연결됐을 때만 진입 가능한 매뉴얼 탭
  Widget _buildManualTabItem(IconData icon, String label) {
    final isActive = _writableChar != null;
    final color = isActive ? primaryBlue : inactiveGrey;

    return GestureDetector(
      onTap: _handleManualTap,
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
                fontWeight: FontWeight.w400,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
