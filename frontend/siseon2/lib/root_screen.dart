import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import 'pages/home_screen.dart';
import 'pages/manual_page.dart';
import 'pages/chatbot_page.dart';
import 'pages/settings/settings_page.dart'; // ✅ SettingsPage 클래스 사용
import 'package:siseon2/models/control_mode.dart';

class RootScreen extends StatefulWidget {
  const RootScreen({super.key});

  @override
  State<RootScreen> createState() => _RootScreenState();
}

class _RootScreenState extends State<RootScreen> {
  final GlobalKey<HomeScreenState> _homeKey = GlobalKey<HomeScreenState>();

  int _currentIndex = 0;
  BluetoothCharacteristic? _writableChar;

  // 루트에서 보유하는 현재 모드(홈과 동기화)
  ControlMode _currentMode = ControlMode.auto;

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  /// 설정 탭으로 이동 (HomeScreen의 onGoToProfile 콜백에 연결)
  void _goToSettingsPage() {
    setState(() {
      _currentIndex = 2; // 0: 홈, 1: 챗봇, 2: 설정
    });
  }

  /// 홈/FAB에서 AI 모드 전환 요청
  void _handleAiModeFromHome() {
    _homeKey.currentState?.setModeExternal(ControlMode.auto); // Home 쪽 단일 진입점
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('🤖 AI 모드로 전환됩니다.'),
        duration: Duration(seconds: 2),
      ),
    );
  }

  /// 탭 전환
  Future<void> _selectTab(int idx) async {
    // 챗봇/설정은 세로 고정
    if (idx == 1 || idx == 2) {
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    }
    setState(() => _currentIndex = idx);
  }

  /// 수동 진입: 먼저 Home 상태 manual로 전환(발행 포함) → 안내 → 가로 회전 → ManualPage
  Future<void> _handleManualTap() async {
    if (_writableChar == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('⚠️ 먼저 BLE 기기를 연결해주세요.'),
          duration: Duration(seconds: 2),
        ),
      );
      return;
    }

    _homeKey.currentState?.setModeExternal(ControlMode.manual);

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('3초 뒤 매뉴얼 화면으로 전환됩니다.'),
        duration: Duration(seconds: 2),
      ),
    );

    await Future.delayed(const Duration(seconds: 3));

    await SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);

    if (!mounted) return;
    await Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => ManualPage(writableChar: _writableChar!)),
    );

    // 복귀 시 세로로 복구
    await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
  }

  @override
  Widget build(BuildContext context) {
    // 하단 안전 영역(inset) 반영
    final bottomInset = MediaQuery.of(context).padding.bottom;

    final pages = [
      HomeScreen(
        key: _homeKey,
        onAiModeSwitch: _handleAiModeFromHome, // 홈 내부에서 호출해도 안전
        onGoToProfile: _goToSettingsPage,      // ✅ 설정으로 이동
        onConnect: (char) {
          setState(() {
            _writableChar = char;
          });
        },
        // ★ 루트↔홈 모드 동기화
        currentMode: _currentMode,
        onModeChange: (mode) {
          setState(() => _currentMode = mode);
        },
      ),
      const ChatbotPage(),
      const SettingsPage(), // ✅ 설정 페이지(클래스명 SettingsPage)
    ];

    return Scaffold(
      extendBody: true,
      backgroundColor: rootBackground,
      body: pages[_currentIndex],

      // 가운데 FAB = AI 모드 스위치
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
          onPressed: _handleAiModeFromHome,
          backgroundColor: primaryBlue,
          elevation: 0,
          child: const Icon(Icons.remove_red_eye, size: 30, color: Colors.white),
        ),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,

      bottomNavigationBar: Container(
        height: 85 + bottomInset,
        padding: EdgeInsets.only(bottom: bottomInset),
        decoration: const BoxDecoration(
          border: Border(top: BorderSide(color: Colors.white12, width: 1)),
          color: rootBackground,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            _buildTabItem(Icons.home, '홈', 0),
            _buildManualTabItem(Icons.menu_book_rounded, '수동'),
            const SizedBox(width: 60), // FAB 자리
            _buildTabItem(Icons.chat_bubble_rounded, '챗봇', 1),
            _buildTabItem(Icons.settings, '설정', 2),
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

  Widget _buildManualTabItem(IconData icon, String label) {
    // BLE 연결되면 파랑, 아니면 회색
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
