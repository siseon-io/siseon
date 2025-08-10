// lib/root_screen.dart
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import 'pages/home_screen.dart';
import 'pages/manual_page.dart';
import 'pages/chatbot_page.dart';
import 'pages/settings/settings_page.dart';

import 'package:siseon2/models/control_mode.dart';
import 'package:siseon2/services/profile_cache_service.dart';

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

  // ✅ 선택된 프로필 ID (SharedPreferences에서 로드)
  int? _profileId;

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  @override
  void initState() {
    super.initState();
    _loadProfileId(); // 앱 시작 시 한 번 로드
  }

  /// ✅ 프로필 ID 로드: loadProfile()에서 profileId 또는 id를 읽어 정수로 변환
  Future<void> _loadProfileId() async {
    try {
      final p = await ProfileCacheService.loadProfile();
      final raw = p == null ? null : (p['profileId'] ?? p['id']);
      setState(() {
        if (raw is int) {
          _profileId = raw;
        } else if (raw is String) {
          _profileId = int.tryParse(raw);
        } else {
          _profileId = null;
        }
      });
    } catch (_) {
      setState(() => _profileId = null);
    }
  }

  /// 설정 탭으로 이동 (HomeScreen의 onGoToProfile 콜백에 연결)
  void _goToSettingsPage() {
    setState(() {
      _currentIndex = 2; // 0: 홈, 1: 챗봇, 2: 설정
    });
  }

  /// 홈/FAB에서 AI 모드 전환 요청
  void _handleAiModeFromHome() {
    _homeKey.currentState?.setModeExternal(ControlMode.auto);
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('🤖 AI 모드로 전환됩니다.'),
        duration: Duration(seconds: 2),
      ),
    );
  }

  /// 탭 전환
  Future<void> _selectTab(int idx) async {
    if (idx == 1 || idx == 2) {
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    }
    // ✅ 챗봇 탭 진입 시 최신 프로필 다시 로드 (설정에서 바꿨을 수 있음)
    if (idx == 1) {
      await _loadProfileId();
    }
    setState(() => _currentIndex = idx);
  }

  /// 수동 진입: BLE 연결 검사 후 가이드 및 전환
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
    await SystemChrome.setPreferredOrientations(
      [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight],
    );

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
    final bottomInset = MediaQuery.of(context).padding.bottom;

    final pages = [
      HomeScreen(
        key: _homeKey,
        onAiModeSwitch: _handleAiModeFromHome,
        onGoToProfile: _goToSettingsPage,
        onConnect: (char) {
          setState(() {
            _writableChar = char;
          });
        },
        currentMode: _currentMode,
        onModeChange: (mode) {
          setState(() => _currentMode = mode);
        },
      ),
      // ✅ 프로필 없으면 안내, 있으면 챗봇 페이지
      (_profileId == null)
          ? _buildNoProfileGate()
          : ChatbotPage(profileId: _profileId!),
      const SettingsPage(),
    ];

    return Scaffold(
      extendBody: true,
      backgroundColor: rootBackground,
      body: pages[_currentIndex],
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
            const SizedBox(width: 60),
            _buildTabItem(Icons.chat_bubble_rounded, '챗봇', 1),
            _buildTabItem(Icons.settings, '설정', 2),
          ],
        ),
      ),
    );
  }

  /// ✅ 프로필 없을 때 챗봇 탭에 보여줄 가드 화면
  Widget _buildNoProfileGate() {
    return Scaffold(
      backgroundColor: rootBackground,
      appBar: AppBar(
        backgroundColor: rootBackground,
        elevation: 0,
        title: const Text('챗봇', style: TextStyle(color: Colors.white)),
      ),
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Icon(Icons.person_outline, color: Colors.white70, size: 56),
            const SizedBox(height: 12),
            const Text(
              '먼저 프로필을 선택/생성해주세요.',
              style: TextStyle(color: Colors.white70),
            ),
            const SizedBox(height: 16),
            ElevatedButton(
              onPressed: () => _selectTab(2),
              style: ElevatedButton.styleFrom(backgroundColor: primaryBlue),
              child: const Text('설정으로 이동'),
            ),
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
    const color = inactiveGrey; // 수동 탭은 항상 회색으로 표시
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
              style: const TextStyle(
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
