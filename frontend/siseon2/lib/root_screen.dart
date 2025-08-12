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

  // 🔇 BLE 디버그 토글 (필요할 때만 true로)
  static const bool _bleDebug = false;

  int _currentIndex = 0;
  BluetoothCharacteristic? _writableChar;
  ControlMode _currentMode = ControlMode.auto;
  int? _profileId;

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  @override
  void initState() {
    super.initState();

    // 🔇 플러그인 로그 전부 끔 (다른 곳에서 verbose로 바뀌지 않게 여기서 명시)
    FlutterBluePlus.setLogLevel(LogLevel.none);

    _loadProfileId();

    // 🔍 필요할 때만 디버그 리스너 부착
    if (_bleDebug) _attachBleDebugListeners();
  }

  void _attachBleDebugListeners() {
    FlutterBluePlus.adapterState.listen((state) {
      debugPrint('🛰️ [AdapterState] 어댑터 상태: $state');
    });

    FlutterBluePlus.scanResults.listen((results) {
      for (final r in results) {
        debugPrint(
          '📡 [ScanResult] name=${r.device.name}, '
              'id=${r.device.id}, RSSI=${r.rssi}, '
              'serviceUuids=${r.advertisementData.serviceUuids}',
        );
      }
    });
  }

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

  void _goToSettingsPage() {
    setState(() => _currentIndex = 2);
  }

  void _handleAiModeFromHome() {
    _homeKey.currentState?.setModeExternal(ControlMode.auto);
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('🤖 AI 모드로 전환됩니다.'), duration: Duration(seconds: 2)),
    );
  }

  Future<void> _selectTab(int idx) async {
    if (idx == 1 || idx == 2) {
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    }
    if (idx == 1) {
      await _loadProfileId();
    }
    setState(() => _currentIndex = idx);
  }

  Future<void> _handleManualTap() async {
    if (_profileId == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ 먼저 프로필을 선택/생성해주세요. (설정 탭)')),
      );
      return;
    }
    if (_writableChar == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ 먼저 BLE 기기를 연결해주세요.')),
      );
      return;
    }

    // ❌ 여기서 HomeScreen 모드 전환하지 말자 (간접 끊김 원인 차단)
    // _homeKey.currentState?.setModeExternal(ControlMode.manual);

    // ❌ 3초 대기 제거
    // ScaffoldMessenger.of(context).showSnackBar(
    //   const SnackBar(content: Text('3초 뒤 매뉴얼 화면으로 전환됩니다.')),
    // );
    // await Future.delayed(const Duration(seconds: 3));

    await SystemChrome.setPreferredOrientations(
      [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight],
    );

    if (!mounted) return;
    await Navigator.push(
      context,
      MaterialPageRoute(
        builder: (_) => ManualPage(
          writableChar: _writableChar!,
          profileId: _profileId!,
        ),
      ),
    );

    await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
  }

  @override
  Widget build(BuildContext context) {
    final pages = [
      HomeScreen(
        key: _homeKey,
        onAiModeSwitch: _handleAiModeFromHome,
        onGoToProfile: _goToSettingsPage,
        onConnect: (char) {
          // 🔕 디버그 출력 제거 (필요하면 _bleDebug로 감싸기)
          if (_bleDebug) {
            debugPrint('🔗 [RootScreen] WritableChar 수신: ${char.uuid}');
          }
          setState(() => _writableChar = char);
        },
        currentMode: _currentMode,
        onModeChange: (mode) {
          if (_bleDebug) debugPrint('🔄 [RootScreen] 모드 변경: $mode');
          setState(() => _currentMode = mode);
        },
      ),
      (_profileId == null)
          ? _buildNoProfileGate()
          : ChatbotPage(profileId: _profileId!),
      const SettingsPage(),
    ];

    return Scaffold(
      backgroundColor: rootBackground,
      body: pages[_currentIndex],
      floatingActionButton: FloatingActionButton(
        onPressed: _handleAiModeFromHome,
        backgroundColor: primaryBlue,
        child: const Icon(Icons.remove_red_eye, size: 30, color: Colors.white),
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,
      bottomNavigationBar: _bottomBar(),
    );
  }

  Widget _bottomBar() {
    return Container(
      height: 85 + MediaQuery.of(context).padding.bottom,
      padding: EdgeInsets.only(bottom: MediaQuery.of(context).padding.bottom),
      color: rootBackground,
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
    );
  }

  Widget _buildNoProfileGate() {
    return Scaffold(
      backgroundColor: rootBackground,
      appBar: AppBar(
        backgroundColor: rootBackground,
        title: const Text('챗봇', style: TextStyle(color: Colors.white)),
      ),
      body: const Center(
        child: Text('먼저 프로필을 선택/생성해주세요.', style: TextStyle(color: Colors.white70)),
      ),
    );
  }

  Widget _buildTabItem(IconData icon, String label, int idx) {
    final isSelected = _currentIndex == idx;
    final color = isSelected ? primaryBlue : inactiveGrey;
    return GestureDetector(
      onTap: () => _selectTab(idx),
      child: SizedBox(
        width: 65,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: color),
            Text(label, style: TextStyle(color: color)),
          ],
        ),
      ),
    );
  }

  Widget _buildManualTabItem(IconData icon, String label) {
    return GestureDetector(
      onTap: _handleManualTap,
      child: SizedBox(
        width: 65,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: inactiveGrey),
            Text(label, style: const TextStyle(color: inactiveGrey)),
          ],
        ),
      ),
    );
  }
}
