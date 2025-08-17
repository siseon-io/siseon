// lib/root_screen.dart
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'pages/home_screen.dart';
import 'pages/manual_page.dart';
import 'pages/chatbot_page.dart';
import 'pages/settings/settings_page.dart';
import 'pages/device_register_page.dart';

import 'package:siseon2/models/control_mode.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/services/device_cache_service.dart';

class RootScreen extends StatefulWidget {
  const RootScreen({super.key});
  @override
  State<RootScreen> createState() => _RootScreenState();
}

class _RootScreenState extends State<RootScreen> {
  final GlobalKey<HomeScreenState> _homeKey = GlobalKey<HomeScreenState>();

  static const bool _bleDebug = false;

  int _currentIndex = 0;
  ControlMode _currentMode = ControlMode.auto;
  int? _profileId;

  BluetoothCharacteristic? _writableChar;
  String? _deviceSerial;

  StreamSubscription<BluetoothConnectionState>? _devStateSub;

  // 🎨 컬러
  static const Color primaryBlue = Color(0xFF60A5FA);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  static const double _fabSize = 60;
  static const double _notchMargin = 4;


  void _snack(String msg) {
    if (!mounted) return;
    ScaffoldMessenger.of(context)
      ..hideCurrentSnackBar()
      ..showSnackBar(
        SnackBar(content: Text(msg)),
      );
  }

  @override
  void initState() {
    super.initState();
    FlutterBluePlus.setLogLevel(LogLevel.none);
    mqttService.connect();
    _loadProfileAndDevice();
    if (_bleDebug) _attachBleDebugListeners();
  }

  @override
  void dispose() {
    _devStateSub?.cancel();
    super.dispose();
  }

  void _attachBleDebugListeners() {
    FlutterBluePlus.adapterState.listen((state) {
      debugPrint('🛰️ [AdapterState] $state');
    });
    FlutterBluePlus.scanResults.listen((results) {
      for (final r in results) {
        debugPrint('📡 [Scan] name=${r.device.name}, id=${r.device.id}, rssi=${r
            .rssi}');
      }
    });
  }

  Future<void> _loadProfileAndDevice() async {
    await _loadProfileId();
    await _ensureDeviceSerialWithFallback();
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

  Future<void> _ensureDeviceSerialWithFallback() async {
    if (_profileId == null) {
      setState(() => _deviceSerial = null);
      return;
    }

    String? serial;

    try {
      final dev = await DeviceCacheService.loadDeviceForProfile(_profileId!);
      serial = dev?['serial']?.toString();
    } catch (_) {}

    if (serial == null || serial.isEmpty) {
      try {
        await DeviceCacheService.fetchAndCacheDevice(profileId: _profileId!);
        final dev2 = await DeviceCacheService.loadDeviceForProfile(_profileId!);
        serial = dev2?['serial']?.toString();
      } catch (_) {}
    }

    if (serial == null || serial.isEmpty) {
      try {
        final prefs = await SharedPreferences.getInstance();
        final legacyReg = prefs.getBool('isDeviceRegistered') ?? false;
        final legacySerial = prefs.getString('deviceSerial');
        if (legacyReg && legacySerial != null && legacySerial.isNotEmpty) {
          serial = legacySerial;
          await DeviceCacheService.saveDeviceForProfile(
            _profileId!,
            {'serial': legacySerial},
          );
        }
      } catch (_) {}
    }

    if ((serial == null || serial.isEmpty) && _writableChar != null) {
      serial = _deviceIdFromChar(_writableChar!);
    }

    setState(() {
      _deviceSerial = (serial != null && serial.isNotEmpty) ? serial : null;
    });
  }

  String _deviceIdFromChar(BluetoothCharacteristic ch) {
    try {
      return ch.device.id.str;
    } catch (_) {
      try {
        return ch.device.remoteId.str;
      } catch (_) {
        try {
          return ch.remoteId.str;
        } catch (_) {
          return ch.device.id.toString();
        }
      }
    }
  }

  Future<bool> _isCharConnected(BluetoothCharacteristic ch) async {
    try {
      final s = await ch.device.state.first;
      return s == BluetoothConnectionState.connected;
    } catch (_) {
      return false;
    }
  }

  void _goToSettingsPage() => setState(() => _currentIndex = 2);

  void _handleAiModeFromHome() {
    _homeKey.currentState?.setModeExternal(ControlMode.auto);
  }

  Future<void> _selectTab(int idx) async {
    if (idx == 1 || idx == 2) {
      await SystemChrome.setPreferredOrientations(
          [DeviceOrientation.portraitUp]);
    }
    if (idx == 0) {
      await _homeKey.currentState?.refreshFromRoot();
    }
    if (idx == 1) {
      await _loadProfileAndDevice();
    }
    setState(() => _currentIndex = idx);
  }

  Future<void> _showLoadingOverlay(String message, Duration dur) async {
    if (!mounted) return;

    // 다이얼로그를 전체 위에 띄우되 SafeArea로 하단 제스처바 침범 방지
    showGeneralDialog(
      context: context,
      barrierLabel: 'loading',
      barrierDismissible: false,
      barrierColor: Colors.black54, // 살짝 어둡게
      transitionDuration: const Duration(milliseconds: 120),
      pageBuilder: (ctx, a1, a2) {
        return SafeArea(
          child: Center(
            child: Material(
              color: const Color(0xFF0D1117),
              borderRadius: BorderRadius.circular(16),
              clipBehavior: Clip.antiAlias, // 경계 깨끗하게
              child: ConstrainedBox(
                constraints: const BoxConstraints(maxWidth: 320),
                child: Padding(
                  padding: const EdgeInsets.symmetric(vertical: 24, horizontal: 28),
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      const SizedBox(
                        width: 28,
                        height: 28,
                        child: CircularProgressIndicator(strokeWidth: 3),
                      ),
                      const SizedBox(height: 16),
                      Text(
                        message,
                        textAlign: TextAlign.center,
                        softWrap: true, // 혹시 모를 줄바꿈 오버플로 방지
                        style: const TextStyle(color: Colors.white, fontSize: 14),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );

    // 지정 시간 뒤 수동으로 닫기 (닫힐 때 레이아웃 튐 방지)
    await Future.delayed(dur);
    if (mounted) {
      Navigator.of(context, rootNavigator: true).pop();
    }
  }

  Future<bool> _requireDeviceRegistered() async {
    if (_profileId == null) {
      return false;
    }

    final dev = await DeviceCacheService.loadDeviceForProfile(_profileId!);
    final isRegistered = dev != null;

    if (isRegistered) {
      if (_deviceSerial == null || _deviceSerial!.isEmpty) {
        await _ensureDeviceSerialWithFallback();
      }
      return true;
    }

    final go = await showDialog<bool>(
      context: context,
      barrierDismissible: true,
      builder: (_) =>
          AlertDialog(
            backgroundColor: rootBackground,
            shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(12)),
            title: const Text('기기 등록이 필요합니다',
                style: TextStyle(
                    color: Colors.white, fontWeight: FontWeight.w700)),
            content: const Text('이 기능을 사용하려면 먼저 기기를 등록해주세요.',
                style: TextStyle(color: Colors.white70)),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(context, true),
                child: const Text('등록하기',
                    style: TextStyle(
                        color: primaryBlue, fontWeight: FontWeight.w700)),
              ),
              TextButton(
                onPressed: () => Navigator.pop(context, false),
                child: const Text(
                    '취소', style: TextStyle(color: Colors.white70)),
              ),
            ],
          ),
    );

    if (go == true) {
      final result =
      await Navigator.push(context,
          MaterialPageRoute(builder: (_) => const DeviceRegisterPage()));
      if (result == true) {
        await _ensureDeviceSerialWithFallback();
        return true;
      }
    }
    return false;
  }

  Future<bool> _publishControlMode(ControlMode nextMode,
      {required String deviceSerial}) async {
    final ok = await _requireDeviceRegistered();
    if (!ok || _profileId == null) return false;

    final payload = {
      'profile_id': _profileId.toString(),
      'previous_mode': _currentMode.name,
      'current_mode': nextMode.name,
    };

    try {
      mqttService.publish('/control_mode/$deviceSerial', payload);
      if (mounted) {
        setState(() => _currentMode = nextMode);
      }
      return true;
    } catch (_) {
      return false;
    }
  }

  Future<void> _handleAiModeTap() async {
    final ok = await _requireDeviceRegistered();
    if (!ok) return;

    String serial = (_deviceSerial != null && _deviceSerial!.isNotEmpty)
        ? _deviceSerial!
        : (_writableChar != null ? _deviceIdFromChar(_writableChar!) : '');

    if (serial.isEmpty) {
      return;
    }

    final ok2 = await _publishControlMode(
        ControlMode.auto, deviceSerial: serial);
    if (!ok2) return;

    _homeKey.currentState?.setModeExternal(ControlMode.auto);
  }

  Future<void> _handleManualTap() async {
    if (_profileId == null) {
      // 프로필 없을 때는 조용히 무시하거나 필요하면 안내
      // _snack('프로필을 먼저 선택/생성해주세요.');
      return;
    }

    final ok = await _requireDeviceRegistered();
    if (!ok) return;

    final ch = _writableChar;

    // ✅ 여기 추가: BLE 미연결 시 안내
    if (ch == null || !(await _isCharConnected(ch))) {
      _devStateSub?.cancel();
      _devStateSub = null;
      setState(() => _writableChar = null);
      _snack('블루투스를 먼저 연결해주세요.');
      return;
    }

    final serial = (_deviceSerial != null && _deviceSerial!.isNotEmpty)
        ? _deviceSerial!
        : ch.remoteId.toString();

    if (serial.isEmpty) {
      // _snack('기기 식별 정보를 확인할 수 없습니다.');
      return;
    }

    await _publishControlMode(ControlMode.manual, deviceSerial: serial);
    await _showLoadingOverlay('3초 후 가로모드로 전환됩니다.', const Duration(seconds: 3));

    await SystemChrome.setPreferredOrientations(
      [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight],
    );

    if (!mounted) return;

    final ControlMode? result = await Navigator.push<ControlMode>(
      context,
      MaterialPageRoute(
        builder: (_) => ManualPage(writableChar: ch, profileId: _profileId!),
      ),
    );

    await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);

    if (result != null && mounted) {
      setState(() => _currentMode = result);
    }
  }

  @override
  Widget build(BuildContext context) {
    final pages = [
      HomeScreen(
        key: _homeKey,
        onAiModeSwitch: _handleAiModeFromHome,
        onGoToProfile: _goToSettingsPage,
        currentMode: _currentMode,
        onModeChange: (mode) {
          if (_bleDebug) debugPrint('🔄 [RootScreen] mode=$mode');
          setState(() => _currentMode = mode);
        },
        onConnect: (c) async {
          setState(() => _writableChar = c);
          _devStateSub?.cancel();
          _devStateSub = c.device.state.listen((st) {
            if (_bleDebug) debugPrint('🔌 [DeviceState] $st');
            if (st == BluetoothConnectionState.disconnected) {
              _devStateSub?.cancel();
              _devStateSub = null;
              if (mounted) {
                setState(() => _writableChar = null);
              }
            }
          });

          if (_deviceSerial == null || _deviceSerial!.isEmpty) {
            final fromBle = _deviceIdFromChar(c);
            if (fromBle.isNotEmpty) setState(() => _deviceSerial = fromBle);
            _ensureDeviceSerialWithFallback();
          }
        },
      ),
      (_profileId == null) ? _buildNoProfileGate() : ChatbotPage(
          profileId: _profileId!),
      const SettingsPage(),
    ];

    // ✅ 전역 탭: GestureDetector → Listener 로 교체 (탭 가로채기 방지)
    return Listener(
      behavior: HitTestBehavior.translucent,
      onPointerDown: (_) {
        final f = FocusScope.of(context);
        if (!f.hasPrimaryFocus && f.focusedChild != null) f.unfocus();
      },
      child: Scaffold(
        backgroundColor: rootBackground,
        extendBody: true,
        resizeToAvoidBottomInset: false,
        body: IndexedStack(index: _currentIndex, children: pages),
        floatingActionButton: Transform.translate(
          offset: const Offset(0, 8),
          child: Container(
            width: _fabSize,
            height: _fabSize,
            decoration: const BoxDecoration(
              color: primaryBlue,
              shape: BoxShape.circle,
            ),
            child: FloatingActionButton(
              onPressed: _handleAiModeTap,
              backgroundColor: Colors.transparent,
              shape: const CircleBorder(),
              elevation: 6,
              clipBehavior: Clip.antiAlias,
              child: const Icon(
                  Icons.remove_red_eye, size: 26, color: Colors.white),
            ),
          ),
        ),
        floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,
        bottomNavigationBar: _bottomBar(),
      ),
    );
  }

  Widget _bottomBar() {
    final media = MediaQuery.of(context);
    final bottom = media.padding.bottom;
    final extra = media.viewInsets.bottom > 0
        ? 0.0
        : bottom; // 키보드 없을 때만 SafeArea 높이

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(height: 1, color: Colors.white24),
        // ⛔️ 하단 빈공간 터치가 본문으로 통과하지 않도록 Stack으로 흡수 레이어 추가
        Stack(
          children: [
            BottomAppBar(
              color: rootBackground,
              elevation: 0,
              child: SafeArea(
                top: false,
                left: false,
                right: false,
                bottom: true,
                child: SizedBox(
                  height: 50 + extra,
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.spaceAround,
                    children: [
                      _buildTabItem(Icons.home, '홈', 0),
                      _buildManualTabItem(),
                      const SizedBox(width: 56),
                      _buildTabItem(Icons.smart_toy_rounded, '챗봇', 1),
                      _buildTabItem(Icons.settings, '설정', 2),
                    ],
                  ),
                ),
              ),
            ),
            // ✅ 제스처 바(SafeArea) 구간: GestureDetector → AbsorbPointer (경쟁 제거)
            if (extra > 0)
              Positioned(
                left: 0,
                right: 0,
                bottom: 0,
                height: extra,
                child: AbsorbPointer(
                  absorbing: true,
                  child: Container(color: Colors.transparent),
                ),
              ),
          ],
        ),
      ],
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
        child: Text(
            '먼저 프로필을 선택/생성해주세요.', style: TextStyle(color: Colors.white70)),
      ),
    );
  }

  // ✅ 탭: GestureDetector → InkWell (안정적인 제스처 처리)
  Widget _buildTabItem(IconData icon, String label, int idx) {
    final isSelected = _currentIndex == idx;
    final color = isSelected ? primaryBlue : inactiveGrey;
    return SizedBox(
      width: 60,
      height: 56,
      child: InkWell(
        borderRadius: BorderRadius.circular(12),
        onTap: () => _selectTab(idx),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, color: color, size: 24),
            const SizedBox(height: 3),
            Text(
              label,
              style: TextStyle(
                  color: color, fontSize: 11, fontWeight: FontWeight.w500),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildManualTabItem() {
    return SizedBox(
      width: 60,
      height: 56,
      child: InkWell(
        borderRadius: BorderRadius.circular(12),
        onTap: _handleManualTap,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.open_with_rounded, color: inactiveGrey, size: 24),
            const SizedBox(height: 3),
            const Text(
              '수동',
              style: TextStyle(
                color: inactiveGrey,
                fontSize: 11,
                fontWeight: FontWeight.w500,
              ),
            ),
          ],
        ),
      ),
    );
  }
}