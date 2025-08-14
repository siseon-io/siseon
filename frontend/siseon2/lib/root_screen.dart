// lib/root_screen.dart
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:shared_preferences/shared_preferences.dart'; // ✅ 레거시 폴백용

import 'pages/home_screen.dart';
import 'pages/manual_page.dart';
import 'pages/chatbot_page.dart';
import 'pages/settings/settings_page.dart';
import 'pages/device_register_page.dart'; // ✅ 기기 등록 페이지 이동용

import 'package:siseon2/models/control_mode.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/mqtt_service.dart';          // ✅ MQTT
import 'package:siseon2/services/device_cache_service.dart';  // ✅ deviceSerial 로드/갱신

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

  // ✅ HomeScreen에서 연결 시 콜백으로 받는다 (ble_session 제거)
  BluetoothCharacteristic? _writableChar;
  String? _deviceSerial; // DeviceCacheService/레거시/BLE에서 폴백

  // ✅ 디바이스 상태 감시 (끊기면 즉시 정리)
  StreamSubscription<BluetoothConnectionState>? _devStateSub;

  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color inactiveGrey = Colors.grey;

  @override
  void initState() {
    super.initState();
    FlutterBluePlus.setLogLevel(LogLevel.none);
    mqttService.connect();         // ✅ MQTT 선연결 시도
    _loadProfileAndDevice();       // 프로필/디바이스 동기화
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
        debugPrint('📡 [Scan] name=${r.device.name}, id=${r.device.id}, rssi=${r.rssi}');
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

  // ✅ 시리얼 확보: 캐시 → 서버조회후캐시 → 레거시 키 → BLE 특성 ID
  Future<void> _ensureDeviceSerialWithFallback() async {
    if (_profileId == null) {
      setState(() => _deviceSerial = null);
      return;
    }

    String? serial;

    // 1) 프로필별 캐시
    try {
      final dev = await DeviceCacheService.loadDeviceForProfile(_profileId!);
      serial = dev?['serial']?.toString();
    } catch (_) {}

    // 2) 서버에서 조회해 캐시 갱신 후 재조회
    if (serial == null || serial.isEmpty) {
      try {
        await DeviceCacheService.fetchAndCacheDevice(profileId: _profileId!);
        final dev2 = await DeviceCacheService.loadDeviceForProfile(_profileId!);
        serial = dev2?['serial']?.toString();
      } catch (_) {}
    }

    // 3) 레거시 키 폴백 (deviceSerial/isDeviceRegistered)
    if (serial == null || serial.isEmpty) {
      try {
        final prefs = await SharedPreferences.getInstance();
        final legacyReg = prefs.getBool('isDeviceRegistered') ?? false;
        final legacySerial = prefs.getString('deviceSerial');
        if (legacyReg && legacySerial != null && legacySerial.isNotEmpty) {
          serial = legacySerial;
          // 👉 프로필별 캐시에 이식(앞으로는 여기서 읽히게)
          await DeviceCacheService.saveDeviceForProfile(
            _profileId!,
            {'serial': legacySerial},
          );
        }
      } catch (_) {}
    }

    // 4) 그래도 없으면 BLE에서 폴백 (연결돼 있다면)
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
        // ignore: deprecated_member_use
        return ch.device.remoteId.str;
      } catch (_) {
        try {
          // ignore: deprecated_member_use
          return ch.remoteId.str;
        } catch (_) {
          return ch.device.id.toString();
        }
      }
    }
  }

  // ✅ 실제 연결 상태 한 번 더 확인 (진입 차단용)
  Future<bool> _isCharConnected(BluetoothCharacteristic ch) async {
    try {
      final s = await ch.device.state.first;
      return s == BluetoothConnectionState.connected;
    } catch (_) {
      return false;
    }
  }

  void _goToSettingsPage() {
    setState(() => _currentIndex = 2);
  }

  // HomeScreen 토글이 AI로 바꿀 때 들어오는 콜백(로컬 UI용)
  void _handleAiModeFromHome() {
    _homeKey.currentState?.setModeExternal(ControlMode.auto); // 홈 카드 갱신
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('🤖 AI 모드로 전환됩니다.'), duration: Duration(seconds: 2)),
    );
  }

  Future<void> _selectTab(int idx) async {
    if (idx == 1 || idx == 2) {
      await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    }
    if (idx == 1) {
      await _loadProfileAndDevice(); // 챗봇/수동 들어갈 때 최신화
    }
    setState(() => _currentIndex = idx);
  }

  // ── 로딩 오버레이 ─────────────────────────────────────────────
  Future<void> _showLoadingOverlay(String message, Duration dur) async {
    if (!mounted) return;
    showDialog(
      context: context,
      barrierDismissible: false,
      useRootNavigator: true,
      builder: (_) => WillPopScope(
        onWillPop: () async => false,
        child: Center(
          child: Container(
            padding: const EdgeInsets.symmetric(vertical: 24, horizontal: 28),
            decoration: BoxDecoration(
              color: const Color(0xFF0D1117).withOpacity(0.9),
              borderRadius: BorderRadius.circular(16),
              border: Border.all(color: Colors.white24),
            ),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const CircularProgressIndicator(),
                const SizedBox(height: 16),
                Text(message,
                    textAlign: TextAlign.center,
                    style: const TextStyle(color: Colors.white, fontSize: 14)),
              ],
            ),
          ),
        ),
      ),
    );
    await Future.delayed(dur);
    if (mounted) Navigator.of(context, rootNavigator: true).pop();
  }

  // ── 기기 등록 가드: 미등록이면 알림 → 등록 페이지 ─────────────
  // ── 기기 등록 가드: 미등록이면 알림 → 등록 페이지 ─────────────
  Future<bool> _requireDeviceRegistered() async {
    if (_profileId == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ 먼저 프로필을 선택/생성해주세요. (설정 탭)')),
      );
      return false;
    }

    // 캐시에서 등록 여부 확인
    final dev = await DeviceCacheService.loadDeviceForProfile(_profileId!);
    final isRegistered = dev != null;

    if (isRegistered) {
      // 시리얼 없으면 한 번 더 보강
      if (_deviceSerial == null || _deviceSerial!.isEmpty) {
        await _ensureDeviceSerialWithFallback();
      }
      return true;
    }

    // 알림 → 등록 페이지 이동
    final go = await showDialog<bool>(
      context: context,
      barrierDismissible: true,
      builder: (_) => AlertDialog(
        backgroundColor: rootBackground,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        title: const Text(
          '기기 등록이 필요합니다',
          style: TextStyle(color: Colors.white, fontWeight: FontWeight.w700),
        ),
        content: const Text(
          '이 기능을 사용하려면 먼저 기기를 등록해주세요.',
          style: TextStyle(color: Colors.white70),
        ),
        actions: [
          // ✅ 순서 변경: 등록하기(왼쪽) → 취소(오른쪽)
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text(
              '등록하기',
              style: TextStyle(color: primaryBlue, fontWeight: FontWeight.w700),
            ),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('취소', style: TextStyle(color: Colors.white70)),
          ),
        ],
      ),
    );

    if (go == true) {
      final result = await Navigator.push(
        context,
        MaterialPageRoute(builder: (_) => const DeviceRegisterPage()),
      );
      if (result == true) {
        await _ensureDeviceSerialWithFallback();
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('✅ 기기 등록 완료')),
        );
        return true;
      }
    }

    return false;
  }


  // ── MQTT 발행 ─────────────────────────────────────────────
  Future<bool> _publishControlMode(ControlMode nextMode, {required String deviceSerial}) async {
    // 👉 발행 전 등록 가드
    final ok = await _requireDeviceRegistered();
    if (!ok) return false;

    if (_profileId == null) return false;

    final payload = {
      'profile_id': _profileId.toString(),
      'previous_mode': _currentMode.name, // 현재 상태
      'current_mode': nextMode.name,
    };

    try {
      // 홈스크린과 동일 규격: /control_mode/<deviceSerial>
      mqttService.publish('/control_mode/$deviceSerial', payload);
      if (mounted) {
        setState(() => _currentMode = nextMode); // 상태 갱신
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('📶 MQTT 발행 완료: ${nextMode.name}')),
        );
      }
      return true;
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('❌ MQTT 발행 실패: $e')),
        );
      }
      return false;
    }
  }

  // ── AI 모드 탭( FAB ) : 등록 가드 + MQTT 발행 + 홈카드 동기화 ─────────────
  Future<void> _handleAiModeTap() async {
    // 등록 여부 확인
    final ok = await _requireDeviceRegistered();
    if (!ok) return;

    // 시리얼 확보
    String serial = (_deviceSerial != null && _deviceSerial!.isNotEmpty)
        ? _deviceSerial!
        : (_writableChar != null ? _deviceIdFromChar(_writableChar!) : '');

    if (serial.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ 디바이스 ID(시리얼)를 확인할 수 없습니다. 등록/연결 후 다시 시도해주세요.')),
      );
      return;
    }

    // MQTT 발행
    final ok2 = await _publishControlMode(ControlMode.auto, deviceSerial: serial);
    if (!ok2) return;

    // 홈 카드 동기화(로컬)
    _homeKey.currentState?.setModeExternal(ControlMode.auto);
  }

  // ── 수동 탭 핸들러 (등록 가드 + 실연결 검증 + MQTT → 로딩 → ManualPage) ────────────────
  Future<void> _handleManualTap() async {
    if (_profileId == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ 먼저 프로필을 선택/생성해주세요. (설정 탭)')),
      );
      return;
    }

    // 0) 등록 가드
    final ok = await _requireDeviceRegistered();
    if (!ok) return;

    final ch = _writableChar;

    // 1) characteristic 존재 & 실제 연결 여부 이중 검증
    if (ch == null || !(await _isCharConnected(ch))) {
      // 끊겼으면 흔적 정리
      _devStateSub?.cancel();
      _devStateSub = null;
      setState(() => _writableChar = null);

      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ BLE가 연결되지 않았습니다. 홈에서 먼저 연결해주세요.')),
      );
      // 필요 시 홈 탭으로 이동하려면 아래 주석 해제
      // await _selectTab(0);
      return;
    }

    final serial = (_deviceSerial != null && _deviceSerial!.isNotEmpty)
        ? _deviceSerial!
        : ch.remoteId.toString();

    if (serial.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⚠️ 디바이스 ID(시리얼)를 확인할 수 없습니다.')),
      );
      return;
    }

    // 2) 수동 모드로 MQTT 발행
    await _publishControlMode(ControlMode.manual, deviceSerial: serial);

    // 3) 3초 로딩
    await _showLoadingOverlay('잠깐만요, 자료 뒤적이는 중 📚', const Duration(seconds: 3));

    // 4) 가로 고정 → ManualPage 진입
    await SystemChrome.setPreferredOrientations(
      [DeviceOrientation.landscapeLeft, DeviceOrientation.landscapeRight],
    );

    if (!mounted) return;

    // ✅ ManualPage가 닫힐 때 ControlMode를 결과로 돌려줌 (auto 기대)
    final ControlMode? result = await Navigator.push<ControlMode>(
      context,
      MaterialPageRoute(
        builder: (_) => ManualPage(
          writableChar: ch,
          profileId: _profileId!,
        ),
      ),
    );

    // 세로 고정 복구
    await SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);

    // ✅ 결과값을 전역 상태와 홈 카드에 반영
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
        // 홈 토글 → 로컬 알림/동기화
        onGoToProfile: _goToSettingsPage,
        currentMode: _currentMode,
        onModeChange: (mode) {
          if (_bleDebug) debugPrint('🔄 [RootScreen] mode=$mode');
          setState(() => _currentMode = mode);
        },
        // ✅ BLE 연결되면 여기로 characteristic 넘어옴
        onConnect: (c) async {
          setState(() => _writableChar = c);

          // 🔎 디바이스 상태 감시: 끊기면 즉시 정리
          _devStateSub?.cancel();
          _devStateSub = c.device.state.listen((st) {
            if (_bleDebug) debugPrint('🔌 [DeviceState] $st');
            if (st == BluetoothConnectionState.disconnected) {
              _devStateSub?.cancel();
              _devStateSub = null;
              if (mounted) {
                setState(() => _writableChar = null);
                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(
                      content: Text('🔌 BLE 연결이 끊어졌습니다. 홈에서 다시 연결해주세요.')),
                );
              }
            }
          });

          // 시리얼 폴백 처리 그대로 유지
          if (_deviceSerial == null || _deviceSerial!.isEmpty) {
            final fromBle = _deviceIdFromChar(c);
            if (fromBle.isNotEmpty) {
              setState(() => _deviceSerial = fromBle);
            }
            // 동시에 캐시/서버 fetch도 백그라운드로 시도
            // ignore: unawaited_futures
            _ensureDeviceSerialWithFallback();
          }
        },
      ),
      (_profileId == null)
          ? _buildNoProfileGate()
          : ChatbotPage(profileId: _profileId!),
      const SettingsPage(),
    ];

// ⛳️ build() 안의 return 부분만 변경
    return Scaffold(
      backgroundColor: rootBackground,
      // 👇 여기! 한 줄 교체
      // body: pages[_currentIndex],
      body: IndexedStack(
        index: _currentIndex,
        children: pages,
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _handleAiModeTap,
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
        child: Text('먼저 프로필을 선택/생성해주세요.',
            style: TextStyle(color: Colors.white70)),
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
            const SizedBox(height: 2),
            const Text('수동', style: TextStyle(color: inactiveGrey)),
          ],
        ),
      ),
    );
  }
}
