// lib/pages/home_screen.dart
import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart'; // ✅ 모드 저장/복원 (프로필별)

import 'package:siseon2/models/control_mode.dart';
import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/services/preset_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/device_cache_service.dart';
import 'package:siseon2/services/stats_service.dart';
import 'package:siseon2/services/ble_session.dart';

import 'package:siseon2/pages/ble_scan_screen.dart';
import 'package:siseon2/pages/device_register_page.dart';
import 'package:siseon2/pages/settings/preset_page.dart';
import 'package:siseon2/pages/settings/stats_page.dart';
import 'package:siseon2/pages/settings/edit_profile.dart';

import 'package:siseon2/widgets/rect_card.dart';

enum PostureBannerStatus { good, bad, none }

class HomeScreen extends StatefulWidget {
  final VoidCallback onAiModeSwitch;
  final VoidCallback onGoToProfile;
  final void Function(BluetoothCharacteristic writableChar)? onConnect;

  final ControlMode currentMode;
  final void Function(ControlMode newMode) onModeChange;

  const HomeScreen({
    super.key,
    required this.onAiModeSwitch,
    required this.onGoToProfile,
    this.onConnect,
    required this.currentMode,
    required this.onModeChange,
  });

  @override
  HomeScreenState createState() => HomeScreenState();
}

class HomeScreenState extends State<HomeScreen> with WidgetsBindingObserver {
  // Colors
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color headerGrey = Color(0xFF161B22);
  static const Color cardGrey = Color(0xFF1E293B);
  static const Color errorRed = Color(0xFFF87171);

  static const double _rightCardHeight = 64.0;
  static const double _rightGap = 8.0;
  static const double _sectionIconSize = 18;

  ImageProvider? _avatarProvider(dynamic src) {
    final s = (src ?? '').toString().trim();
    if (s.isEmpty) return null;
    if (s.startsWith('http')) return NetworkImage(s);
    if (s.startsWith('assets/')) return AssetImage(s);
    return null; // 규격 밖이면 기본 아이콘 유지
  }

  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];

  bool _isBluetoothOn = false;
  bool _isDeviceRegistered = false;
  bool _deviceStateReady = false;
  String? _deviceSerial;
  String? _targetCharUuid;

  // BLE session snapshot (UI 표시용)
  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

  late ControlMode _mode;

  int _goodSecToday = 0;
  int _badSecToday = 0;

  // ⬇️ 폴링(1분) & 로딩상태
  Timer? _pollTimer;
  bool _isPolling = false;

  DateTime? _postureTime;
  bool _loadingPosture = false;
  PostureBannerStatus _postureStatus = PostureBannerStatus.none;

  bool get _bleReady => bleSession.isReady;

  void _copyFromSession() {
    _connectedDevice = bleSession.device;
    _writableChar = bleSession.char;
  }

  void _onBleSessionChanged() {
    if (!mounted) return;
    setState(_copyFromSession);
  }

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this); // 라이프사이클 감시
    _mode = widget.currentMode;
    FlutterBluePlus.setLogLevel(LogLevel.none);
    _initPermissions();
    _checkBluetoothState();
    _syncProfileAndDevice();
    _loadLatestPosture(); // 첫 로딩은 스피너 보임
    _startPolling();      // 1분 폴링 시작

    bleSession.addListener(_onBleSessionChanged);
    _copyFromSession();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    Future.microtask(_syncProfileAndDevice);
  }

  @override
  void didUpdateWidget(covariant HomeScreen oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.currentMode != widget.currentMode) {
      setState(() => _mode = widget.currentMode);
    }
  }

  @override
  void dispose() {
    _stopPolling();
    bleSession.removeListener(_onBleSessionChanged);
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  // 앱 라이프사이클: 복귀 시 즉시 새로고침, 백그라운드 시 폴링 중단
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.resumed) {
      _refreshSilently();
      _startPolling();
    } else if (state == AppLifecycleState.paused ||
        state == AppLifecycleState.inactive ||
        state == AppLifecycleState.detached) {
      _stopPolling();
    }
  }

  void _startPolling() {
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(const Duration(minutes: 1), (_) => _refreshSilently());
  }

  void _stopPolling() {
    _pollTimer?.cancel();
    _pollTimer = null;
  }

  Future<void> _refreshSilently() async {
    if (_isPolling) return;
    _isPolling = true;
    try {
      await _loadLatestPosture(silent: true); // 스피너 없이
      await _loadDailyStats();                // 오늘 통계도 갱신
    } finally {
      _isPolling = false;
    }
  }

  // ───────────── 공용 헬퍼: 프로필별 모드 키 ─────────────
  String? _modeKeyForCurrentProfile() {
    final pid = _profile?['id'];
    if (pid == null) return null;
    return 'mode:profile:$pid';
  }

  // ✅ 외부에서 모드 동기화만 (발행 없음) + 프로필별 저장
  void setModeLocal(ControlMode newMode) {
    if (_mode == newMode) return;
    setState(() => _mode = newMode);
    _persistMode(newMode); // 프로필별 저장
    widget.onModeChange(newMode);
  }

  void setModeExternal(ControlMode newMode) => setModeLocal(newMode);

  // ─────────── 권한/상태 ───────────
  Future<void> _initPermissions() async {
    final bleScan = await Permission.bluetoothScan.request();
    final bleConnect = await Permission.bluetoothConnect.request();
    final location = await Permission.location.request();
    if (!bleScan.isGranted || !bleConnect.isGranted || !location.isGranted) {
      if (await Permission.bluetoothConnect.isPermanentlyDenied) {
        openAppSettings();
      }
    }
  }

  Future<void> _checkBluetoothState() async {
    final isOn = await FlutterBluePlus.isOn;
    if (!mounted) return;
    setState(() => _isBluetoothOn = isOn);
  }

  // ─────────── 프로필/디바이스 동기화 ───────────
  Future<void> _syncProfileAndDevice() async {
    final profile = await ProfileCacheService.loadProfile();

    // 프로필 전환 직후: 로딩 동안 이전 프로필 모드가 비치지 않도록 즉시 OFF 표기
    setState(() {
      _profile = profile;
      _deviceStateReady = false;
      _isDeviceRegistered = false;
      _deviceSerial = null;
      _targetCharUuid = null;
      _mode = ControlMode.off; // 임시 OFF
    });

    await _loadProfileAndPresets();
    await _loadDailyStats();
    final pid = profile?['id'] as int?;
    await _loadDeviceStateFor(pid);
  }

  Future<void> _loadDeviceStateFor(int? profileId) async {
    if (profileId == null) {
      if (!mounted) return;
      setState(() {
        _isDeviceRegistered = false;
        _deviceSerial = null;
        _targetCharUuid = null;
        _deviceStateReady = true;
      });
      setModeLocal(ControlMode.off);
      return;
    }

    final device = await DeviceCacheService.loadDeviceForProfile(profileId);
    if (!mounted) return;
    setState(() {
      _isDeviceRegistered = device != null;
      _deviceSerial = device?['serial'];
      _targetCharUuid = device?['targetCharUuid'] ?? device?['charUuid'] ?? device?['characteristicUuid'];
      _deviceStateReady = true;
    });

    if (!_isDeviceRegistered) {
      setModeLocal(ControlMode.off);
    } else {
      await _restoreModeForProfile(); // 프로필별 복원 (발행 없음)
    }
  }

  Future<void> _loadProfileAndPresets() async {
    final profile = await ProfileCacheService.loadProfile();
    if (profile == null) return;
    final presets = await PresetService.fetchPresets(profile['id']);
    if (!mounted) return;
    setState(() {
      _profile = profile;
      _presets = presets.take(3).toList();
    });
  }

  // ─────────── 통계 로딩 ───────────
  Future<void> _loadDailyStats() async {
    try {
      final profile = await ProfileCacheService.loadProfile();
      final profileId = profile?['profileId'] ?? profile?['id'];
      if (profileId == null) return;

      final now = DateTime.now();
      final start = DateTime(now.year, now.month, now.day);

      final daily = await StatsService.fetchPostureStats(
        profileId: profileId,
        period: 'daily',
        from: start,
        to: now,
      );

      int g = 0, b = 0;
      for (final s in daily) {
        final v = _valid(s);
        if (v == null) continue;
        if (v) {
          g += s.durationSeconds;
        } else {
          b += s.durationSeconds;
        }
      }
      if (!mounted) return;
      setState(() {
        _goodSecToday = g;
        _badSecToday = b;
      });
    } catch (_) {
      if (!mounted) return;
      setState(() {
        _goodSecToday = 0;
        _badSecToday = 0;
      });
    }
  }

  Future<void> _loadLatestPosture({bool silent = false}) async {
    try {
      if (!silent) setState(() => _loadingPosture = true);

      final profile = await ProfileCacheService.loadProfile();
      final profileId = profile?['profileId'] ?? profile?['id'];
      if (profileId == null) {
        setState(() {
          _postureStatus = PostureBannerStatus.none;
          _postureTime = null;
          _loadingPosture = false;
        });
        return;
      }

      final now = DateTime.now();
      final lookback = now.subtract(const Duration(hours: 12));

      final list = await StatsService.fetchPostureStats(
        profileId: profileId,
        period: 'daily',
        from: lookback,
        to: now,
      );

      if (list.isEmpty) {
        setState(() {
          _postureStatus = PostureBannerStatus.none;
          _postureTime = null;
          _loadingPosture = false;
        });
        return;
      }

      list.sort((a, b) => a.endAt.compareTo(b.endAt));
      final latest = list.last;
      final v = _valid(latest);

      setState(() {
        _postureStatus = (v == true)
            ? PostureBannerStatus.good
            : (v == false)
            ? PostureBannerStatus.bad
            : PostureBannerStatus.none;
        _postureTime = latest.endAt.toLocal();
        _loadingPosture = false;
      });
    } catch (_) {
      if (!mounted) return;
      setState(() {
        _postureStatus = PostureBannerStatus.none;
        _postureTime = null;
        _loadingPosture = false;
      });
    }
  }

  bool? _valid(PostureStats s) {
    try {
      final v = (s as dynamic).validPosture;
      if (v is bool) return v;
    } catch (_) {}
    return null;
  }

  // ─────────── 등록/스캔/연결 ───────────
  Future<void> _registerDevice() async {
    final result = await Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => const DeviceRegisterPage()),
    );
    if (result == true) {
      final pid = _profile?['id'] as int?;
      await _loadDeviceStateFor(pid);
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('✅ 기기 등록 완료')),
      );
    }
  }

  Future<void> _requestPairAndScan() async {
    if (_profile == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 프로필 정보가 없습니다.')),
      );
      return;
    }
    if (!_isDeviceRegistered) {
      await _registerDevice();
      return;
    }

    final int profileId = _profile!['id'];
    final topic = '/request_pair/$_deviceSerial';
    mqttService.publish(topic, {'profile_id': profileId.toString()});
    await Future.delayed(const Duration(milliseconds: 350));

    final result = await Navigator.push<Map<String, dynamic>>(
      context,
      MaterialPageRoute(
        builder: (_) => BleScanScreen(targetCharUuid: _targetCharUuid),
      ),
    );

    if (bleSession.isReady) {
      setState(_copyFromSession);
      widget.onConnect?.call(bleSession.char!);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('✅ BLE 연결 성공')),
        );
      }
      return;
    }

    if (result != null) {
      setState(() {
        _connectedDevice = result['device'] as BluetoothDevice?;
        _writableChar = result['writableChar'] as BluetoothCharacteristic?;
      });
      if (_writableChar != null) {
        widget.onConnect?.call(_writableChar!);
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text('✅ BLE 연결 성공')),
          );
        }
      }
    }
  }

  Future<void> _handleDisconnect() async {
    await bleSession.disconnect();
    setState(_copyFromSession);
    setModeLocal(ControlMode.off);
  }

  // ─────────── 모드 영속화 (프로필별) ───────────
  Future<void> _persistMode(ControlMode m) async {
    final key = _modeKeyForCurrentProfile();
    if (key == null) return;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(key, m.name);
  }

  Future<void> _restoreModeForProfile() async {
    final key = _modeKeyForCurrentProfile();
    final prefs = await SharedPreferences.getInstance();
    String? s;
    if (key != null) {
      s = prefs.getString(key);
    }
    // (하위 호환) 예전 디바이스 기반 키가 있으면 한번 읽어와줌
    if (s == null && _deviceSerial != null) {
      s = prefs.getString('mode:${_deviceSerial!}');
    }
    final restored = ControlMode.values.firstWhere(
          (e) => e.name == s,
      orElse: () => ControlMode.off,
    );
    setModeLocal(restored); // 발행 없이 UI만
  }

  // ─────────── 미등록/미준비 가드 ───────────
  Future<bool> _requireDeviceReadyAndRegistered() async {
    if (!_deviceStateReady) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('⏳ 기기 상태 확인 중입니다. 잠시만요.')),
      );
      return false;
    }
    if (_isDeviceRegistered) return true;

    final go = await showDialog<bool>(
      context: context,
      barrierDismissible: true,
      builder: (_) => AlertDialog(
        backgroundColor: headerGrey,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        title: const Text('기기 등록이 필요합니다',
            style: TextStyle(color: Colors.white, fontWeight: FontWeight.w700)),
        content: const Text('이 기능을 사용하려면 먼저 기기를 등록해주세요.',
            style: TextStyle(color: Colors.white70)),
        actions: [
          // ✅ 순서: 등록하기(왼쪽) → 취소(오른쪽)
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text('등록하기',
                style: TextStyle(color: primaryBlue, fontWeight: FontWeight.w700)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('취소', style: TextStyle(color: Colors.white70)),
          ),
        ],
      ),
    );

    if (go == true) {
      await _registerDevice();
      return _isDeviceRegistered;
    }
    return false;
  }

  // ─────────── 모드 제어 ───────────
  void _setMode(ControlMode newMode) {
    final prev = _mode;
    if (prev == newMode) return;
    setState(() => _mode = newMode);
    _persistMode(newMode); // 프로필별 저장
    _publishMode(prev, newMode);
    widget.onModeChange(newMode);
  }

  void _publishMode(ControlMode prev, ControlMode curr) {
    if (_profile == null || _deviceSerial == null) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 장치/프로필 없음: MQTT 미발행')),
      );
      return;
    }
    final topic = '/control_mode/$_deviceSerial';
    final body = {
      'profile_id': _profile!['id'].toString(),
      'previous_mode': prev.name,
      'current_mode': curr.name,
    };
    mqttService.publish(topic, body);
  }

  void _switchToOffMode() {
    if (_profile == null) return;
    _setMode(ControlMode.off);
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('🔴 OFF 모드로 전환되었습니다.')),
    );
  }

  Future<void> _switchToAiMode() async {
    if (_profile == null) return;
    final ok = await _requireDeviceReadyAndRegistered();
    if (!ok) return;
    _setMode(ControlMode.auto);
    widget.onAiModeSwitch();
  }

  Future<void> _handlePresetSelect(int presetId) async {
    if (_profile == null) return;
    final ok = await _requireDeviceReadyAndRegistered();
    if (!ok) return;

    final prev = _mode;
    final profileId = _profile!['id'];

    try {
      final token = await AuthService.getValidAccessToken();
      await http.post(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/preset-coordinate'),
        headers: {'Content-Type': 'application/json', 'Authorization': 'Bearer $token'},
        body: jsonEncode({"profile_id": profileId, "preset_id": presetId}),
      );
      _setMode(ControlMode.preset);
    } catch (_) {
      _publishMode(prev, prev);
    }
  }

  Future<void> _addPreset() async {
    if (_profile == null) return;
    if (_presets.length >= 3) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('❌ 프리셋은 최대 3개까지 가능합니다')),
      );
      return;
    }
    final profileId = _profile!['id'];
    final created =
    await PresetService.createPreset('프리셋 ${_presets.length + 1}', profileId, 1);
    if (created != null) await _loadProfileAndPresets();
  }

  // ─────────── MQTT 전체 초기화(모든 prev -> off 발행) ───────────
  Future<void> _resetMqttAllModes() async {
    final ok = await _requireDeviceReadyAndRegistered();
    if (!ok) return;

    // 모든 모드에 대해 prev -> off 전송
    for (final prev in ControlMode.values) {
      if (prev == ControlMode.off) continue;
      _publishMode(prev, ControlMode.off);
      await Future.delayed(const Duration(milliseconds: 120));
    }

    setModeLocal(ControlMode.off);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('🔁 MQTT 초기화: 모든 모드를 OFF로 재발행 완료')),
    );
  }

  Color _getModeColor() {
    switch (_mode) {
      case ControlMode.auto:
        return Colors.blueAccent;
      case ControlMode.preset:
        return Colors.purpleAccent;
      case ControlMode.manual:
        return Colors.orangeAccent;
      case ControlMode.fix:
        return Colors.grey;
      case ControlMode.off:
        return Colors.redAccent;
    }
  }

  String _formatDurationKr(int seconds) {
    if (seconds <= 0) return '0분';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return '${h}시간 ${m}분';
    return '${m}분';
  }

  Widget _sectionHeader({required String title, VoidCallback? onTap}) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Text(title,
            style: const TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.bold)),
        if (onTap != null)
          IconButton(
            onPressed: onTap,
            icon: const Icon(Icons.settings, color: Colors.white70),
            iconSize: _sectionIconSize,
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(),
            tooltip: title,
          ),
      ],
    );
  }

  // ─────────── UI ───────────
  @override
  Widget build(BuildContext context) {
    if (_profile == null) {
      return const Scaffold(
        backgroundColor: backgroundBlack,
        body: Center(child: CircularProgressIndicator(color: Colors.white)),
      );
    }

    final isConnected = _bleReady;

    return Scaffold(
      backgroundColor: backgroundBlack,
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.fromLTRB(16, 18, 16, 20),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  const SizedBox(width: 2),
                  CircleAvatar(
                    radius: 24,
                    backgroundColor: const Color(0xFF1F2937),
                    foregroundImage: _avatarProvider(_profile?['imageUrl']), // 없으면 null
                    child: const Icon(Icons.person, size: 24, color: Colors.white30),
                  ),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Transform.translate(
                      offset: const Offset(0, 4),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Text(
                            _profile!['name'] ?? '사용자',
                            style: const TextStyle(
                                fontSize: 18, fontWeight: FontWeight.bold, color: Colors.white, height: 1.15),
                          ),
                          const SizedBox(height: 2),
                          Text(
                            _profile!['email'] ?? '',
                            style: const TextStyle(color: Colors.white70, fontSize: 12, height: 1.1),
                          ),
                        ],
                      ),
                    ),
                  ),
                  IconButton(
                    icon: const Icon(Icons.settings, color: Colors.white),
                    onPressed: () async {
                      final changed =
                      await Navigator.push(context, MaterialPageRoute(builder: (_) => const EditProfilePage()));
                      if (changed == true) {
                        _loadProfileAndPresets();
                      }
                    },
                    padding: EdgeInsets.zero,
                    constraints: const BoxConstraints(),
                  ),
                ],
              ),

              const SizedBox(height: 12),
              _buildTopGrid(isConnected),
              const SizedBox(height: 14),
              _postureBanner(),
              const SizedBox(height: 12),

              _sectionHeader(title: '오늘 통계'),
              const SizedBox(height: 8),
              _buildTodayStatsCard(),

              const SizedBox(height: 18),
              _sectionHeader(
                title: '프리셋',
                onTap: () async {
                  final changed =
                  await Navigator.push(context, MaterialPageRoute(builder: (_) => const PresetPage()));
                  if (changed == true) await _loadProfileAndPresets();
                },
              ),
              const SizedBox(height: 10),
              _buildPresetArea(),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildTopGrid(bool isConnected) {
    final leftHeight = _rightCardHeight * 2 + _rightGap;
    return Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Expanded(flex: 2, child: SizedBox(height: leftHeight, child: _modeStatusCardCentered())),
        const SizedBox(width: 10),
        Expanded(
          flex: 1,
          child: Column(
            children: [
              SizedBox(height: _rightCardHeight, child: _powerToggleCard()),
              const SizedBox(height: _rightGap),
              SizedBox(
                height: _rightCardHeight,
                child: _deviceStateReady ? _bleCard(isConnected) : _bleCardSkeleton(),
              ),
            ],
          ),
        ),
      ],
    );
  }

  // 🔴 우상단 작은 초기화 버튼 포함
  Widget _modeStatusCardCentered() {
    return RectCard(
      bgColor: headerGrey,
      child: Stack(
        children: [
          // 초기화 버튼 (우상단)
          Positioned(
            top: 6,
            right: 6,
            child: SizedBox(
              width: 28,
              height: 28,
              child: IconButton(
                tooltip: 'MQTT 초기화 (전체 → OFF)',
                padding: EdgeInsets.zero,
                onPressed: _resetMqttAllModes,
                icon: const Icon(Icons.restart_alt, size: 16, color: Colors.redAccent),
              ),
            ),
          ),
          // 본문
          Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Text('모드 상태', style: TextStyle(color: Colors.white54, fontSize: 14)),
                const SizedBox(height: 6),
                Text(
                  _mode == ControlMode.off ? '전원 꺼짐' : _mode.name.toUpperCase(),
                  textAlign: TextAlign.center,
                  style: TextStyle(color: _getModeColor(), fontSize: 26, fontWeight: FontWeight.w800),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _powerToggleCard() {
    return RectCard(
      bgColor: headerGrey,
      child: Center(
        child: Switch.adaptive(
          value: _mode != ControlMode.off,
          onChanged: (on) => on ? _switchToAiMode() : _switchToOffMode(),
          activeColor: primaryBlue,
          activeTrackColor: primaryBlue.withOpacity(0.45),
          inactiveThumbColor: Colors.grey,
          inactiveTrackColor: Colors.grey.withOpacity(0.5),
        ),
      ),
    );
  }

  Widget _bleCard(bool isConnected) {
    const double iconSize = 22;
    return RectCard(
      bgColor: headerGrey,
      child: Center(
        child: _isDeviceRegistered
            ? (isConnected
            ? GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTap: () async {
            await _handleDisconnect();
            if (!mounted) return;
            ScaffoldMessenger.of(context).showSnackBar(
              const SnackBar(content: Text('🔌 BLE 연결을 해제했습니다.')),
            );
          },
          child: const Icon(Icons.bluetooth_connected, color: primaryBlue, size: iconSize),
        )
            : IconButton(
          tooltip: '스캔',
          onPressed: _requestPairAndScan,
          iconSize: iconSize,
          icon: const Icon(Icons.bluetooth_searching),
          color: Colors.white70,
        ))
            : IconButton(
          tooltip: '기기 등록',
          onPressed: _registerDevice,
          iconSize: iconSize,
          icon: const Icon(Icons.link),
          color: Colors.white70,
        ),
      ),
    );
  }

  Widget _bleCardSkeleton() {
    return RectCard(
      bgColor: headerGrey,
      child: const Center(
        child: SizedBox(
          width: 54,
          height: 12,
          child: DecoratedBox(decoration: BoxDecoration(color: Colors.white12)),
        ),
      ),
    );
  }

  Widget _postureBanner() {
    if (_loadingPosture) {
      return RectCard(
        bgColor: headerGrey,
        outlineColor: primaryBlue.withOpacity(0.45),
        child: const Row(
          children: [
            SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white)),
            SizedBox(width: 8),
            Expanded(child: Text('최근 자세 데이터를 불러오는 중...', style: TextStyle(color: Colors.white, fontWeight: FontWeight.w600))),
          ],
        ),
      );
    }

    if (_postureStatus == PostureBannerStatus.none) return const SizedBox.shrink();

    final isGood = _postureStatus == PostureBannerStatus.good;
    final title = isGood ? '올바른 자세입니다! 대단해요!' : '잘못된 자세입니다! 교정해주세요!';
    final icon = isGood ? Icons.check_circle : Icons.error_outline;

    return RectCard(
      bgColor: headerGrey,
      outlineColor: isGood ? primaryBlue : errorRed,
      child: Row(
        children: [
          Icon(icon, color: isGood ? primaryBlue : errorRed),
          const SizedBox(width: 8),
          Expanded(
            child: Text(title, style: const TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w700)),
          ),
          if (_postureTime != null)
            Text(DateFormat('HH:mm').format(_postureTime!), style: const TextStyle(color: Colors.white70, fontSize: 12)),
        ],
      ),
    );
  }

  Widget _miniTodayPie() {
    final good = _goodSecToday;
    final bad = _badSecToday;
    final total = good + bad;

    if (total == 0) {
      return const Center(child: Text('데이터 없음', style: TextStyle(color: Colors.white70, fontSize: 12)));
    }

    return PieChart(
      PieChartData(
        sectionsSpace: 2,
        centerSpaceRadius: 28,
        sections: [
          PieChartSectionData(value: good.toDouble(), color: primaryBlue, title: ''),
          PieChartSectionData(value: bad.toDouble(), color: errorRed, title: ''),
        ],
      ),
    );
  }

  Widget _buildTodayStatsCard() {
    final total = _goodSecToday + _badSecToday;

    return RectCard(
      bgColor: headerGrey,
      outlineColor: Colors.white.withOpacity(0.16),
      onTap: () => Navigator.push(context, MaterialPageRoute(builder: (_) => const StatsPage())),
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 4),
        child: Row(
          children: [
            Padding(padding: const EdgeInsets.only(left: 20), child: SizedBox(width: 110, height: 110, child: _miniTodayPie())),
            const SizedBox(width: 28),
            Expanded(
              child: total == 0
                  ? const Align(
                alignment: Alignment.centerRight,
                child: Text('오늘 데이터가 아직 없어요', style: TextStyle(color: Colors.white, fontSize: 13), textAlign: TextAlign.right),
              )
                  : Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.end,
                children: [
                  _legendLine(errorRed, '잘못된 자세', _formatDurationKr(_badSecToday)),
                  const SizedBox(height: 6),
                  _legendLine(primaryBlue, '올바른 자세', _formatDurationKr(_goodSecToday)),
                  const SizedBox(height: 8),
                  _rightInfoLine('총 시간', _formatDurationKr(total)),
                  const SizedBox(height: 2),
                  const Text('탭하면 자세한 통계로 이동', style: TextStyle(color: Colors.white, fontSize: 11), textAlign: TextAlign.right),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _legendLine(Color dotColor, String label, String value) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      children: [
        Container(width: 8, height: 8, decoration: BoxDecoration(color: dotColor, borderRadius: BorderRadius.circular(2))),
        const SizedBox(width: 6),
        Text('$label : $value', style: const TextStyle(color: Colors.white, fontSize: 13), textAlign: TextAlign.right, softWrap: false),
      ],
    );
  }

  Widget _rightInfoLine(String label, String value) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      children: [
        Text('$label: $value', style: const TextStyle(color: Colors.white, fontSize: 12), textAlign: TextAlign.right, softWrap: false),
      ],
    );
  }

  Widget _buildPresetArea() {
    final items = _presets.take(3).toList();

    if (items.isEmpty) return _addPresetCTA();

    return Row(
      children: [
        ...items.map((entry) {
          final name = entry['name'] ?? '이름 없음';
          final id = entry['id'] ?? 0;
          return Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _presetButton(name, id),
            ),
          );
        }),
        if (items.length < 3)
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _addPresetIconButton(),
            ),
          ),
      ],
    );
  }

  Widget _addPresetCTA() {
    return RectCard(
      bgColor: headerGrey,
      outlineColor: Colors.white.withOpacity(0.16),
      elevated: true,
      height: 56,
      onTap: _addPreset,
      child: const Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(Icons.add, size: 16, color: Colors.white70),
          SizedBox(width: 8),
          Text('프리셋 추가', style: TextStyle(color: Colors.white, fontSize: 14, fontWeight: FontWeight.w600)),
        ],
      ),
    );
  }

  Widget _addPresetIconButton() {
    return RectCard(
      bgColor: headerGrey,
      outlineColor: Colors.white.withOpacity(0.16),
      elevated: true,
      height: 56,
      onTap: _addPreset,
      child: const Center(child: Icon(Icons.add, size: 18, color: Colors.white70)),
    );
  }

  Widget _presetButton(String name, int presetId) {
    return RectCard(
      bgColor: headerGrey,
      outlineColor: Colors.white.withOpacity(0.16),
      elevated: true,
      height: 56,
      onTap: () => _handlePresetSelect(presetId),
      child: Center(
        child: Text(name, overflow: TextOverflow.ellipsis, style: const TextStyle(color: Colors.white, fontSize: 14, fontWeight: FontWeight.w600)),
      ),
    );
  }
}
