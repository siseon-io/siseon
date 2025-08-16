// lib/pages/home_screen.dart
import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:intl/intl.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';

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
  // THEME
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color headerGrey = Color(0xFF161B22);
  static const Color errorRed = Color(0xFFF87171);

  static const double _rightCardHeight = 64.0;
  static const double _rightGap = 8.0;
  static const double _sectionIconSize = 18;

  // 프로필 아바타 강제새로고침 버전
  int _avatarBust = 0;

  // 한번만 수행 가드
  bool _profileCheckScheduled = false;
  bool _deviceCheckScheduled = false;
  bool _skipNextProfileCacheCheck = false; // ✅ 추가
  // 아바타 provider
  Future<void> _reloadPresetsForCurrentProfile() async {
    if (_profile == null) return;
    final presets = await PresetService.fetchPresets(_profile!['id']);
    if (!mounted) return;
    setState(() {
      _presets = presets.take(3).toList();
      _avatarBust = DateTime.now().millisecondsSinceEpoch; // 네트워크 아바타 캐시 버스트
    });
  }

  ImageProvider? _avatarProvider(dynamic src) {
    final s = (src ?? '').toString().trim();
    if (s.isEmpty) return null;

    if (s.startsWith('http://') || s.startsWith('https://')) {
      final sep = s.contains('?') ? '&' : '?';
      final withBust = _avatarBust > 0 ? '$s${sep}v=$_avatarBust' : s;
      return NetworkImage(withBust);
    }
    if (s.startsWith('assets/')) {
      return AssetImage(s);
    }
    return null;
  }

  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];

  bool _isBluetoothOn = false;
  bool _isDeviceRegistered = false;
  bool _deviceStateReady = false;
  String? _deviceSerial;
  String? _targetCharUuid;

  // BLE session snapshot
  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

  late ControlMode _mode;

  int _goodSecToday = 0;
  int _badSecToday = 0;

  // 폴링
  Timer? _pollTimer;
  bool _isPolling = false;

  DateTime? _postureTime;
  bool _loadingPosture = false;
  PostureBannerStatus _postureStatus = PostureBannerStatus.none;

  // 최신 나쁜자세 라벨들
  List<String> _badLabels = [];

  // ✅ 현재 활성화(적용)된 프리셋 ID (모드가 preset일 때만 하이라이트)
  int? _activePresetId;

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
    WidgetsBinding.instance.addObserver(this);
    _mode = widget.currentMode;
    FlutterBluePlus.setLogLevel(LogLevel.none);
    _initPermissions();
    _checkBluetoothState();
    _syncProfileAndDevice();
    _loadLatestPosture();
    _startPolling();

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

  // 앱 라이프사이클
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.resumed) {
      _refreshSilently();
      _checkCachedProfileOnce();
      _checkCachedDeviceOnce();
      _startPolling();
    } else if (state == AppLifecycleState.paused ||
        state == AppLifecycleState.inactive ||
        state == AppLifecycleState.detached) {
      _stopPolling();
    }
  }

  void _startPolling() {
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(const Duration(seconds: 30), (_) async {
      if (!mounted) return;
      await _refreshSilently();
      _checkCachedProfileOnce();
      _checkCachedDeviceOnce();
    });
  }

  void _stopPolling() {
    _pollTimer?.cancel();
    _pollTimer = null;
  }

  Future<void> _refreshSilently() async {
    if (_isPolling) return;
    _isPolling = true;
    try {
      await _loadLatestPosture(silent: true);
      await _loadDailyStats();
      _checkCachedProfileOnce();
      _checkCachedDeviceOnce();
    } finally {
      _isPolling = false;
    }
  }

  // ───────────── 키 유틸 ─────────────
  String? _modeKeyForCurrentProfile() {
    final pid = _profile?['id'];
    if (pid == null) return null;
    return 'mode:profile:$pid';
  }

  String? _activePresetKeyForProfile() {
    final pid = _profile?['id'];
    if (pid == null) return null;
    return 'active_preset:profile:$pid';
  }

  Future<void> _persistActivePreset(int? id) async {
    final key = _activePresetKeyForProfile();
    if (key == null) return;
    final prefs = await SharedPreferences.getInstance();
    if (id == null) {
      await prefs.remove(key);
    } else {
      await prefs.setInt(key, id);
    }
  }

  Future<void> _restoreActivePreset() async {
    final key = _activePresetKeyForProfile();
    if (key == null) return;
    final prefs = await SharedPreferences.getInstance();
    if (!mounted) return;
    setState(() => _activePresetId = prefs.getInt(key));
  }

  // 외부에서 모드 동기화 (발행 없음)
  void setModeLocal(ControlMode newMode) {
    if (_mode == newMode) return;
    setState(() {
      _mode = newMode;
      // preset 모드가 아니면 하이라이트 해제
      if (newMode != ControlMode.preset) {
        _activePresetId = null;
      }
    });
    _persistMode(newMode);
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

    setState(() {
      _profile = profile;
      _deviceStateReady = false;
      _isDeviceRegistered = false;
      _deviceSerial = null;
      _targetCharUuid = null;
      _mode = ControlMode.off;
      _activePresetId = null;
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
        _activePresetId = null;
      });
      setModeLocal(ControlMode.off);
      return;
    }

    final device = await DeviceCacheService.loadDeviceForProfile(profileId);
    if (!mounted) return;
    setState(() {
      _isDeviceRegistered = device != null;
      _deviceSerial = device?['serial'];
      _targetCharUuid = device?['targetCharUuid'] ??
          device?['charUuid'] ??
          device?['characteristicUuid'];
      _deviceStateReady = true;
      if (!_isDeviceRegistered) _activePresetId = null;
    });

    if (!_isDeviceRegistered) {
      setModeLocal(ControlMode.off);
    } else {
      await _restoreModeForProfile();
      await _restoreActivePreset(); // ✅ 프로필별 활성 프리셋 복원
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
      _avatarBust = DateTime.now().millisecondsSinceEpoch;
    });
    // 프로필 바뀌었을 수도 있으니 활성 프리셋 복원 시도
    await _restoreActivePreset();
  }

  // 홈이 보일 때 로컬 캐시 프로필 변경 반영
  void _checkCachedProfileOnce() {
    if (_skipNextProfileCacheCheck) { // ✅ 방금 수정한 값을 캐시가 되돌리지 않게 1회 차단
      _skipNextProfileCacheCheck = false;
      return;
    }
    if (_profileCheckScheduled) return;
    _profileCheckScheduled = true;
    SchedulerBinding.instance.addPostFrameCallback((_) async {
      _profileCheckScheduled = false;
      final cached = await ProfileCacheService.loadProfile();
      if (!mounted || cached == null) return;

      final newUrl = (cached['imageUrl'] ?? '').toString();
      final oldUrl = (_profile?['imageUrl'] ?? '').toString();
      final changed = newUrl != oldUrl || (cached['name'] != _profile?['name']);

      if (changed) {
        setState(() {
          _profile = cached;
          _avatarBust = DateTime.now().millisecondsSinceEpoch;
        });
      }
    });
  }

  // 로컬 캐시의 디바이스 등록상태 변경 반영
  void _checkCachedDeviceOnce() {
    if (_deviceCheckScheduled) return;
    _deviceCheckScheduled = true;
    SchedulerBinding.instance.addPostFrameCallback((_) async {
      _deviceCheckScheduled = false;
      final pid = _profile?['id'] as int?;
      if (pid == null || !mounted) return;

      final cached = await DeviceCacheService.loadDeviceForProfile(pid);
      final registeredNow = cached != null;
      final serialNow = cached?['serial'];
      final charNow = cached?['targetCharUuid'] ??
          cached?['charUuid'] ??
          cached?['characteristicUuid'];

      final changed = (registeredNow != _isDeviceRegistered) ||
          (serialNow != _deviceSerial) ||
          (charNow != _targetCharUuid);

      if (changed) {
        setState(() {
          _isDeviceRegistered = registeredNow;
          _deviceSerial = serialNow;
          _targetCharUuid = charNow;
          _deviceStateReady = true;
        });
        if (!registeredNow) {
          setModeLocal(ControlMode.off);
          _activePresetId = null;
          _persistActivePreset(null);
        }
      }
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
          _badLabels = [];
          _loadingPosture = false;
        });
        return;
      }

      final mins = await StatsService.fetchMinuteStats(
        profileId: profileId,
        period: 'daily',
      );

      if (mins.isEmpty) {
        setState(() {
          _postureStatus = PostureBannerStatus.none;
          _postureTime = null;
          _badLabels = [];
          _loadingPosture = false;
        });
        return;
      }

      mins.sort((a, b) => a.endAt.compareTo(b.endAt));
      final latest = mins.last;

      bool? isValid = latest.validPosture;
      try { final v2 = (latest as dynamic).valid; if (v2 is bool) isValid = v2; } catch (_) {}
      try {
        final br = (latest as dynamic).badReasons;
        final v3 = (br as dynamic).valid;
        if (v3 is bool) isValid = v3;
      } catch (_) {}

      if (isValid == false) {
        final labels = _extractBadLabels(latest);
        setState(() {
          _postureStatus = PostureBannerStatus.bad;
          _postureTime   = latest.endAt.toLocal();
          _badLabels     = labels;
          _loadingPosture = false;
        });
      } else if (isValid == true) {
        setState(() {
          _postureStatus = PostureBannerStatus.good;
          _postureTime   = latest.endAt.toLocal();
          _badLabels     = [];
          _loadingPosture = false;
        });
      } else {
        setState(() {
          _postureStatus = PostureBannerStatus.none;
          _postureTime = null;
          _badLabels = [];
          _loadingPosture = false;
        });
      }
    } catch (_) {
      if (!mounted) return;
      setState(() {
        _postureStatus = PostureBannerStatus.none;
        _postureTime = null;
        _badLabels = [];
        _loadingPosture = false;
      });
    }
  }

  bool? _valid(PostureStats s) {
    try {
      final v = (s as dynamic).validPosture;
      if (v is bool) return v;
    } catch (_) {}
    try {
      final v2 = (s as dynamic).valid;
      if (v2 is bool) return v2;
    } catch (_) {}
    try {
      final br = (s as dynamic).badReasons;
      final v3 = (br as dynamic).valid;
      if (v3 is bool) return v3;
    } catch (_) {}
    return null;
  }

  // Mojibake 복구
  String _fixKoreanIfGarbled(String s) {
    final looksGarbled = RegExp(r'(Ã.|Â.|ì.|í.|ë.|ê.|°|±|²|³|¼|½|¾)').hasMatch(s) &&
        !RegExp(r'[가-힣]').hasMatch(s);
    if (!looksGarbled) return s;
    try {
      final repaired = utf8.decode(latin1.encode(s));
      if (RegExp(r'[가-힣]').hasMatch(repaired)) return repaired;
      return s;
    } catch (_) {
      return s;
    }
  }

  String _cleanLabel(String input) {
    final fixed = _fixKoreanIfGarbled(input);
    return fixed.replaceAll(RegExp(r'\s+'), ' ').trim();
  }

  List<String> _extractBadLabels(dynamic item) {
    final labels = <String>[];

    void _collectFromSummary(String? sum) {
      if (sum == null || sum.trim().isEmpty) return;
      final fixed = _fixKoreanIfGarbled(sum).trim();
      for (final part in fixed.split(RegExp(r'\s*,\s*'))) {
        if (part.isEmpty) continue;
        final nameOnly = part.split('(').first.trim();
        if (nameOnly.isNotEmpty) {
          labels.add(nameOnly);
        }
      }
    }

    try {
      final br = (item as dynamic).badReasons;
      if (br != null) {
        final sum = (br as dynamic).summary?.toString();
        _collectFromSummary(sum);
      }
    } catch (_) {}

    if (labels.isEmpty) {
      try {
        final sum2 = (item as dynamic).summary?.toString();
        _collectFromSummary(sum2);
      } catch (_) {}
    }

    if (labels.isEmpty) {
      try {
        final br = (item as dynamic).badReasons;
        final rs = (br as dynamic).reasons;
        if (rs is Iterable) {
          for (final r in rs) {
            final lbl = (r as dynamic).label;
            if (lbl is String && lbl.trim().isNotEmpty) {
              labels.add(_cleanLabel(lbl));
            }
          }
        }
      } catch (_) {}
    }

    return labels.toSet().toList();
  }

  // ─────────── 등록/스캔/연결 ───────────
  Future<void> _registerDevice() async {
    final result = await Navigator.push(
      context,
      MaterialPageRoute(builder: (_) => const DeviceRegisterPage()),
    );

    final pid = _profile?['id'] as int?;
    await _loadDeviceStateFor(pid);

    if (!mounted) return;
    // 스낵바 제거: 성공/실패 메시지 표시 없음
    // if (_isDeviceRegistered && (result == true)) { ... }
  }

  Future<void> _requestPairAndScan() async {
    if (_profile == null) {
      // 스낵바 제거
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
      // 스낵바 제거
      return;
    }

    if (result != null) {
      setState(() {
        _connectedDevice = result['device'] as BluetoothDevice?;
        _writableChar = result['writableChar'] as BluetoothCharacteristic?;
      });
      if (_writableChar != null) {
        widget.onConnect?.call(_writableChar!);
        // 스낵바 제거
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
    if (s == null && _deviceSerial != null) {
      s = prefs.getString('mode:${_deviceSerial!}');
    }
    final restored = ControlMode.values.firstWhere(
          (e) => e.name == s,
      orElse: () => ControlMode.off,
    );
    setModeLocal(restored);
  }

  // ─────────── 미등록/미준비 가드 ───────────
  Future<bool> _requireDeviceReadyAndRegistered() async {
    if (!_deviceStateReady) {
      // 스낵바 제거
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
    _persistMode(newMode);
    _publishMode(prev, newMode);
    widget.onModeChange(newMode);
    if (newMode != ControlMode.preset) {
      _activePresetId = null;
      _persistActivePreset(null);
    }
  }

  void _publishMode(ControlMode prev, ControlMode curr) {
    if (_profile == null || _deviceSerial == null) {
      // 스낵바 제거
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
    // 스낵바 제거
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
      final res = await http.post(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/preset-coordinate'),
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token'
        },
        body: jsonEncode({"profile_id": profileId, "preset_id": presetId}),
      );

      if (res.statusCode >= 200 && res.statusCode < 300) {
        _setMode(ControlMode.preset);
        setState(() => _activePresetId = presetId); // ✅ 하이라이트
        _persistActivePreset(presetId);
      } else {
        _publishMode(prev, prev);
        // 스낵바 제거
      }
    } catch (_) {
      _publishMode(prev, prev);
      // 스낵바 제거
    }
  }

  Future<void> _addPreset() async {
    if (_profile == null) return;
    if (_presets.length >= 3) {
      // 스낵바 제거
      return;
    }

    final profileId = _profile!['id'];
    final name = '프리셋 ${_presets.length + 1}';

    try {
      final created = await PresetService.createPreset(name, profileId, 1);
      if (created != null) {
        await _loadProfileAndPresets();
        // 스낵바 제거
      }
    } on PresetSaveException catch (e) {
      final msg = (e.code == 'no_raw_posture')
          ? '약 10초 후 다시 시도해주세요.'
          : e.message;
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
    } catch (_) {
      // 스낵바 제거
    }
  }
  Future<void> refreshFromRoot() async {
    // 프리셋/프로필 최신화
    await _loadProfileAndPresets();
    // 통계/배너 등도 같이 갱신
    await _refreshSilently();
  }
  // 전체 OFF 재발행
  Future<void> _resetMqttAllModes() async {
    final ok = await _requireDeviceReadyAndRegistered();
    if (!ok) return;

    for (final prev in ControlMode.values) {
      if (prev == ControlMode.off) continue;
      _publishMode(prev, ControlMode.off);
      await Future.delayed(const Duration(milliseconds: 120));
    }

    setModeLocal(ControlMode.off);
    if (!mounted) return;
    // 스낵바 제거
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
            style: const TextStyle(
                color: Colors.white, fontSize: 16, fontWeight: FontWeight.bold)),
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
    _checkCachedProfileOnce();
    _checkCachedDeviceOnce();

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
        child: RefreshIndicator(
          color: Colors.white,
          backgroundColor: primaryBlue,
          onRefresh: () async {
            _checkCachedProfileOnce();
            _checkCachedDeviceOnce();
            await _loadProfileAndPresets();
            final pid = _profile?['id'] as int?;
            await _loadDeviceStateFor(pid);
            await _refreshSilently();
          },
          child: LayoutBuilder(
            builder: (context, constraints) {
              return SingleChildScrollView(
                physics: const AlwaysScrollableScrollPhysics(),
                padding: const EdgeInsets.fromLTRB(16, 18, 16, 20),
                child: ConstrainedBox(
                  constraints: BoxConstraints(minHeight: constraints.maxHeight),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      // 헤더
                      Row(
                        crossAxisAlignment: CrossAxisAlignment.center,
                        children: [
                          const SizedBox(width: 2),
                          CircleAvatar(
                            radius: 24,
                            backgroundColor: const Color(0xFF1F2937),
                            foregroundImage: _avatarProvider(_profile?['imageUrl']),
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
                                        fontSize: 18, fontWeight: FontWeight.bold,
                                        color: Colors.white, height: 1.15),
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
                              final result = await Navigator.push<Map<String, dynamic>?>(
                                context,
                                MaterialPageRoute(builder: (_) => const EditProfilePage()),
                              );

                              if (!mounted) return;

                              if (result != null) {
                                // 1) 수정 결과 즉시 UI 반영
                                setState(() {
                                  _profile = result;
                                  _avatarBust = DateTime.now().millisecondsSinceEpoch; // 아바타 버스트
                                });

                                // 2) 프리셋만 최신화 (프로필은 다시 캐시에서 읽지 않음)
                                await _reloadPresetsForCurrentProfile();

                                // 3) 캐시 동기화 체크가 방금 값 덮어쓰지 않도록 1회 스킵
                                _skipNextProfileCacheCheck = true;
                              } else {
                                // 결과를 안 주는 페이지라면, 기존 경로로(캐시 의존)
                                await _loadProfileAndPresets();
                              }

                              await _refreshSilently();
                            },
                            padding: EdgeInsets.zero,
                            constraints: const BoxConstraints(),
                          ),
                        ],
                      ),

                      const SizedBox(height: 12),
                      _buildTopGrid(_bleReady),
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
                          await Navigator.push(
                            context,
                            MaterialPageRoute(builder: (_) => const PresetPage()),
                          );
                          await _loadProfileAndPresets();
                          if (!mounted) return;
                          await _refreshSilently();
                        },
                      ),
                      const SizedBox(height: 10),
                      _buildPresetArea(),
                    ],
                  ),
                ),
              );
            },
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
        Expanded(
          flex: 2,
          child: SizedBox(height: leftHeight, child: _modeStatusCardCentered()),
        ),
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
  Future<void> _showResetConfirmDialog() async {
    final yes = await showDialog<bool>(
      context: context,
      barrierDismissible: true,
      builder: (_) => AlertDialog(
        backgroundColor: headerGrey,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        title: const Text(
          '모드 초기화',
          style: TextStyle(color: Colors.white, fontWeight: FontWeight.w700),
        ),
        content: const Text(
          '초기화 시 OFF 모드로 전환됩니다.',
          style: TextStyle(color: Colors.white70),
        ),
        actions: [
          // ✅ 왼쪽: 네
          TextButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text(
              '네',
              style: TextStyle(color: primaryBlue, fontWeight: FontWeight.w700),
            ),
          ),
          // ✅ 오른쪽: 아니요
          TextButton(
            onPressed: () => Navigator.pop(context, false),
            child: const Text('아니요', style: TextStyle(color: Colors.white70)),
          ),
        ],
      ),
    );

    if (yes == true) {
      await _resetMqttAllModes(); // 실제 초기화 실행 (OFF로 전환)
    }
  }
  // 🔴 우상단 작은 초기화 버튼 포함
  Widget _modeStatusCardCentered() {
    return RectCard(
      bgColor: headerGrey,
      child: Stack(
        children: [
          Positioned(
            top: 6,
            right: 6,
            child: SizedBox(
              width: 28,
              height: 28,
              child: IconButton(
                tooltip: 'MQTT 초기화 (전체 → OFF)',
                padding: EdgeInsets.zero,
                onPressed: _showResetConfirmDialog, // ⬅️ 바로 초기화 → 확인 다이얼로그
                icon: const Icon(Icons.restart_alt, size: 16, color: Colors.redAccent),
              ),
            ),
          ),
          Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Text('모드 상태',
                    style: TextStyle(color: Colors.white54, fontSize: 14)),
                const SizedBox(height: 6),
                Text(
                  _mode == ControlMode.off ? '전원 꺼짐' : _mode.name.toUpperCase(),
                  textAlign: TextAlign.center,
                  style: TextStyle(
                    color: _getModeColor(),
                    fontSize: 26,
                    fontWeight: FontWeight.w800,
                  ),
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
            // 스낵바 제거
          },
          child: const Icon(Icons.bluetooth_connected,
              color: primaryBlue, size: iconSize),
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
          child: DecoratedBox(
            decoration: BoxDecoration(color: Colors.white12),
          ),
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
            SizedBox(
              width: 18,
              height: 18,
              child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white),
            ),
            SizedBox(width: 8),
            Expanded(
              child: Text(
                '최근 자세 데이터를 불러오는 중...',
                style: TextStyle(color: Colors.white, fontWeight: FontWeight.w600),
              ),
            ),
          ],
        ),
      );
    }

    if (_postureStatus == PostureBannerStatus.none) {
      return const SizedBox.shrink();
    }

    final isGood = _postureStatus == PostureBannerStatus.good;
    final isBad = _postureStatus == PostureBannerStatus.bad;
    final hasLabels = isBad && _badLabels.isNotEmpty;

    final title = hasLabels
        ? '${_badLabels.join(', ')}이(가) 감지됩니다.'
        : (isGood ? '올바른 자세입니다. 유지해주세요!' : '잘못된 자세입니다. 교정해주세요!');
    final sub = hasLabels ? '탭하면 자세한 통계로 이동합니다.' : '탭하면 자세한 통계로 이동';
    final icon = isGood ? Icons.check_circle : Icons.error_outline;

    return RectCard(
      bgColor: headerGrey,
      outlineColor: isGood ? primaryBlue : errorRed,
      onTap: () => Navigator.push(
        context,
        MaterialPageRoute(builder: (_) => const StatsPage()),
      ),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Icon(icon, color: isGood ? primaryBlue : errorRed),
          const SizedBox(width: 8),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  softWrap: true,
                  overflow: TextOverflow.visible,
                  style: const TextStyle(
                      color: Colors.white, fontSize: 16, fontWeight: FontWeight.w700),
                ),
                const SizedBox(height: 4),
                Text(sub, style: const TextStyle(color: Colors.white70, fontSize: 12)),
              ],
            ),
          ),
          if (_postureTime != null)
            Padding(
              padding: const EdgeInsets.only(right: 4.0),
              child: Text(
                DateFormat('HH:mm').format(_postureTime!),
                style: const TextStyle(color: Colors.white70, fontSize: 12),
              ),
            ),
        ],
      ),
    );
  }

  Widget _miniTodayPie() {
    final good = _goodSecToday;
    final bad = _badSecToday;
    final total = good + bad;

    if (total == 0) {
      return const Center(
          child: Text('데이터 없음',
              style: TextStyle(color: Colors.white70, fontSize: 12)));
    }

    return PieChart(
      PieChartData(
        sectionsSpace: 2,
        centerSpaceRadius: 28,
        sections: [
          PieChartSectionData(
              value: good.toDouble(), color: primaryBlue, title: ''),
          PieChartSectionData(
              value: bad.toDouble(), color: errorRed, title: ''),
        ],
      ),
    );
  }

  Widget _buildTodayStatsCard() {
    final total = _goodSecToday + _badSecToday;

    return RectCard(
      bgColor: headerGrey,
      outlineColor: Colors.white.withOpacity(0.16),
      onTap: () => Navigator.push(
          context, MaterialPageRoute(builder: (_) => const StatsPage())),
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 4),
        child: Row(
          children: [
            Padding(
              padding: const EdgeInsets.only(left: 20),
              child: SizedBox(width: 110, height: 110, child: _miniTodayPie()),
            ),
            const SizedBox(width: 28),
            Expanded(
              child: total == 0
                  ? const Align(
                alignment: Alignment.centerRight,
                child: Text('오늘 데이터가 아직 없어요',
                    style: TextStyle(color: Colors.white, fontSize: 13),
                    textAlign: TextAlign.right),
              )
                  : Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.end,
                children: [
                  _legendLine(errorRed, '잘못된 자세',
                      _formatDurationKr(_badSecToday)),
                  const SizedBox(height: 6),
                  _legendLine(primaryBlue, '올바른 자세',
                      _formatDurationKr(_goodSecToday)),
                  const SizedBox(height: 8),
                  _rightInfoLine('총 시간', _formatDurationKr(total)),
                  const SizedBox(height: 2),
                  const Text('탭하면 자세한 통계로 이동',
                      style: TextStyle(color: Colors.white, fontSize: 11),
                      textAlign: TextAlign.right),
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
        Container(
            width: 8,
            height: 8,
            decoration: BoxDecoration(
                color: dotColor, borderRadius: BorderRadius.circular(2))),
        const SizedBox(width: 6),
        Text('$label : $value',
            style: const TextStyle(color: Colors.white, fontSize: 13),
            textAlign: TextAlign.right,
            softWrap: false),
      ],
    );
  }

  Widget _rightInfoLine(String label, String value) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      children: [
        Text('$label: $value',
            style: const TextStyle(color: Colors.white, fontSize: 12),
            textAlign: TextAlign.right,
            softWrap: false),
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
          Text('프리셋 추가',
              style: TextStyle(
                  color: Colors.white,
                  fontSize: 14,
                  fontWeight: FontWeight.w600)),
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
      child:
      const Center(child: Icon(Icons.add, size: 18, color: Colors.white70)),
    );
  }

  // ✅ 선택된 프리셋은 파란 테두리만 (체크 아이콘 제거)
  Widget _presetButton(String name, int presetId) {
    final bool selected =
        (_mode == ControlMode.preset) && (_activePresetId == presetId);

    return RectCard(
      bgColor: headerGrey,
      outlineColor: selected ? primaryBlue : Colors.white.withOpacity(0.16),
      elevated: true,
      height: 56,
      onTap: () => _handlePresetSelect(presetId),
      child: Center(
        child: Text(
          name,
          overflow: TextOverflow.ellipsis,
          style: const TextStyle(
            color: Colors.white,
            fontSize: 14,
            fontWeight: FontWeight.w600,
          ),
        ),
      ),
    );
  }
}
