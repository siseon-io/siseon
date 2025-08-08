import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:http/http.dart' as http;

import 'package:siseon2/models/control_mode.dart';
import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/mqtt_service.dart';
import 'package:siseon2/services/preset_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/services/device_cache_service.dart';
import 'package:siseon2/services/stats_service.dart';

import 'package:siseon2/pages/ble_scan_screen.dart';
import 'package:siseon2/pages/device_register_page.dart';
import 'package:siseon2/pages/settings/preset_page.dart';

// 🔹 enum은 파일 최상위에 둬야 함
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

class HomeScreenState extends State<HomeScreen> {
  // THEME
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color headerGrey = Color(0xFF161B22);
  static const Color cardGrey = Color(0xFF1E293B);
  static const Color errorRed = Color(0xFFF87171);

  // Top grid sizes
  static const double _rightCardHeight = 64.0;
  static const double _rightGap = 8.0;

  // Today stats sizes
  static const double kTodayCardHeight = 140;
  static const double kDonutSize = 50;        // 도넛 지름
  static const double kSliceThickness = 4;    // 링 두께
  static const double kDonutLeftOffset = 40;  // 왼쪽 여백

  Map<String, dynamic>? _profile;
  List<Map<String, dynamic>> _presets = [];

  bool _isBluetoothOn = false;
  bool _isDeviceRegistered = false;
  bool _deviceStateReady = false;
  String? _deviceSerial;

  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

  late ControlMode _mode;

  // daily stats
  int _goodSecToday = 0;
  int _badSecToday = 0;

  // 최근 자세 배너 상태
  Timer? _postureTimer;
  DateTime? _postureTime; // endAt 기준
  bool _loadingPosture = false;
  PostureBannerStatus _postureStatus = PostureBannerStatus.none;

  @override
  void initState() {
    super.initState();
    _mode = widget.currentMode;
    FlutterBluePlus.setLogLevel(LogLevel.none);
    _initPermissions();
    _checkBluetoothState();
    _syncProfileAndDevice();
    _loadLatestPosture(); // 첫 진입 로드
    _postureTimer =
        Timer.periodic(const Duration(minutes: 5), (_) => _loadLatestPosture());
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
    _postureTimer?.cancel();
    super.dispose();
  }

  void setModeExternal(ControlMode newMode) => _setMode(newMode);

  // PERMS / BLE
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

  // LOAD
  Future<void> _syncProfileAndDevice() async {
    final profile = await ProfileCacheService.loadProfile();
    final pid = profile?['id'] as int?;
    setState(() {
      _profile = profile;
      _deviceStateReady = false;
    });
    await _loadProfileAndPresets();
    await _loadDailyStats();
    await _loadDeviceStateFor(pid);
  }

  Future<void> _loadDeviceStateFor(int? profileId) async {
    if (profileId == null) {
      if (!mounted) return;
      setState(() {
        _isDeviceRegistered = false;
        _deviceSerial = null;
        _deviceStateReady = true;
      });
      return;
    }
    final device = await DeviceCacheService.loadDeviceForProfile(profileId);
    if (!mounted) return;
    setState(() {
      _isDeviceRegistered = device != null;
      _deviceSerial = device?['serial'];
      _deviceStateReady = true;
    });
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
        // ✅ 서버의 validPosture만 사용
        final v = _valid(s);
        if (v == null) continue; // 값 없으면 스킵 (원하면 false로 처리 가능)
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

  // 최근 자세 1건 조회 (배치 10분 주기 → 5분 폴링)
  Future<void> _loadLatestPosture() async {
    try {
      setState(() => _loadingPosture = true);

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

      // 최신(endAt 기준)으로 정렬 후 가장 마지막 사용
      list.sort((a, b) => a.endAt.compareTo(b.endAt));
      final latest = list.last;

      // ✅ 서버 validPosture만 사용
      final v = _valid(latest);

      setState(() {
        if (v == null) {
          _postureStatus = PostureBannerStatus.none; // 값 없으면 배너 숨김
        } else {
          _postureStatus =
          v ? PostureBannerStatus.good : PostureBannerStatus.bad;
        }
        _postureTime = latest.endAt.toLocal();
        _loadingPosture = false;
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _postureStatus = PostureBannerStatus.none;
        _postureTime = null;
        _loadingPosture = false;
      });
    }
  }

  // ✅ 서버값만 사용 (없으면 null)
  bool? _valid(PostureStats s) {
    try {
      final v = (s as dynamic).validPosture;
      if (v is bool) return v;
    } catch (_) {}
    return null;
  }

  // DEVICE ACTIONS
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
      MaterialPageRoute(builder: (_) => const BleScanScreen()),
    );
    if (result == null) return;

    setState(() {
      _connectedDevice = result['device'] as BluetoothDevice;
      _writableChar = result['writableChar'] as BluetoothCharacteristic;
    });

    widget.onConnect?.call(_writableChar!);

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('✅ BLE 연결 성공')),
    );
  }

  Future<void> _handleDisconnect() async {
    await _connectedDevice?.disconnect();
    setState(() {
      _connectedDevice = null;
      _writableChar = null;
    });
  }

  // MODE / MQTT
  void _setMode(ControlMode newMode) {
    final prev = _mode;
    if (prev == newMode) return;
    setState(() => _mode = newMode);
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

  void _switchToAiMode() {
    if (_profile == null) return;
    _setMode(ControlMode.auto);
    widget.onAiModeSwitch();
  }

  // PRESET
  Future<void> _handlePresetSelect(int presetId) async {
    if (_profile == null) return;
    final prev = _mode;
    final profileId = _profile!['id'];

    try {
      final token = await AuthService.getValidAccessToken();
      await http.post(
        Uri.parse('http://i13b101.p.ssafy.io:8080/api/preset-coordinate'),
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
        body: jsonEncode({"profile_id": profileId, "preset_id": presetId}),
      );
      _setMode(ControlMode.preset);
    } catch (e) {
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
    final created = await PresetService.createPreset(
      '프리셋 ${_presets.length + 1}',
      profileId,
      1,
    );
    if (created != null) await _loadProfileAndPresets();
  }

  // HELPERS
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

  // UI
  @override
  Widget build(BuildContext context) {
    if (_profile == null) {
      return const Scaffold(
        backgroundColor: backgroundBlack,
        body: Center(child: CircularProgressIndicator(color: Colors.white)),
      );
    }

    final isConnected = _connectedDevice != null;

    return Scaffold(
      backgroundColor: backgroundBlack,
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.fromLTRB(16, 12, 16, 20),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              // 프로필 헤더
              Row(
                children: [
                  CircleAvatar(
                    radius: 24,
                    backgroundImage: (() {
                      final imageUrl = _profile?['imageUrl'];
                      if (imageUrl != null) {
                        if (imageUrl.toString().startsWith('http')) {
                          return NetworkImage(imageUrl);
                        } else if (imageUrl.toString().startsWith('assets/')) {
                          return AssetImage(imageUrl) as ImageProvider;
                        }
                      }
                      return const AssetImage('assets/images/profile_cat.png');
                    })(),
                  ),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(_profile!['name'] ?? '사용자',
                            style: const TextStyle(
                                fontSize: 18,
                                fontWeight: FontWeight.bold,
                                color: Colors.white)),
                        Text(_profile!['email'] ?? '',
                            style: const TextStyle(
                                color: Colors.white70, fontSize: 12)),
                      ],
                    ),
                  ),
                  IconButton(
                    icon: const Icon(Icons.settings, color: Colors.white),
                    onPressed: widget.onGoToProfile,
                  ),
                ],
              ),

              const SizedBox(height: 12),

              // 상단 3개 블록
              _buildTopGrid(isConnected),

              const SizedBox(height: 14),

              // 최근 자세 배너 (통계 카드 위)
              _postureBanner(),
              const SizedBox(height: 12),

              // 오늘 통계
              _buildTodayStatsCard(),

              const SizedBox(height: 18),

              // 프리셋
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  const Text('프리셋',
                      style: TextStyle(
                          color: Colors.white,
                          fontSize: 16,
                          fontWeight: FontWeight.bold)),
                  IconButton(
                    icon: const Icon(Icons.settings, color: Colors.white70),
                    onPressed: () async {
                      final changed = await Navigator.push(
                        context,
                        MaterialPageRoute(builder: (_) => const PresetPage()),
                      );
                      if (changed == true) await _loadProfileAndPresets();
                    },
                  ),
                ],
              ),
              const SizedBox(height: 10),
              _buildPresetArea(),
            ],
          ),
        ),
      ),
    );
  }

  // Top Grid
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
                child:
                _deviceStateReady ? _bleCard(isConnected) : _bleCardSkeleton(),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _modeStatusCardCentered() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration:
      BoxDecoration(color: headerGrey, borderRadius: BorderRadius.circular(14)),
      child: Center(
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
    );
  }

  Widget _powerToggleCard() {
    return Container(
      decoration:
      BoxDecoration(color: headerGrey, borderRadius: BorderRadius.circular(14)),
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
    return Container(
      decoration:
      BoxDecoration(color: headerGrey, borderRadius: BorderRadius.circular(14)),
      child: Center(
        child: _isDeviceRegistered
            ? (!isConnected
            ? IconButton(
          tooltip: '스캔',
          onPressed: _requestPairAndScan,
          icon: const Icon(Icons.bluetooth_searching),
          color: Colors.white,
        )
            : Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            const Icon(Icons.bluetooth_connected,
                color: Colors.white70, size: 20),
            const SizedBox(width: 6),
            TextButton(
              onPressed: _handleDisconnect,
              child: const Text('해제',
                  style: TextStyle(
                      color: Colors.white70, fontSize: 12)),
            ),
          ],
        ))
            : IconButton(
          tooltip: '기기 등록',
          onPressed: _registerDevice,
          icon: const Icon(Icons.link),
          color: Colors.white,
        ),
      ),
    );
  }

  Widget _bleCardSkeleton() {
    return Container(
      decoration:
      BoxDecoration(color: headerGrey, borderRadius: BorderRadius.circular(14)),
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

  // 최근 자세 배너 UI
  Widget _postureBanner() {
    if (_loadingPosture) {
      return Container(
        padding: const EdgeInsets.all(12),
        decoration: BoxDecoration(
          color: headerGrey,
          borderRadius: BorderRadius.circular(14),
          border: Border.all(color: primaryBlue.withOpacity(0.45)),
        ),
        child: const Row(
          children: [
            SizedBox(
              width: 18,
              height: 18,
              child: CircularProgressIndicator(
                  strokeWidth: 2, color: Colors.white),
            ),
            SizedBox(width: 8),
            Expanded(
              child: Text(
                '최근 자세 데이터를 불러오는 중...',
                style:
                TextStyle(color: Colors.white, fontWeight: FontWeight.w600),
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
    final title =
    isGood ? '올바른 자세입니다! 대단해요!' : '잘못된 자세입니다! 교정해주세요!';
    final icon = isGood ? Icons.check_circle : Icons.error_outline;

    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: headerGrey,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: isGood ? primaryBlue : errorRed),
      ),
      child: Row(
        children: [
          Icon(icon, color: isGood ? primaryBlue : errorRed),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              title,
              style: const TextStyle(
                  color: Colors.white, fontSize: 16, fontWeight: FontWeight.w700),
            ),
          ),
          if (_postureTime != null)
            Text(
              DateFormat('HH:mm').format(_postureTime!),
              style: const TextStyle(color: Colors.white70, fontSize: 12),
            ),
        ],
      ),
    );
  }

  // Today stats
  Widget _buildTodayStatsCard() {
    final total = _goodSecToday + _badSecToday;

    return Container(
      height: kTodayCardHeight,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: cardGrey,
        borderRadius: BorderRadius.circular(14),
      ),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          SizedBox(width: kDonutLeftOffset),

          // 도넛(작게, 고정 크기)
          SizedBox(
            width: kDonutSize,
            height: kDonutSize,
            child: total == 0
                ? const SizedBox.shrink()
                : PieChart(
              PieChartData(
                startDegreeOffset: -90,
                sectionsSpace: 1,
                centerSpaceRadius: kDonutSize / 2 - kSliceThickness,
                sections: [
                  PieChartSectionData(
                    value: _badSecToday.toDouble(), // 잘못된 자세
                    color: errorRed,
                    showTitle: false,
                  ),
                  PieChartSectionData(
                    value: _goodSecToday.toDouble(), // 올바른 자세
                    color: primaryBlue,
                    showTitle: false,
                  ),
                ],
              ),
            ),
          ),

          const SizedBox(width: 12),

          // 텍스트(가운데 정렬)
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Text(
                  '오늘 통계',
                  textAlign: TextAlign.center,
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 15,
                    fontWeight: FontWeight.w700,
                  ),
                ),
                const SizedBox(height: 6),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const Icon(Icons.square, color: errorRed, size: 10),
                    const SizedBox(width: 6),
                    Text(
                      '잘못된 자세 : ${_formatDurationKr(_badSecToday)}',
                      style:
                      const TextStyle(color: Colors.white70, fontSize: 13),
                    ),
                  ],
                ),
                const SizedBox(height: 2),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    const Icon(Icons.square, color: primaryBlue, size: 10),
                    const SizedBox(width: 6),
                    Text(
                      '올바른 자세 : ${_formatDurationKr(_goodSecToday)}',
                      style:
                      const TextStyle(color: Colors.white70, fontSize: 13),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // PRESETS
  Widget _buildPresetArea() {
    if (_presets.isEmpty) return _addPresetButton();

    return Row(
      children: [
        ..._presets.map((entry) {
          final presetName = entry['name'] ?? '이름 없음';
          final presetId = entry['id'] ?? 0;
          return Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _presetButton(presetName, presetId),
            ),
          );
        }).toList(),
        if (_presets.length < 3)
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 6),
              child: _addPresetButton(),
            ),
          ),
      ],
    );
  }

  Widget _addPresetButton() {
    return InkWell(
      onTap: _addPreset,
      borderRadius: BorderRadius.circular(12),
      splashColor: Colors.blue.withOpacity(0.3),
      child: Container(
        height: 56,
        decoration: BoxDecoration(
          color: cardGrey,
          borderRadius: BorderRadius.circular(12),
          border: Border.all(color: primaryBlue, width: 1.3),
        ),
        child: const Center(
          child: Icon(Icons.add_circle_outline, size: 26, color: Colors.white),
        ),
      ),
    );
  }

  Widget _presetButton(String name, int presetId) {
    return InkWell(
      onTap: () => _handlePresetSelect(presetId),
      borderRadius: BorderRadius.circular(12),
      splashColor: Colors.white24,
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 120),
        height: 56,
        decoration: BoxDecoration(
          color: primaryBlue,
          borderRadius: BorderRadius.circular(12),
          boxShadow: [
            BoxShadow(
              color: primaryBlue.withOpacity(0.35),
              blurRadius: 5,
              offset: const Offset(0, 3),
            )
          ],
        ),
        child: Center(
          child: Text(
            name,
            style: const TextStyle(
                color: Colors.white, fontSize: 15, fontWeight: FontWeight.w600),
          ),
        ),
      ),
    );
  }
}
