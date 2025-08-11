import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class AppColors {
  static const backgroundBlack = Color(0xFF0D1117);
  static const cardGrey = Color(0xFF161B22);
  static const cardBorder = Color(0xFF334155);
  static const primaryBlue = Color(0xFF3B82F6);
  static const text = Colors.white;
  static const textSub = Colors.white70;
}

class BleScanScreen extends StatefulWidget {
  const BleScanScreen({Key? key}) : super(key: key);

  @override
  State<BleScanScreen> createState() => _BleScanScreenState();
}

class _BleScanScreenState extends State<BleScanScreen> {
  // 네이티브 GATT 캐시 리프레시용 MethodChannel
  static const _btChannel = MethodChannel('siseon2/bluetooth');

  // 스캔된 기기 및 광고된 서비스 UUID 저장
  final Map<String, BluetoothDevice> _foundDevices = {};
  final Map<String, List<Guid>> _serviceUuids = {};

  String? _selectedMac; // 로그 화면 전환 트리거
  final List<String> _logs = [];
  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

  static const String TARGET_CHAR_UUID =
      "12345678-1234-5678-1234-56789abcdef1"; // 원하는 특성 UUID

  // 스캔 상태/스트림 구독
  bool _isScanning = false;
  StreamSubscription<List<ScanResult>>? _scanSub;
  StreamSubscription<bool>? _isScanningSub;

  @override
  void initState() {
    super.initState();
    // 스캔 상태 구독
    _isScanningSub = FlutterBluePlus.isScanning.listen((s) {
      if (mounted) setState(() => _isScanning = s);
    });
    // 스캔 결과 구독 (1회만)
    _scanSub = FlutterBluePlus.scanResults.listen((results) {
      if (!mounted) return;
      setState(() {
        for (final r in results) {
          final mac = r.device.id.id;
          final name = r.device.name.toLowerCase();
          // 이름에 'pi5' 포함된 기기만 노출
          if (name.contains('pi5')) {
            _foundDevices[mac] = r.device;
            _serviceUuids[mac] = r.advertisementData.serviceUuids;
          }
        }
      });
    });

    _startScan();
  }

  // ────────────────────────────── BLE 제어 ──────────────────────────────
  Future<void> _startScan() async {
    _foundDevices.clear();
    _serviceUuids.clear();
    _logs.clear();
    _connectedDevice = null;
    _writableChar = null;

    try {
      await FlutterBluePlus.stopScan();
      await FlutterBluePlus.startScan(
        timeout: const Duration(seconds: 6),
      );
    } catch (e) {
      _addLog('❌ startScan error: $e');
    }
  }

  Future<void> _stopScan() async {
    try {
      await FlutterBluePlus.stopScan();
    } catch (e) {
      _addLog('❌ stopScan error: $e');
    }
  }

  Future<bool> _clearGattCache() async {
    _addLog('🔄 Clearing GATT cache…');
    try {
      final ok = await _btChannel.invokeMethod<bool>('refreshGatt');
      _addLog(ok == true ? '✅ GATT cache cleared' : '⚠️ GATT cache clear failed');
      return ok ?? false;
    } catch (e) {
      _addLog('❌ Cache clear error: $e');
      return false;
    }
  }

  Future<void> _refreshGattServices(BluetoothDevice device) async {
    if (await _clearGattCache()) {
      try {
        final services = await device.discoverServices();
        _addLog('🧪 Re-discovered services: ${services.length}');
      } catch (e) {
        _addLog('❌ Service rediscovery failed: $e');
      }
    }
  }

  Future<void> _onSelectDevice(String mac) async {
    setState(() {
      _selectedMac = mac;
      _logs.clear();
    });

    await _stopScan();
    final device = _foundDevices[mac];
    if (device == null) return;

    _connectedDevice = device;
    _addLog('🔌 Target device: $mac');

    // 연결 여부 확인 후 연결
    final state = await device.state.first;
    if (state != BluetoothConnectionState.connected) {
      _addLog('🔗 Connecting...');
      try {
        await device.connect(autoConnect: false, timeout: const Duration(seconds: 10));
        _addLog('✅ Connected');
      } catch (e) {
        _addLog('❌ Connection failed: $e');
        return;
      }
    } else {
      _addLog('ℹ️ Already connected');
    }

    // GATT 캐시 리프레시 후 서비스 재탐색 2회
    await _refreshGattServices(device);
    await _refreshGattServices(device);

    // 특성 검색
    try {
      final services = await device.discoverServices();
      for (final s in services) {
        for (final c in s.characteristics) {
          final uuid = c.uuid.toString().toLowerCase();
          if (uuid == TARGET_CHAR_UUID && c.properties.write && c.properties.read) {
            _writableChar = c;
            _addLog('✅ Found writable+readable characteristic');
            break;
          }
        }
        if (_writableChar != null) break;
      }
      if (_writableChar == null) {
        _addLog('⚠️ Writable+Readable not found');
      } else {
        if (!mounted) return;
        Navigator.pop(context, {
          'device': _connectedDevice,
          'writableChar': _writableChar,
        });
      }
    } catch (e) {
      _addLog('❌ Characteristic search failed: $e');
    }
  }

  void _addLog(String msg) {
    setState(() {
      _logs.insert(0, msg);
      if (_logs.length > 200) _logs.removeLast();
    });
  }

  @override
  void dispose() {
    _stopScan();
    _scanSub?.cancel();
    _isScanningSub?.cancel();
    super.dispose();
  }

  // ────────────────────────────── UI ──────────────────────────────
  @override
  Widget build(BuildContext context) {
    final onLogView = _selectedMac != null;

    return Scaffold(
      backgroundColor: AppColors.backgroundBlack,
      appBar: AppBar(
        backgroundColor: AppColors.backgroundBlack,
        elevation: 0,
        centerTitle: true,

        foregroundColor: Colors.white,
        iconTheme: const IconThemeData(color: Colors.white, size: 22),

        leading: onLogView
            ? IconButton(
          // ✅ 더 하얗고 크게
          icon: const Icon(Icons.arrow_back_ios_new, color: Colors.white, size: 22),
          onPressed: () => setState(() => _selectedMac = null),
          splashRadius: 22,
        )
            : null,
        title: Text(
          onLogView ? 'BLE Logs' : 'BLE Scan & Connect',
          style: const TextStyle(
            color: AppColors.text,
            fontFamily: 'Pretendard',
            fontWeight: FontWeight.w700,
          ),
        ),
        actions: [
          if (!onLogView)
            IconButton(
              tooltip: _isScanning ? 'Stop scan' : 'Start scan',
              icon: Icon(_isScanning ? Icons.stop_circle_outlined : Icons.refresh, color: Colors.white70),
              onPressed: () => _isScanning ? _stopScan() : _startScan(),
            ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: onLogView ? _logView() : _deviceListView(),
      ),
      bottomNavigationBar: onLogView
          ? null
          : SafeArea(
        top: false,
        minimum: const EdgeInsets.fromLTRB(16, 0, 16, 16),
        child: SizedBox(
          height: 48,
          child: ElevatedButton.icon(
            onPressed: () => _isScanning ? _stopScan() : _startScan(),
            icon: _isScanning
                ? const SizedBox(
              width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white),
            )
                : const Icon(Icons.bluetooth_searching, color: Colors.white),
            label: const Text(
              '스캔 시작', // 버튼 텍스트는 상태에 따라 아래에서 바꿔도 됨
              style: TextStyle(fontWeight: FontWeight.w700, fontFamily: 'Pretendard'),
            ),
            style: ElevatedButton.styleFrom(
              backgroundColor: AppColors.primaryBlue,
              foregroundColor: Colors.white,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
            ),
          ),
        ),
      ),
    );
  }

  // ────────────────────────────── Views ──────────────────────────────
  Widget _deviceListView() {
    final devices = _foundDevices.entries.toList();

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        // 상태 배지
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
          decoration: BoxDecoration(
            color: AppColors.cardGrey,
            borderRadius: BorderRadius.circular(12),
            border: Border.all(color: AppColors.cardBorder.withOpacity(0.5), width: 1),
          ),
          child: Row(
            children: [
              _dot(_isScanning ? Colors.greenAccent : Colors.orangeAccent),
              const SizedBox(width: 10),
              Expanded(
                child: Text(
                  _isScanning ? '주변 BLE 기기 스캔 중… (pi5만 표시)' : '스캔 대기 상태입니다. 하단 버튼으로 스캔을 시작하세요.',
                  style: const TextStyle(color: AppColors.textSub, fontFamily: 'Pretendard'),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ),
        const SizedBox(height: 16),

        // 리스트
        if (devices.isEmpty)
          Expanded(
            child: _emptyState(),
          )
        else
          Expanded(
            child: ListView.separated(
              itemCount: devices.length,
              separatorBuilder: (_, __) => const SizedBox(height: 10),
              itemBuilder: (context, idx) {
                final mac = devices[idx].key;
                final dev = devices[idx].value;
                final advUuids = _serviceUuids[mac] ?? [];

                return InkWell(
                  onTap: () => _onSelectDevice(mac),
                  borderRadius: BorderRadius.circular(16),
                  child: Container(
                    padding: const EdgeInsets.fromLTRB(14, 12, 14, 12),
                    decoration: BoxDecoration(
                      color: AppColors.cardGrey,
                      borderRadius: BorderRadius.circular(16),
                      border: Border.all(color: Colors.white.withOpacity(0.12), width: 1),
                      boxShadow: [
                        BoxShadow(
                          color: Colors.black.withOpacity(0.20),
                          blurRadius: 12,
                          offset: const Offset(0, 6),
                        ),
                      ],
                    ),
                    child: Row(
                      children: [
                        // 아이콘
                        Container(
                          width: 44,
                          height: 44,
                          decoration: BoxDecoration(
                            color: AppColors.primaryBlue.withOpacity(0.15),
                            borderRadius: BorderRadius.circular(12),
                            border: Border.all(color: AppColors.primaryBlue.withOpacity(0.35), width: 1),
                          ),
                          child: const Icon(Icons.bluetooth, color: AppColors.primaryBlue),
                        ),
                        const SizedBox(width: 12),
                        // 텍스트들
                        Expanded(
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text(
                                dev.name.isNotEmpty ? dev.name : 'Unknown',
                                style: const TextStyle(
                                  fontSize: 16,
                                  fontFamily: 'Pretendard',
                                  fontWeight: FontWeight.w700,
                                  color: AppColors.text,
                                ),
                              ),
                              const SizedBox(height: 4),
                              Text(
                                mac,
                                style: const TextStyle(
                                  fontSize: 12,
                                  color: Colors.white60,
                                  fontFamily: 'Pretendard',
                                ),
                              ),
                              if (advUuids.isNotEmpty) const SizedBox(height: 8),
                              if (advUuids.isNotEmpty)
                                Wrap(
                                  spacing: 6,
                                  runSpacing: 6,
                                  children: advUuids
                                      .map((g) => _chip(g.toString().toLowerCase()))
                                      .toList(growable: false),
                                ),
                            ],
                          ),
                        ),
                        const SizedBox(width: 8),
                        const Icon(Icons.chevron_right, color: Colors.white38),
                      ],
                    ),
                  ),
                );
              },
            ),
          ),
      ],
    );
  }

  Widget _logView() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        // 타겟 기기 표시
        Container(
          padding: const EdgeInsets.all(12),
          decoration: BoxDecoration(
            color: AppColors.cardGrey,
            borderRadius: BorderRadius.circular(12),
            border: Border.all(color: AppColors.cardBorder.withOpacity(0.5), width: 1),
          ),
          child: Row(
            children: [
              const Icon(Icons.device_hub, color: AppColors.primaryBlue),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  'Target: ${_selectedMac ?? '-'}',
                  style: const TextStyle(color: AppColors.text, fontFamily: 'Pretendard'),
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
          ),
        ),
        const SizedBox(height: 12),
        // 로그 리스트
        Expanded(
          child: Container(
            decoration: BoxDecoration(
              color: AppColors.cardGrey,
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: Colors.white.withOpacity(0.10), width: 1),
            ),
            padding: const EdgeInsets.fromLTRB(12, 10, 12, 10),
            child: _logs.isEmpty
                ? const Center(
              child: Text(
                '로그가 없습니다.',
                style: TextStyle(color: AppColors.textSub, fontFamily: 'Pretendard'),
              ),
            )
                : ListView.builder(
              reverse: true,
              itemCount: _logs.length,
              itemBuilder: (context, idx) {
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 4),
                  child: Text(
                    _logs[idx],
                    style: const TextStyle(
                      color: Colors.white,
                      fontFamily: 'RobotoMono',
                      fontSize: 13,
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      ],
    );
  }

  // ────────────────────────────── Widgets ──────────────────────────────
  Widget _dot(Color c) => Container(
    width: 10,
    height: 10,
    decoration: BoxDecoration(color: c, shape: BoxShape.circle),
  );

  Widget _chip(String text) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
      decoration: BoxDecoration(
        color: AppColors.backgroundBlack,
        borderRadius: BorderRadius.circular(999),
        border: Border.all(color: Colors.white.withOpacity(0.12), width: 1),
      ),
      child: Text(
        text,
        style: const TextStyle(
          fontSize: 11,
          color: Colors.white70,
          fontFamily: 'Pretendard',
        ),
      ),
    );
  }

  Widget _emptyState() {
    return Center(
      child: Container(
        padding: const EdgeInsets.all(20),
        decoration: BoxDecoration(
          color: AppColors.cardGrey,
          borderRadius: BorderRadius.circular(16),
          border: Border.all(color: Colors.white.withOpacity(0.12), width: 1),
        ),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: const [
            Icon(Icons.search, color: AppColors.primaryBlue, size: 40),
            SizedBox(height: 12),
            Text(
              '검색된 기기가 없습니다',
              style: TextStyle(
                color: AppColors.text,
                fontSize: 16,
                fontFamily: 'Pretendard',
                fontWeight: FontWeight.w600,
              ),
            ),
            SizedBox(height: 6),
            Text(
              '기기의 전원이 켜져 있고\n이름에 "pi5"가 포함되어야 합니다.',
              textAlign: TextAlign.center,
              style: TextStyle(color: AppColors.textSub, fontFamily: 'Pretendard'),
            ),
          ],
        ),
      ),
    );
  }
}
