// lib/pages/ble_scan_screen.dart
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

// 전역 세션을 쓰려면 이 임포트 유지 (원치 않으면 지워도 앱 동작엔 영향 없음)
import 'package:siseon2/services/ble_session.dart';

class AppColors {
  static const backgroundBlack = Color(0xFF0D1117);
  static const cardGrey = Color(0xFF161B22);
  static const cardBorder = Color(0xFF334155);
  static const primaryBlue = Color(0xFF3B82F6);
  static const text = Colors.white;
  static const textSub = Colors.white70;
}

class BleScanScreen extends StatefulWidget {
  /// 있으면 이 UUID로 매칭, 없으면 자동 선택
  final String? targetCharUuid;

  const BleScanScreen({Key? key, this.targetCharUuid}) : super(key: key);

  @override
  State<BleScanScreen> createState() => _BleScanScreenState();
}

class _BleScanScreenState extends State<BleScanScreen> {
  static const _btChannel = MethodChannel('siseon2/bluetooth');

  final Map<String, BluetoothDevice> _foundDevices = {};
  final Map<String, List<Guid>> _serviceUuids = {};

  final List<String> _logs = [];
  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

  bool _isScanning = false;
  bool _busy = false; // ✅ 연결 중 오버레이 표시용
  StreamSubscription<List<ScanResult>>? _scanSub;
  StreamSubscription<bool>? _isScanningSub;
  StreamSubscription<BluetoothConnectionState>? _connStateSub;

  // ✅ 정상적으로 결과를 넘겨(핸드오프) 이 화면을 닫는지 여부
  bool _handoff = false;

  // 표준 서비스(무시 대상)
  static const Set<String> _ignoreSvc = {
    '00001800-0000-1000-8000-00805f9b34fb', // Generic Access
    '00001801-0000-1000-8000-00805f9b34fb', // Generic Attribute
  };

  @override
  void initState() {
    super.initState();

    // 화면 내 플러그인 로그 억제 (필요 시 verbose로 바꿔 디버깅)
    FlutterBluePlus.setLogLevel(LogLevel.none);

    _isScanningSub = FlutterBluePlus.isScanning.listen((s) {
      if (mounted) setState(() => _isScanning = s);
    });

    _scanSub = FlutterBluePlus.scanResults.listen((results) {
      if (!mounted) return;
      setState(() {
        for (final r in results) {
          final mac = r.device.id.id;
          final name = (r.device.name.isNotEmpty ? r.device.name : '');
          if (name.toLowerCase().contains('pi5')) {
            _foundDevices[mac] = r.device;
            _serviceUuids[mac] = r.advertisementData.serviceUuids;

            // 콘솔에도 광고 UUID를 즉시 찍어줌
            final adv = r.advertisementData.serviceUuids
                .map((g) => g.toString().toLowerCase())
                .toList();
            debugPrint('ADV uuids($mac): ${adv.join(', ')}');
          }
        }
      });
    });

    _startScan();
  }

  Future<void> _startScan() async {
    _foundDevices.clear();
    _serviceUuids.clear();
    _logs.clear();
    _connectedDevice = null;
    _writableChar = null;

    try {
      await FlutterBluePlus.stopScan();
      await FlutterBluePlus.startScan(timeout: const Duration(seconds: 6));
    } catch (e) {
      _addLog('❌ startScan error: ${_formatBleError(e)}');
    }
  }

  Future<void> _stopScan() async {
    try {
      await FlutterBluePlus.stopScan();
    } catch (e) {
      _addLog('❌ stopScan error: ${_formatBleError(e)}');
    }
  }

  Future<bool> _clearGattCache() async {
    _addLog('🔄 Clearing GATT cache…');
    try {
      final ok = await _btChannel.invokeMethod<bool>('refreshGatt');
      _addLog(ok == true ? '✅ GATT cache cleared' : '⚠️ GATT cache clear failed');
      return ok ?? false;
    } catch (e) {
      _addLog('❌ Cache clear error: ${_formatBleError(e)}');
      return false;
    }
  }

  // 광고에서 온 UUID 중 커스텀(비표준) 서비스 후보 1개 뽑기
  String? _pickCustomAdvSvc(List<Guid> advUuids) {
    for (final g in advUuids) {
      final s = g.toString().toLowerCase();
      // 표준 16-bit 확장 형태/무시 목록 제외
      if (_ignoreSvc.contains(s)) continue;
      if (_looksLike16BitBase(s)) continue;
      return s; // 커스텀 128-bit로 판단
    }
    return null;
  }

  bool _looksLike16BitBase(String uuidLower) {
    // 16-bit 확장: 0000XXXX-0000-1000-8000-00805f9b34fb
    return RegExp(r'^0000[0-9a-f]{4}-0000-1000-8000-00805f9b34fb$')
        .hasMatch(uuidLower);
  }

  Future<void> _onSelectDevice(String mac) async {
    if (_busy) return;
    _setBusy(true);
    _logs.clear();

    try {
      await _stopScan();
      final device = _foundDevices[mac];
      if (device == null) {
        return;
      }

      _connectedDevice = device;
      _addLog('🔌 Target device: $mac');

      // 🔎 광고에 실린 서비스 UUID들 확인 + 커스텀 서비스 힌트 추출
      final advUuids = _serviceUuids[mac] ?? const [];
      String? advSvcHint;
      if (advUuids.isEmpty) {
        _addLog('📣 ADV.serviceUuids = [] (광고에 UUID 미포함/OS 캐시 이슈 가능)');
      } else {
        final advList = advUuids.map((g) => g.toString().toLowerCase()).toList();
        _addLog('📣 ADV.serviceUuids = [${advList.join(', ')}]');
        advSvcHint = _pickCustomAdvSvc(advUuids);
        if (advSvcHint != null) {
          _addLog('🔎 advSvcHint = $advSvcHint');
        }
      }

      // 🔎 외부에서 타겟 캐릭터 UUID가 들어왔다면 표시 (우선권 가짐)
      final wantChar = widget.targetCharUuid?.toLowerCase().trim();
      if (wantChar != null && wantChar.isNotEmpty) {
        _addLog('🎯 targetCharUuid (external) = $wantChar');
      }

      // 연결 상태 실시간 로그
      _connStateSub?.cancel();
      _connStateSub = device.state.listen((s) {
        _addLog('🔄 Device state: $s');
      });

      // 1) 연결
      final current = await device.state.first;
      if (current != BluetoothConnectionState.connected) {
        _addLog('🔗 Connecting...');
        try {
          await device.connect(
            autoConnect: false,
            timeout: const Duration(seconds: 10),
          );
          _addLog('✅ Connected');
        } on PlatformException catch (e) {
          _addLog('❌ Connection failed (PlatformException): ${_formatPlatformException(e)}');
          await _safeDisconnect(device);
          await _clearGattCache(); // 일부 GATT 133 등 복구용
          return;
        } catch (e) {
          _addLog('❌ Connection failed: ${_formatBleError(e)}');
          await _safeDisconnect(device);
          return;
        }
      } else {
        _addLog('ℹ️ Already connected');
      }

      // 2) 연결 직후 안정화
      try {
        await device.requestConnectionPriority(
          connectionPriorityRequest: ConnectionPriority.high,
        );
      } catch (_) {}
      await Future.delayed(const Duration(milliseconds: 300));
      try {
        await device.requestMtu(247);
      } catch (_) {}
      await Future.delayed(const Duration(milliseconds: 150));

      // 3) 서비스 탐색(1회)
      List<BluetoothService> services = const [];
      try {
        services = await device.discoverServices();
        _addLog('🧪 Discovered services: ${services.length}');

        // 🔎 각 서비스/캐릭터 전부 덤프
        for (final s in services) {
          _addLog('🧩 Service: ${s.uuid.toString().toLowerCase()} '
              '(${s.characteristics.length} chars)');
          for (final c in s.characteristics) {
            final p = c.properties;
            _addLog('   └─ Char: ${c.uuid.toString().toLowerCase()} '
                '[read=${p.read}, write=${p.write}, writeNR=${p.writeWithoutResponse}, '
                'notify=${p.notify}, indicate=${p.indicate}]');
          }
        }
      } on PlatformException catch (e) {
        _addLog('❌ discoverServices failed (PlatformException): ${_formatPlatformException(e)}');
        await _safeDisconnect(device);
        return;
      } catch (e) {
        _addLog('❌ discoverServices failed: ${_formatBleError(e)}');
        await _safeDisconnect(device);
        return;
      }

      // 4) 서비스/캐릭터 선택 로직
      try {
        // 4-1) 선호 서비스 집합 만들기: 광고 힌트 > 비표준(커스텀) 서비스
        List<BluetoothService> preferredServices = [];

        if (advSvcHint != null) {
          final match = services.where((s) =>
          s.uuid.toString().toLowerCase() == advSvcHint);
          preferredServices.addAll(match);
        }

        if (preferredServices.isEmpty) {
          preferredServices = services.where((s) {
            final su = s.uuid.toString().toLowerCase();
            return !_ignoreSvc.contains(su) && !_looksLike16BitBase(su);
          }).toList();
        }

        // 폴백: 그래도 없으면 전체 사용
        final searchSpace = preferredServices.isNotEmpty
            ? preferredServices
            : services;

        // 4-2) 캐릭터 선택
        BluetoothCharacteristic? bestReadableWritable;
        BluetoothCharacteristic? bestWritable;

        for (final s in searchSpace) {
          final svcLower = s.uuid.toString().toLowerCase();
          if (_ignoreSvc.contains(svcLower)) continue;

          for (final c in s.characteristics) {
            final uuidLower = c.uuid.toString().toLowerCase();
            final canWrite = c.properties.write || c.properties.writeWithoutResponse;
            final canRead  = c.properties.read;

            // 외부에서 특정 char 요구하면 그것부터
            if (wantChar != null && wantChar.isNotEmpty) {
              if (uuidLower == wantChar && canWrite) {
                _writableChar = c;
                break;
              }
            } else {
              if (canWrite && canRead && bestReadableWritable == null) {
                bestReadableWritable = c;
              }
              if (canWrite && bestWritable == null) {
                bestWritable = c;
              }
            }
          }
          if (_writableChar != null) break;
        }

        _writableChar ??= bestReadableWritable ?? bestWritable;

        if (_writableChar == null) {
          _addLog('⚠️ 쓸 수 있는 특성을 찾지 못했습니다. (write/read 속성 확인 필요)');
        } else {
          // ✅ 선택 결과 자세히 출력 (service + char UUID)
          final c = _writableChar!;
          final p = c.properties;
          final svcUuid  = c.serviceUuid.toString().toLowerCase();
          final charUuid = c.uuid.toString().toLowerCase();

          _addLog('✅ Selected service: $svcUuid');
          _addLog('✅ Selected characteristic: $charUuid');
          _addLog('🧷 Char props → read=${p.read}, write=${p.write}, '
              'writeNR=${p.writeWithoutResponse}, notify=${p.notify}, indicate=${p.indicate}');

          // 🔒 링크 검증 (read 또는 notify on/off)
          final verified = await _verifyLink(_connectedDevice!, _writableChar!);
          _addLog(verified ? '🔒 Link verify: OK' : '⚠️ Link verify: skipped or best-effort');

          // (옵션) 전역 세션에 저장 — 전역 사용 원치 않으면 이 3줄 주석 처리
          try {
            await bleSession.setConnected(_connectedDevice!, _writableChar!);
            _addLog('🌐 Global session set');
          } catch (_) {}

          if (!mounted) return;

          // ✅ 결과 정상 전달 → 이 화면에서는 disconnect 하지 않음
          _handoff = true;

          Navigator.pop(context, {
            'device': _connectedDevice,
            'writableChar': _writableChar,
            'serviceUuid': c.serviceUuid.toString(),
            'charUuid': c.uuid.toString(),
          });
        }
      } catch (e) {
        _addLog('❌ Characteristic search failed: ${_formatBleError(e)}');
      }
    } finally {
      _setBusy(false);
    }
  }

  // 🔎 무해한 링크 검증: read 가능하면 read, 아니면 notify on/off
  Future<bool> _verifyLink(BluetoothDevice d, BluetoothCharacteristic c) async {
    await Future.delayed(const Duration(milliseconds: 120));

    if (c.properties.read) {
      try {
        final data = await c.read().timeout(const Duration(seconds: 1));
        _addLog('🔎 Read probe ok (${data.length}B)');
        return true;
      } catch (e) {
        _addLog('⚠️ Read probe failed: ${_formatBleError(e)}');
      }
    }

    if (c.properties.notify || c.properties.indicate) {
      try {
        await c.setNotifyValue(true).timeout(const Duration(seconds: 1));
        _addLog('🔔 Notify enabled');
        await Future.delayed(const Duration(milliseconds: 80));
        await c.setNotifyValue(false).timeout(const Duration(seconds: 1));
        _addLog('🔕 Notify disabled');
        return true;
      } catch (e) {
        _addLog('⚠️ Notify probe failed: ${_formatBleError(e)}');
      }
    }
    return false; // 검증 못해도 연결은 유지
  }

  Future<void> _safeDisconnect(BluetoothDevice d) async {
    try {
      await d.disconnect();
      _addLog('🔌 Disconnected');
    } catch (_) {}
  }

  String _formatBleError(Object e) {
    if (e is PlatformException) {
      return _formatPlatformException(e);
    }
    final msg = e.toString();
    final status = _extractGattStatus(msg);
    final hint = status != null ? _gattHint(status) : null;
    return status == null ? msg : '$msg (status=$status${hint != null ? ", $hint" : ""})';
  }

  String _formatPlatformException(PlatformException e) {
    final statusFromDetails = _extractGattStatus('${e.details}');
    final statusFromMessage = _extractGattStatus('${e.message}');
    final status = statusFromDetails ?? statusFromMessage;

    final base = '[${e.code}] ${e.message ?? ''} ${e.details ?? ''}'.trim();
    if (status == null) return base;

    final hint = _gattHint(status);
    return '$base (status=$status${hint != null ? ", $hint" : ""})';
  }

  int? _extractGattStatus(String text) {
    final m = RegExp(r'(status|gatt|ATT)_?(code|status)?[=: ]+(-?\d+)').firstMatch(text);
    if (m != null) {
      return int.tryParse(m.group(3)!);
    }
    final n = RegExp(r'\b(133|62|8|22|19|257)\b').firstMatch(text);
    if (n != null) return int.tryParse(n.group(1)!);
    return null;
  }

  String? _gattHint(int status) {
    switch (status) {
      case 133:
        return 'Android Generic GATT Error(133) — 캐시 문제일 수 있음. 다시 시도/캐시 초기화 권장';
      case 8:
        return 'GATT_CONN_TIMEOUT — 기기 응답 지연/거리/간섭 확인';
      case 62:
        return 'GATT_CONN_FAIL_ESTABLISH — 페어링/전원/거리 확인';
      case 22:
        return 'GATT_CONN_TERMINATE_PEER_USER — 상대가 연결 종료';
      case 19:
        return 'GATT_CONN_TERMINATE_LOCAL_HOST — 로컬이 연결 종료';
      case 257:
        return 'INSUFFICIENT AUTHENTICATION — 권한/페어링 필요';
      default:
        return null;
    }
  }

  void _addLog(String msg) {
    debugPrint(msg); // 콘솔(Logcat)에도 같이 출력
    // 화면에는 로그 안 띄우지만, 필요하면 여기에 SnackBar 등 붙일 수 있음
    _logs.insert(0, msg);
    if (_logs.length > 200) {
      _logs.removeLast();
    }
  }

  void _setBusy(bool v) {
    if (!mounted) return;
    setState(() => _busy = v);
  }

  @override
  void dispose() {
    _stopScan();
    _scanSub?.cancel();
    _isScanningSub?.cancel();
    _connStateSub?.cancel();

    // ✅ 정상 핸드오프가 아니면(사용자 취소/실패 등) 여기서 정리
    if (!_handoff) {
      _connectedDevice?.disconnect();
    }

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: AppColors.backgroundBlack,
      appBar: AppBar(
        backgroundColor: AppColors.backgroundBlack,
        elevation: 0,
        centerTitle: true,
        foregroundColor: Colors.white,
        iconTheme: const IconThemeData(color: Colors.white, size: 22),
        title: const Text(
          'BLE Scan & Connect',
          style: TextStyle(
            color: AppColors.text,
            fontFamily: 'Pretendard',
            fontWeight: FontWeight.w700,
          ),
        ),
        actions: [
          IconButton(
            tooltip: _isScanning ? 'Stop scan' : 'Start scan',
            icon: Icon(
              _isScanning ? Icons.stop_circle_outlined : Icons.refresh,
              color: Colors.white70,
            ),
            onPressed: () => _isScanning ? _stopScan() : _startScan(),
          ),
        ],
      ),
      body: Stack(
        children: [
          Padding(
            padding: const EdgeInsets.all(16),
            child: _deviceListView(),
          ),
          if (_busy)
            Positioned.fill(
              child: Container(
                color: Colors.black54,
                child: Center(
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: const [
                      SizedBox(
                        width: 36,
                        height: 36,
                        child: CircularProgressIndicator(strokeWidth: 3),
                      ),
                      SizedBox(height: 12),
                      Text(
                        '기기와 연결 중…',
                        style: TextStyle(color: Colors.white, fontWeight: FontWeight.w600),
                      ),
                    ],
                  ),
                ),
              ),
            ),
        ],
      ),
      bottomNavigationBar: SafeArea(
        top: false,
        minimum: const EdgeInsets.fromLTRB(16, 0, 16, 16),
        child: SizedBox(
          height: 48,
          child: ElevatedButton.icon(
            onPressed: () => _isScanning ? _stopScan() : _startScan(),
            icon: _isScanning
                ? const SizedBox(
              width: 18,
              height: 18,
              child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white),
            )
                : const Icon(Icons.bluetooth_searching, color: Colors.white),
            label: const Text(
              '스캔 시작',
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

  Widget _deviceListView() {
    final devices = _foundDevices.entries.toList();

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
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
        if (devices.isEmpty)
          Expanded(child: _emptyState())
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
