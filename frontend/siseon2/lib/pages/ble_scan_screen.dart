import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class BleScanScreen extends StatefulWidget {
  const BleScanScreen({Key? key}) : super(key: key);

  @override
  State<BleScanScreen> createState() => _BleScanScreenState();
}

class _BleScanScreenState extends State<BleScanScreen> {
  final Map<String, BluetoothDevice> _foundDevices = {};
  String? _selectedMac;
  final List<String> _logs = [];
  BluetoothDevice? _connectedDevice;
  BluetoothCharacteristic? _writableChar;

  static const String TARGET_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
  static const String TARGET_CHAR_UUID   = "12345678-1234-5678-1234-56789abcdef1";

  @override
  void initState() {
    super.initState();
    _startScan();
  }

  void _startScan() {
    _foundDevices.clear();
    _logs.clear();
    _connectedDevice = null;
    _writableChar   = null;

    // 오직 TARGET_SERVICE_UUID 를 광고하는 기기만 스캔
    FlutterBluePlus.startScan(
      timeout: const Duration(seconds: 5),
      withServices: [Guid(TARGET_SERVICE_UUID)],
    );

    FlutterBluePlus.scanResults.listen((results) {
      setState(() {
        for (var r in results) {
          _foundDevices[r.device.id.id] = r.device;
        }
      });
    });
  }

  void _onSelectDevice(String mac) async {
    setState(() {
      _selectedMac = mac;
      _logs.clear();
    });

    FlutterBluePlus.stopScan();
    final device = _foundDevices[mac];
    if (device == null) return;

    _addLog('🔌 Connecting to $mac...');
    try {
      // 이전 연결 초기화
      try {
        await device.disconnect();
      } catch (_) {}
      await Future.delayed(const Duration(seconds: 1));

      await device.connect(
        autoConnect: false,
        timeout: const Duration(seconds: 10),
      );
      _addLog('✅ Connected');

      _connectedDevice = device;
      final services = await device.discoverServices();
      _addLog('🧪 Services found: ${services.length}');

      for (var service in services) {
        for (var char in service.characteristics) {
          final uuid = char.uuid.toString().toLowerCase();
          if (uuid == TARGET_CHAR_UUID &&
              char.properties.write &&
              char.properties.read) {
            _addLog('✅ Found writable+readable characteristic');
            _writableChar = char;
            break;
          }
        }
        if (_writableChar != null) break;
      }

      if (_writableChar == null) {
        _addLog('⚠️ Writable+Readable not found');
      } else {
        Navigator.pop(context, {
          'device': _connectedDevice,
          'writableChar': _writableChar,
        });
      }
    } catch (e) {
      _addLog('❌ Connection failed: $e');
    }
  }

  void _addLog(String msg) {
    setState(() {
      _logs.insert(0, msg);
      if (_logs.length > 100) _logs.removeLast();
    });
  }

  @override
  void dispose() {
    FlutterBluePlus.stopScan();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey[100],
      appBar: AppBar(
        title: const Text('BLE Scan & Connect'),
        backgroundColor: Colors.blueGrey,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: _selectedMac == null
            ? _deviceListView()
            : _logView(),
      ),
    );
  }

  Widget _deviceListView() {
    final devices = _foundDevices.entries.toList()
      ..sort((a, b) {
        final aHas = a.value.name.isNotEmpty;
        final bHas = b.value.name.isNotEmpty;
        if (aHas && !bHas) return -1;
        if (!aHas && bHas) return 1;
        return a.value.name.compareTo(b.value.name);
      });

    if (devices.isEmpty) {
      return const Center(
        child: Text(
          '📰 No devices with target service found',
          style: TextStyle(fontSize: 16),
        ),
      );
    }

    return ListView.separated(
      itemCount: devices.length,
      separatorBuilder: (_, __) => const SizedBox(height: 8),
      itemBuilder: (context, idx) {
        final entry = devices[idx];
        final mac  = entry.key;
        final name = entry.value.name.isNotEmpty ? entry.value.name : 'Unknown';
        return Card(
          elevation: 4,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          child: ListTile(
            leading: const Icon(Icons.bluetooth, color: Colors.blueGrey),
            title: Text(name,
                style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
            subtitle: Text(mac,
                style: const TextStyle(fontSize: 14, color: Colors.black54)),
            onTap: () => _onSelectDevice(mac),
          ),
        );
      },
    );
  }

  Widget _logView() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text('🔗 Connection Logs',
            style: TextStyle(fontWeight: FontWeight.bold)),
        const SizedBox(height: 12),
        Expanded(
          child: ListView.builder(
            itemCount: _logs.length,
            itemBuilder: (context, idx) {
              return Padding(
                padding: const EdgeInsets.symmetric(vertical: 4.0),
                child: Text(_logs[idx]),
              );
            },
          ),
        ),
      ],
    );
  }
}
