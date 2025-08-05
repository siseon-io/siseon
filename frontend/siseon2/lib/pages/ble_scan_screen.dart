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
  List<String> _logs = [];
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

    FlutterBluePlus.startScan(timeout: const Duration(seconds: 5));
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
      _addLog('🧹 Cleaning previous connection...');
      try {
        await device.disconnect();
      } catch (_) {}
      await Future.delayed(const Duration(seconds: 1));

      await device.connect(
        autoConnect: false,
        timeout: const Duration(seconds: 10),
      );
      _addLog('✅ Connected to $mac');
      _connectedDevice = device;

      final services = await device.discoverServices();
      _addLog('🧪 Found ${services.length} services');

      for (var service in services) {
        final serviceUuid = service.uuid.toString().toLowerCase();
        _addLog('🔍 Checking service: $serviceUuid');
        for (var char in service.characteristics) {
          final charUuid = char.uuid.toString().toLowerCase();
          _addLog('📍 Characteristic: $charUuid');
          if (charUuid == TARGET_CHAR_UUID &&
              char.properties.write &&
              char.properties.read) {
            _addLog('✅ Found writable+readable characteristic');
            _writableChar = char;

            // Write 테스트
            final dataToSend = [0x41, 0x42, 0x43]; // "ABC"
            await char.write(dataToSend);
            _addLog('📤 Wrote data: ${_toHex(dataToSend)}');

            // Read 테스트
            final received = await char.read();
            _addLog('📥 Read data: ${_toHex(received)}');
          }
        }
      }

      if (_writableChar == null) {
        _addLog('⚠️ Writable+Readable characteristic not found!');
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

  String _toHex(List<int> bytes) =>
      bytes.map((b) => b.toRadixString(16).padLeft(2, '0')).join(' ');

  void _addLog(String msg) {
    setState(() {
      _logs.insert(0, msg);
      if (_logs.length > 100) _logs.removeLast();
    });
  }

  @override
  void dispose() {
    FlutterBluePlus.stopScan();
    // 연결 유지하도록 disconnect 호출 제거!
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
        child: _selectedMac == null ? _deviceListView() : _logView(),
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
          '📰 Scanning for devices...',
          style: TextStyle(fontSize: 16),
        ),
      );
    }

    return ListView.separated(
      itemCount: devices.length,
      separatorBuilder: (_, __) => const SizedBox(height: 8),
      itemBuilder: (context, index) {
        final entry = devices[index];
        final mac = entry.key;
        final name = entry.value.name.isNotEmpty
            ? entry.value.name
            : 'Unknown';
        return Card(
          elevation: 4,
          shape:
          RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          child: ListTile(
            leading: const Icon(Icons.bluetooth, color: Colors.blueGrey),
            title: Text(name,
                style: const TextStyle(
                    fontSize: 18, fontWeight: FontWeight.bold)),
            subtitle: Text(mac,
                style: const TextStyle(
                    fontSize: 14, color: Colors.black54)),
            onTap: () => _onSelectDevice(mac),
          ),
        );
      },
    );
  }

  Widget _logView() {
    return Column(
      children: [
        const Text('🔗 연결 로그',
            style: TextStyle(fontWeight: FontWeight.bold)),
        const SizedBox(height: 12),
        Expanded(
          child: ListView.builder(
            itemCount: _logs.length,
            reverse: true,
            itemBuilder: (context, index) {
              return Padding(
                padding: const EdgeInsets.symmetric(vertical: 4.0),
                child: Text(_logs[index]),
              );
            },
          ),
        ),
      ],
    );
  }
}
