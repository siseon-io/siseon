import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class BleScanScreen extends StatefulWidget {
  const BleScanScreen({super.key});

  @override
  State<BleScanScreen> createState() => _BleScanScreenState();
}

class _BleScanScreenState extends State<BleScanScreen> {
  final List<ScanResult> _devices = [];
  final Map<String, bool> _isConnected = {};
  late final Stream<List<ScanResult>> _scanResultsStream;

  @override
  void initState() {
    super.initState();
    _scanResultsStream = FlutterBluePlus.scanResults;
    _startScan();
  }

  Future<void> _startScan() async {
    _devices.clear();
    setState(() {});

    await FlutterBluePlus.stopScan();

    _scanResultsStream.listen((results) {
      for (var result in results) {
        final id = result.device.id.id;
        if (!_devices.any((d) => d.device.id.id == id)) {
          _devices.add(result);
          _isConnected[id] = false;
        }
      }
      setState(() {});
    });

    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 5));
  }

  Future<void> _toggleConnection(BluetoothDevice device) async {
    final id = device.id.id;
    final isConnected = _isConnected[id] ?? false;

    if (isConnected) {
      await device.disconnect();
      _isConnected[id] = false;
      setState(() {});
      return;
    }

    try {
      await device.connect(timeout: const Duration(seconds: 8));
      _isConnected[id] = true;
      setState(() {});

      // ✅ 서비스 UUID 로그 출력
      List<BluetoothService> services = await device.discoverServices();
      for (var service in services) {
        print('🧬 서비스 UUID: ${service.uuid}');
      }
    } catch (e) {
      print('❌ 연결 실패: $e');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('BLE 기기 검색'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _startScan,
          ),
        ],
      ),
      body: ListView.builder(
        itemCount: _devices.length,
        itemBuilder: (context, index) {
          final result = _devices[index];
          final device = result.device;
          final name = result.advertisementData.localName.isNotEmpty
              ? result.advertisementData.localName
              : '(이름 없음)';
          final id = device.id.id;
          final connected = _isConnected[id] ?? false;

          return ListTile(
            leading: const Icon(Icons.bluetooth),
            title: Text(
              name,
              style: const TextStyle(color: Colors.blue, fontWeight: FontWeight.bold),
            ),
            subtitle: Text(id),
            trailing: Icon(
              connected ? Icons.link : Icons.link_off,
              color: connected ? Colors.green : Colors.grey,
            ),
            onTap: () => _toggleConnection(device),
          );
        },
      ),
    );
  }
}
