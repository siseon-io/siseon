// lib/pages/ble_test_page.dart
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

class BleTestPage extends StatefulWidget {
  const BleTestPage({super.key});

  @override
  State<BleTestPage> createState() => _BleTestPageState();
}

class _BleTestPageState extends State<BleTestPage> {
  BluetoothDevice? connectedDevice;
  BluetoothCharacteristic? notifyChar;
  BluetoothCharacteristic? writeChar;

  @override
  void initState() {
    super.initState();
    _startBLE();
  }

  Future<void> _startBLE() async {
    await Permission.bluetooth.request();
    await Permission.bluetoothScan.request();
    await Permission.bluetoothConnect.request();
    await Permission.location.request();

    FlutterBluePlus.startScan(timeout: const Duration(seconds: 5));

    FlutterBluePlus.scanResults.listen((results) async {
      for (ScanResult r in results) {
        if (r.device.name == 'MyBLEDevice') { // ✅ 기기 이름 바꿔
          print('🔍 발견: ${r.device.name}');
          await FlutterBluePlus.stopScan();
          await r.device.connect();
          connectedDevice = r.device;
          print('✅ 연결됨: ${r.device.name}');
          _discoverServices();
          break;
        }
      }
    });
  }

  Future<void> _discoverServices() async {
    List<BluetoothService> services = await connectedDevice!.discoverServices();

    for (var service in services) {
      for (var c in service.characteristics) {
        if (c.properties.notify) {
          notifyChar = c;
          await notifyChar!.setNotifyValue(true);
          notifyChar!.lastValueStream.listen((value) {
            print('📥 받은 데이터: ${utf8.decode(value)}');
          });
        }

        if (c.properties.write) {
          writeChar = c;
        }
      }
    }
  }

  Future<void> _sendData(String text) async {
    if (writeChar != null) {
      await writeChar!.write(utf8.encode(text));
      print('📤 보낸 데이터: $text');
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('BLE 테스트')),
      body: Center(
        child: ElevatedButton(
          onPressed: () => _sendData('hello BLE!'),
          child: const Text('데이터 보내기'),
        ),
      ),
    );
  }
}
s