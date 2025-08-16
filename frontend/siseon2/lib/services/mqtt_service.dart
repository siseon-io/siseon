// lib/services/mqtt_service.dart
import 'dart:io';
import 'dart:convert';
import 'package:flutter/services.dart' show rootBundle;
import 'package:flutter_dotenv/flutter_dotenv.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:siseon2/services/device_cache_service.dart';

class MqttService {
  static final MqttService _instance = MqttService._internal();
  factory MqttService() => _instance;
  late MqttServerClient client;

  bool _isConnecting = false;

  MqttService._internal();

  Future<void> connect() async {
    if (_isConnecting) return;
    _isConnecting = true;

    try {
      print("1️⃣ MQTT 연결 시작 (TLS + 자체서명 인증서)");

      // ✅ .env에서 값 불러오기
      final host = dotenv.env['MQTT_HOST']!;
      final port = int.parse(dotenv.env['MQTT_PORT']!);
      final username = dotenv.env['MQTT_USERNAME']!;
      final password = dotenv.env['MQTT_PASSWORD']!;
      final certPath = dotenv.env['MQTT_CERT']!;

      // 인증서 로드
      final certData = await rootBundle.load(certPath);
      final context = SecurityContext(withTrustedRoots: false);
      context.setTrustedCertificatesBytes(certData.buffer.asUint8List());

      // MQTT 클라이언트 초기화
      final clientId = 'flutter_monitor_client_${DateTime.now().millisecondsSinceEpoch}';
      client = MqttServerClient.withPort(host, clientId, port)
        ..secure = true
        ..securityContext = context
        ..setProtocolV311()
        ..logging(on: true)
        ..keepAlivePeriod = 60
        ..onConnected = onConnected
        ..onDisconnected = onDisconnected
        ..onSubscribed = onSubscribed;

      client.connectionMessage = MqttConnectMessage()
          .withClientIdentifier(clientId)
          .startClean();

      print("🟢 MQTT Connect 시도: username=$username");
      await client.connect(username, password);
    } catch (e) {
      print('❌ MQTT 연결 실패: $e');
      client.disconnect();
    } finally {
      _isConnecting = false;
    }
  }

  // 연결 성공 시 deviceSerial 기반으로 자동 구독
  Future<void> onConnected() async {
    print('✅ MQTT 연결 성공');
    try {
      final device = await DeviceCacheService.loadDevice();
      final serial = device?['serial']?.toString();
      if (serial != null && serial.isNotEmpty) {
        subscribe('/control_mode/$serial');
      } else {
        print('⚠️ 구독 실패: deviceSerial 없음');
      }
    } catch (e) {
      print('⚠️ 구독 시 deviceSerial 로드 실패: $e');
    }
  }

  void onDisconnected() => print('❌ MQTT 연결 끊김');
  void onSubscribed(String topic) => print('📌 구독 성공: $topic');

  void publish(String topic, Map<String, dynamic> payload) async {
    if (client.connectionStatus?.state != MqttConnectionState.connected) {
      print('⚠️ MQTT 연결 안 됨: 재연결 시도');
      await connect();
    }

    if (client.connectionStatus?.state == MqttConnectionState.connected) {
      final builder = MqttClientPayloadBuilder();
      builder.addString(jsonEncode(payload));
      client.publishMessage(topic, MqttQos.atMostOnce, builder.payload!);
      print("📤 MQTT 발행: $topic → ${jsonEncode(payload)}");
    } else {
      print('❌ MQTT 발행 실패: 연결 불가');
    }
  }

  void subscribe(String topic) {
    if (client.connectionStatus?.state == MqttConnectionState.connected) {
      client.subscribe(topic, MqttQos.atLeastOnce);
      print("🔔 MQTT 구독 요청: $topic");
    } else {
      print('⚠️ MQTT 연결 안 됨: 구독 실패');
    }
  }
}

final mqttService = MqttService();
