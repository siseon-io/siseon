import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';

class DeviceCacheService {
  static const _baseUrl = 'http://i13b101.p.ssafy.io:8080';

  // ── 키 생성 (프로필별 분리) ────────────────────────────────────────────────
  static String _serialKey(int profileId) => 'deviceSerial_$profileId';
  static String _regKey(int profileId) => 'isDeviceRegistered_$profileId';

  static Future<int?> _currentProfileId() async {
    final profile = await ProfileCacheService.loadProfile();
    return profile?['id'] as int?;
  }

  // ── 서버에서 기기 조회 → 해당 프로필로 캐싱 ─────────────────────────────
  static Future<void> fetchAndCacheDevice({int? profileId}) async {
    final pid = profileId ?? await _currentProfileId();
    if (pid == null) {
      throw Exception('⚠️ 프로필 정보를 불러올 수 없습니다.');
    }

    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$_baseUrl/api/device/profile/$pid');

    try {
      final res = await http.get(
        url,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
      );

      // 404/204면 "없음"으로 간주하고 캐시 비움
      if (res.statusCode == 404 || res.statusCode == 204) {
        await clearDeviceForProfile(pid);
        return;
      }

      // 200만 성공 처리, 그 외는 캐시 유지(일시 네트워크 오류 가정)
      if (res.statusCode != 200) {
        return;
      }

      if (res.body.isEmpty) {
        await clearDeviceForProfile(pid);
        return;
      }

      final body = jsonDecode(res.body);

      // 응답 케이스 분기
      dynamic deviceData;

      if (body is List) {
        deviceData = body.isNotEmpty ? body.first : null;
      } else if (body is Map && body.containsKey('serials')) {
        final serials = body['serials'] as List<dynamic>;
        if (serials.isNotEmpty) {
          await saveDeviceForProfile(pid, {'serial': serials.first.toString()});
          return;
        }
        deviceData = null;
      } else {
        deviceData = body;
      }

      final serial = () {
        if (deviceData == null) return null;
        if (deviceData is Map) {
          return deviceData['serial'] ?? deviceData['serialNumber'];
        }
        return null;
      }();

      if (serial != null && serial.toString().isNotEmpty) {
        await saveDeviceForProfile(pid, {'serial': serial.toString()});
      } else {
        await clearDeviceForProfile(pid);
      }
    } catch (_) {
      // 네트워크/파싱 에러 시 캐시를 건드리지 않음
      return;
    }
  }

  // ── 등록 직후 즉시 반영(UX 개선용): 로컬에 먼저 저장 후 서버 확인 ─────────
  static Future<void> saveDeviceSerialLocal({int? profileId, required String serial}) async {
    final pid = profileId ?? await _currentProfileId();
    if (pid == null) {
      throw Exception('⚠️ 프로필 정보를 불러올 수 없습니다.');
    }
    await saveDeviceForProfile(pid, {'serial': serial});
    // 서버 확인은 백그라운드로
    // ignore: unawaited_futures
    fetchAndCacheDevice(profileId: pid);
  }

  // ── 프로필별 저장/조회/삭제 API ─────────────────────────────────────────
  static Future<void> saveDeviceForProfile(int profileId, Map<String, dynamic> device) async {
    final prefs = await SharedPreferences.getInstance();
    final serial = device['serial'] ?? device['serialNumber'];
    if (serial == null || serial.toString().isEmpty) {
      throw ArgumentError('serial(혹은 serialNumber)이 없습니다.');
    }
    await prefs.setString(_serialKey(profileId), serial.toString());
    await prefs.setBool(_regKey(profileId), true);
  }

  static Future<Map<String, dynamic>?> loadDeviceForProfile(int profileId) async {
    final prefs = await SharedPreferences.getInstance();
    final isRegistered = prefs.getBool(_regKey(profileId)) ?? false;
    final serial = prefs.getString(_serialKey(profileId));
    if (!isRegistered || serial == null) {
      return null;
    }
    return {'serial': serial, 'isRegistered': isRegistered};
  }

  static Future<void> clearDeviceForProfile(int profileId) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove(_serialKey(profileId));
    await prefs.setBool(_regKey(profileId), false);
  }

  // ── 호환용(현재 프로필 기준) ───────────────────────────────────────────
  static Future<Map<String, dynamic>?> loadDevice() async {
    final pid = await _currentProfileId();
    if (pid == null) return null;
    return loadDeviceForProfile(pid);
  }

  static Future<void> clearDevice() async {
    final prefs = await SharedPreferences.getInstance();
    final keys = prefs.getKeys();
    for (final k in keys) {
      if (k.startsWith('deviceSerial_')) {
        await prefs.remove(k);
      } else if (k.startsWith('isDeviceRegistered_')) {
        await prefs.remove(k);
      }
    }
  }
}
