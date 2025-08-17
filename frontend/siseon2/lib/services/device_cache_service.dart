// lib/services/device_cache_service.dart
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;

import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';

class DeviceCacheService {
  static const _baseUrl = 'https://i13b101.p.ssafy.io/siseon';

  // 신규(권장): 프로필별 단일 JSON 키
  static String _jsonKey(int pid) => 'device_profile_$pid';

  // 레거시(이전 코드 호환/정리용): 프로필별 분리 키
  static String _legacySerialKey(int pid) => 'deviceSerial_$pid';
  static String _legacyRegKey(int pid) => 'isDeviceRegistered_$pid';

  // 더 오래된 글로벌 키(혹시 남아있으면 혼동 방지용으로 제거)
  static const _legacyGlobalSerial = 'deviceSerial';
  static const _legacyGlobalReg = 'isDeviceRegistered';

  // 현재 선택된 프로필 ID
  static Future<int?> _currentProfileId() async {
    final p = await ProfileCacheService.loadProfile();
    return (p?['profileId'] ?? p?['id']) as int?;
  }

  // ───────────────────────────────────────────────────────────────────────────
  // 서버에서 기기 조회 → 해당 프로필로 캐싱
  // ───────────────────────────────────────────────────────────────────────────
  static Future<void> fetchAndCacheDevice({int? profileId}) async {
    final pid = profileId ?? await _currentProfileId();
    if (pid == null) throw Exception('⚠️ 프로필 정보를 불러올 수 없습니다.');

    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$_baseUrl/api/device/profile/$pid');

    try {
      final res = await http.get(url, headers: {
        'Content-Type': 'application/json',
        'Authorization': 'Bearer $token',
      });

      // 404/204 => 등록 없음
      if (res.statusCode == 404 || res.statusCode == 204) {
        await clearDeviceForProfile(pid);
        return;
      }

      if (res.statusCode != 200) {
        // 네트워크/서버 일시 오류: 캐시 유지
        return;
      }

      if (res.body.isEmpty) {
        await clearDeviceForProfile(pid);
        return;
      }

      final body = jsonDecode(res.body);

      // 응답이 배열/객체 등 다양한 케이스를 커버
      dynamic deviceData;
      if (body is List) {
        deviceData = body.isNotEmpty ? body.first : null;
      } else if (body is Map && body.containsKey('serials')) {
        final serials = body['serials'] as List<dynamic>;
        if (serials.isNotEmpty) {
          await saveDeviceForProfile(pid, {
            'serial': serials.first.toString(),
          });
          return;
        }
        deviceData = null;
      } else {
        deviceData = body; // 일반 단일 객체
      }

      String? serial;
      String? charUuid;
      if (deviceData is Map) {
        serial = (deviceData['serial'] ??
            deviceData['serialNumber'] ??
            deviceData['deviceSerial'])
            ?.toString();
        charUuid = (deviceData['targetCharUuid'] ??
            deviceData['charUuid'] ??
            deviceData['characteristicUuid'])
            ?.toString();
      }

      if (serial != null && serial.isNotEmpty) {
        await saveDeviceForProfile(pid, {
          'serial': serial,
          if (charUuid != null && charUuid.isNotEmpty) 'targetCharUuid': charUuid,
        });
      } else {
        await clearDeviceForProfile(pid);
      }
    } catch (_) {
      // 네트워크/파싱 오류 시 캐시 건드리지 않음
    }
  }

  // ───────────────────────────────────────────────────────────────────────────
  // 등록 직후 UX: 로컬 저장 → 서버 확인은 비동기
  // ───────────────────────────────────────────────────────────────────────────
  static Future<void> saveDeviceSerialLocal({
    int? profileId,
    required String serial,
    String? targetCharUuid,
  }) async {
    final pid = profileId ?? await _currentProfileId();
    if (pid == null) throw Exception('⚠️ 프로필 정보를 불러올 수 없습니다.');

    await saveDeviceForProfile(pid, {
      'serial': serial,
      if (targetCharUuid != null && targetCharUuid.isNotEmpty)
        'targetCharUuid': targetCharUuid,
    });

    // 서버 데이터 동기화(결과는 기다리지 않음)
    // ignore: unawaited_futures
    fetchAndCacheDevice(profileId: pid);
  }

  // ───────────────────────────────────────────────────────────────────────────
  // 프로필별 저장 / 조회 / 삭제
  // ───────────────────────────────────────────────────────────────────────────
  static Future<void> saveDeviceForProfile(
      int profileId,
      Map<String, dynamic> device,
      ) async {
    final prefs = await SharedPreferences.getInstance();

    final serial = (device['serial'] ?? device['serialNumber'])?.toString();
    if (serial == null || serial.isEmpty) {
      throw ArgumentError('serial(혹은 serialNumber)이 없습니다.');
    }

    // 기존 JSON 값을 머지(누락 필드 보존)
    final old = await loadDeviceForProfile(profileId);
    final merged = {
      ...?old,
      ...device,
      'serial': serial,
      'profileId': profileId,
      'isRegistered': true,
      'savedAt': DateTime.now().toIso8601String(),
    };

    await prefs.setString(_jsonKey(profileId), jsonEncode(merged));

    // 레거시 키는 더 이상 쓰지 않으므로 정리
    await _clearLegacyForProfile(profileId, prefs);
    await _clearLegacyGlobals(prefs);
  }

  static Future<Map<String, dynamic>?> loadDeviceForProfile(
      int profileId,
      ) async {
    final prefs = await SharedPreferences.getInstance();

    // 1) 신규 JSON 키 우선
    final js = prefs.getString(_jsonKey(profileId));
    if (js != null) {
      try {
        final m = jsonDecode(js);
        if (m is Map<String, dynamic>) {
          // 소유자 불일치 방지
          final owner = m['profileId'];
          if (owner == null || owner == profileId) return m;
        }
      } catch (_) {}
    }

    // 2) 레거시 분리 키가 남아있다면 마이그레이션
    final migrated = await _migrateLegacyIfAny(profileId, prefs);
    if (migrated != null) return migrated;

    return null;
  }

  static Future<void> clearDeviceForProfile(int profileId) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove(_jsonKey(profileId));

    // 레거시 키도 함께 정리(+혹시 전역 키를 참조하는 화면 대비 초기화)
    await _clearLegacyForProfile(profileId, prefs);
    await _clearLegacyGlobals(prefs);
  }

  // 현재 선택 프로필 기준 헬퍼
  static Future<Map<String, dynamic>?> loadDevice() async {
    final pid = await _currentProfileId();
    if (pid == null) return null;
    return loadDeviceForProfile(pid);
  }

  // 모든 프로필의 디바이스 캐시 삭제
  static Future<void> clearDevice() async {
    final prefs = await SharedPreferences.getInstance();
    final keys = prefs.getKeys().toList();

    for (final k in keys) {
      if (k.startsWith('device_profile_')) {
        await prefs.remove(k);
      } else if (k.startsWith('deviceSerial_') ||
          k.startsWith('isDeviceRegistered_')) {
        await prefs.remove(k);
      }
    }
    await _clearLegacyGlobals(prefs);
  }

  // targetCharUuid만 갱신하고 싶을 때
  static Future<void> setTargetCharUuid(String targetCharUuid,
      {int? profileId}) async {
    final pid = profileId ?? await _currentProfileId();
    if (pid == null) return;

    final prefs = await SharedPreferences.getInstance();
    final js = prefs.getString(_jsonKey(pid));
    Map<String, dynamic> obj = {};
    if (js != null) {
      try {
        final m = jsonDecode(js);
        if (m is Map<String, dynamic>) obj = m;
      } catch (_) {}
    }
    obj['targetCharUuid'] = targetCharUuid;
    obj['profileId'] = pid;
    obj['isRegistered'] = true;
    obj['savedAt'] = DateTime.now().toIso8601String();

    await prefs.setString(_jsonKey(pid), jsonEncode(obj));
    await _clearLegacyForProfile(pid, prefs);
  }

  // ───────────────────────────────────────────────────────────────────────────
  // 내부: 레거시 키 처리
  // ───────────────────────────────────────────────────────────────────────────
  static Future<Map<String, dynamic>?> _migrateLegacyIfAny(
      int profileId,
      SharedPreferences prefs,
      ) async {
    final serial = prefs.getString(_legacySerialKey(profileId));
    final reg = prefs.getBool(_legacyRegKey(profileId)) ?? false;

    if (reg && serial != null && serial.isNotEmpty) {
      final migrated = {
        'profileId': profileId,
        'serial': serial,
        'isRegistered': true,
        'savedAt': DateTime.now().toIso8601String(),
      };
      await prefs.setString(_jsonKey(profileId), jsonEncode(migrated));
      await _clearLegacyForProfile(profileId, prefs);
      await _clearLegacyGlobals(prefs);
      return migrated;
    }

    // 레거시 값이 ‘미등록’이면 깨끗이 정리
    if (!reg) {
      await _clearLegacyForProfile(profileId, prefs);
      await _clearLegacyGlobals(prefs);
    }

    return null;
  }

  static Future<void> _clearLegacyForProfile(
      int profileId,
      SharedPreferences prefs,
      ) async {
    await prefs.remove(_legacySerialKey(profileId));
    await prefs.remove(_legacyRegKey(profileId));
  }

  static Future<void> _clearLegacyGlobals(SharedPreferences prefs) async {
    await prefs.remove(_legacyGlobalSerial);
    await prefs.remove(_legacyGlobalReg);
  }
}
