// 📁 lib/services/preset_service.dart
import 'dart:convert';
import 'package:http/http.dart' as http;

import 'auth_service.dart';

/// 프리셋 저장/확정 시 "최근 수집 데이터 없음" 상황을 구분하기 위한 예외
class PresetSaveException implements Exception {
  /// 'no_raw_posture' | 'unauthorized' | 'unknown'
  final String code;
  final String message;
  PresetSaveException(this.code, this.message);
  @override
  String toString() => message;
}

class PresetService {
  static const String baseUrl = 'https://i13b101.p.ssafy.io/siseon';

  // ────────────────────────────────────────────────────────────────────────────
  // 공통 유틸
  // ────────────────────────────────────────────────────────────────────────────
  static Map<String, String> _headers(String token, {bool json = true}) {
    return {
      'Authorization': 'Bearer $token',
      if (json) 'Content-Type': 'application/json',
      'Accept': '*/*', // HTML 에러 페이지/빈 응답 등도 허용
    };
  }

  static bool _looksNoRawPosture(String body) {
    final b = body.toLowerCase();
    return b.contains('raw_posture') ||
        b.contains('raw posture') ||
        body.contains('raw_posture 데이터가 없습니다') ||
        body.contains('데이터가 없습니다') ||
        body.contains('데이터 없음') ||
        body.contains('자료가 없습니다');
  }

  static Never _throwFromResponse(http.Response res) {
    final text = utf8.decode(res.bodyBytes, allowMalformed: true);

    // 401/403 → 인증 에러
    if (res.statusCode == 401 || res.statusCode == 403) {
      throw PresetSaveException('unauthorized', '로그인이 필요합니다.');
    }

    // 바디 메시지에 raw_posture 단서가 있으면 즉시 매핑
    if (_looksNoRawPosture(text)) {
      throw PresetSaveException(
        'no_raw_posture',
        '최근 수집된 자세 데이터가 아직 없습니다.\n약 10초 후 다시 시도해주세요.',
      );
    }

    // json 포맷으로 감싸진 경우 message/trace 내부도 검사
    try {
      final decoded = jsonDecode(text);
      if (decoded is Map) {
        final msg = (decoded['message'] ?? decoded['error'] ?? '').toString();
        final trace = (decoded['trace'] ?? '').toString();
        if (_looksNoRawPosture(msg) || _looksNoRawPosture(trace)) {
          throw PresetSaveException(
            'no_raw_posture',
            '최근 수집된 자세 데이터가 아직 없습니다.\n약 10초 후 다시 시도해주세요.',
          );
        }
      }
    } catch (_) {
      // JSON 아니면 무시
    }

    // 🔴 마지막 안전장치:
    // preset 저장/확정 요청에서 5xx가 오면 바디가 빈 경우가 많음 → 경로로 보정
    final path = res.request?.url.path ?? '';
    if (res.statusCode >= 500 && path.contains('/preset')) {
      throw PresetSaveException(
        'no_raw_posture',
        '최근 수집된 자세 데이터가 아직 없습니다.\n약 10초 후 다시 시도해주세요.',
      );
    }

    throw PresetSaveException('unknown', '요청 실패 (${res.statusCode})');
  }

  // ────────────────────────────────────────────────────────────────────────────
  // 목록
  // ────────────────────────────────────────────────────────────────────────────
  static Future<List<Map<String, dynamic>>> fetchPresets(int profileId) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return [];

    final url = Uri.parse('$baseUrl/api/preset/profile/$profileId');
    final response = await http.get(url, headers: _headers(token, json: false));

    if (response.statusCode == 200) {
      if (response.bodyBytes.isEmpty) return [];
      final List raw = jsonDecode(utf8.decode(response.bodyBytes)) as List;
      return raw.map<Map<String, dynamic>>((item) {
        final map = Map<String, dynamic>.from(item as Map);
        return {
          'id': map['id'] ?? map['presetId'] ?? map['preset_id'],
          'name': map['name'] ?? '이름 없음',
          'deviceId': map['deviceId'] ?? map['device_id'],
          'position': map['position'] ?? {},
        };
      }).toList();
    }
    return [];
  }

  // ────────────────────────────────────────────────────────────────────────────
  // 생성/확정
  // ────────────────────────────────────────────────────────────────────────────
  static Future<Map<String, dynamic>?> createPreset(
      String name,
      int profileId,
      int deviceId, // 시그니처 호환용(미사용)
      ) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      throw PresetSaveException('unauthorized', '로그인이 필요합니다.');
    }

    final url = Uri.parse('$baseUrl/api/preset');
    final res = await http.post(
      url,
      headers: _headers(token),
      body: jsonEncode({'profileId': profileId, 'name': name}),
    );

    if (res.statusCode >= 200 && res.statusCode < 300) {
      final t = utf8.decode(res.bodyBytes, allowMalformed: true).trim();
      if (t.isEmpty) return {};
      final js = jsonDecode(t);
      return (js is Map<String, dynamic>) ? js : {};
    }

    _throwFromResponse(res);
  }

  /// FCM 'preset_suggest' 전용 저장
  static Future<void> confirm({
    required int profileId,
    String? name,
  }) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      throw PresetSaveException('unauthorized', '로그인이 필요합니다.');
    }

    final headers = _headers(token);
    final payload = jsonEncode({'profileId': profileId, if (name != null) 'name': name});

    // 1) /confirm
    final r1 = await http.post(
      Uri.parse('$baseUrl/api/preset/confirm'),
      headers: headers,
      body: payload,
    );
    if (r1.statusCode >= 200 && r1.statusCode < 300) return;

    // 2) 구버전 서버 → /preset
    if (r1.statusCode == 404 || r1.statusCode == 405) {
      final r2 = await http.post(
        Uri.parse('$baseUrl/api/preset'),
        headers: headers,
        body: payload,
      );
      if (r2.statusCode >= 200 && r2.statusCode < 300) return;
      _throwFromResponse(r2);
    }

    _throwFromResponse(r1);
  }

  // ────────────────────────────────────────────────────────────────────────────
  // 수정/삭제
  // ────────────────────────────────────────────────────────────────────────────
  static Future<bool> updatePresetName(
      int presetId,
      String name,
      int profileId,
      ) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return false;

    final url = Uri.parse('$baseUrl/api/preset/$presetId');
    final res = await http.put(
      url,
      headers: _headers(token),
      body: jsonEncode({'name': name, 'profileId': profileId}),
    );
    return res.statusCode == 200;
  }

  static Future<bool> updatePreset(
      int presetId,
      String name,
      int profileId,
      int deviceId,
      Map<String, dynamic> position,
      ) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return false;

    final url = Uri.parse('$baseUrl/api/preset/$presetId');
    final res = await http.put(
      url,
      headers: _headers(token),
      body: jsonEncode({
        'name': name,
        'profileId': profileId,
        'deviceId': deviceId,
        'position': position,
      }),
    );
    return res.statusCode == 200;
  }

  static Future<bool> deletePreset(int presetId) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) return false;

    final url = Uri.parse('$baseUrl/api/preset/$presetId');
    final res = await http.delete(url, headers: _headers(token, json: false));
    return res.statusCode == 200 || res.statusCode == 204;
  }
}
