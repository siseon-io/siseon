import 'dart:convert';
import 'package:http/http.dart' as http;
import 'auth_service.dart';

class PresetService {
  static const String baseUrl = 'http://i13b101.p.ssafy.io:8080';

  /// ✅ 프리셋 목록 불러오기
  static Future<List<Map<String, dynamic>>> fetchPresets(int profileId) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset/profile/$profileId');

    print('📤 fetchPresets 요청 URL: $url');
    final response = await http.get(
      url,
      headers: {'Authorization': 'Bearer $token'},
    );
    print('📥 상태코드: ${response.statusCode}');
    print('📥 응답 본문: ${utf8.decode(response.bodyBytes)}');

    if (response.statusCode == 200) {
      final List<dynamic> rawList = jsonDecode(utf8.decode(response.bodyBytes));

      // 🔑 서버 응답을 UI에서 일관되게 사용하도록 변환
      final presets = rawList.map((item) {
        final map = Map<String, dynamic>.from(item);
        return {
          'id': map['id'] ?? map['presetId'] ?? map['preset_id'], // ✅ ID 키 매핑
          'name': map['name'] ?? '이름 없음',
          'deviceId': map['deviceId'] ?? map['device_id'],
          'position': map['position'] ?? {},
        };
      }).toList();

      print("✅ 변환된 프리셋 목록: $presets");
      return List<Map<String, dynamic>>.from(presets);
    }
    return [];
  }


  /// ✅ 프리셋 생성 (position 포함)
  static Future<Map<String, dynamic>?> createPreset(
      String name,
      int profileId,
      int deviceId,
      ) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset');
    final body = {
      'profileId': profileId,
      'name': name,
    };

    print('📤 createPreset 요청 URL: $url');
    print('📤 요청 바디: $body');

    final response = await http.post(
      url,
      headers: {
        'Authorization': 'Bearer $token',
        'Content-Type': 'application/json',
      },
      body: jsonEncode(body),
    );

    print('📥 상태코드: ${response.statusCode}');
    print('📥 응답 본문: ${utf8.decode(response.bodyBytes)}');

    if (response.statusCode == 200 || response.statusCode == 201) {
      return jsonDecode(utf8.decode(response.bodyBytes));
    }
    return null;
  }

  /// ✅ 프리셋 수정
  static Future<bool> updatePreset(
      int presetId,
      String name,
      int profileId,
      int deviceId,
      Map<String, dynamic> position,
      ) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset/$presetId');
    final body = {
      'name': name,
      'profileId': profileId,
      'deviceId': deviceId,
      'position': position,
    };

    print('📤 updatePreset 요청 URL: $url');
    print('📤 요청 바디: $body');

    final response = await http.put(
      url,
      headers: {
        'Authorization': 'Bearer $token',
        'Content-Type': 'application/json',
      },
      body: jsonEncode(body),
    );

    print('📥 updatePreset 응답 코드: ${response.statusCode}');
    print('📥 updatePreset 응답 본문: ${utf8.decode(response.bodyBytes)}');

    return response.statusCode == 200;
  }

  /// ✅ 프리셋 삭제
  static Future<bool> deletePreset(int presetId) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset/$presetId');

    print('🗑️ deletePreset 요청 URL: $url');

    final response = await http.delete(
      url,
      headers: {'Authorization': 'Bearer $token'},
    );

    print('📥 deletePreset 응답 코드: ${response.statusCode}');
    print('📥 deletePreset 응답 본문: ${utf8.decode(response.bodyBytes)}');

    return response.statusCode == 200 || response.statusCode == 204;
  }
}
