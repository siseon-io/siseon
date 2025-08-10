import 'dart:convert';
import 'package:http/http.dart' as http;
import 'auth_service.dart';

class PresetService {
  static const String baseUrl = 'http://i13b101.p.ssafy.io:8080';

  /// 프리셋 목록 불러오기
  static Future<List<Map<String, dynamic>>> fetchPresets(int profileId) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset/profile/$profileId');

    final response = await http.get(
      url,
      headers: {'Authorization': 'Bearer $token'},
    );

    if (response.statusCode == 200) {
      final List<dynamic> rawList = jsonDecode(utf8.decode(response.bodyBytes));
      final presets = rawList.map((item) {
        final map = Map<String, dynamic>.from(item);
        return {
          'id': map['id'] ?? map['presetId'] ?? map['preset_id'],
          'name': map['name'] ?? '이름 없음',
          'deviceId': map['deviceId'] ?? map['device_id'],
          'position': map['position'] ?? {},
        };
      }).toList();
      return List<Map<String, dynamic>>.from(presets);
    }
    return [];
  }

  /// 프리셋 생성 (deviceId는 서버에서 안 쓰면 무시)
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

    final response = await http.post(
      url,
      headers: {
        'Authorization': 'Bearer $token',
        'Content-Type': 'application/json',
      },
      body: jsonEncode(body),
    );

    if (response.statusCode == 200 || response.statusCode == 201) {
      return jsonDecode(utf8.decode(response.bodyBytes));
    }
    return null;
  }

  /// 프리셋 이름만 수정
  static Future<bool> updatePresetName(
      int presetId,
      String name,
      int profileId,
      ) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset/$presetId');
    final body = {
      'name': name,
      'profileId': profileId,
    };

    final response = await http.put(
      url,
      headers: {
        'Authorization': 'Bearer $token',
        'Content-Type': 'application/json',
      },
      body: jsonEncode(body),
    );

    return response.statusCode == 200;
  }

  /// 프리셋 전체 수정 (필요 시 사용)
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

    final response = await http.put(
      url,
      headers: {
        'Authorization': 'Bearer $token',
        'Content-Type': 'application/json',
      },
      body: jsonEncode(body),
    );

    return response.statusCode == 200;
  }

  /// 프리셋 삭제
  static Future<bool> deletePreset(int presetId) async {
    final token = await AuthService.getValidAccessToken();
    final url = Uri.parse('$baseUrl/api/preset/$presetId');

    final response = await http.delete(
      url,
      headers: {'Authorization': 'Bearer $token'},
    );

    return response.statusCode == 200 || response.statusCode == 204;
  }
}
