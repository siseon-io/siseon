import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:jwt_decoder/jwt_decoder.dart'; // ⬅️ pubspec.yaml에 추가 필요

class AuthService {
  static const String _baseUrl = 'http://i13b101.p.ssafy.io:8080';
  static const String _refreshEndpoint = '$_baseUrl/api/auth/refresh';

  /// 🔑 accessToken 불러오기
  static Future<String?> getAccessToken() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString('accessToken');
  }

  /// 🔑 refreshToken 불러오기
  static Future<String?> getRefreshToken() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString('refreshToken');
  }

  /// 💾 토큰 저장
  static Future<void> saveTokens(String accessToken, String refreshToken) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('accessToken', accessToken);
    await prefs.setString('refreshToken', refreshToken);
  }

  /// ✅ 유효한 accessToken 반환 (없거나 만료됐으면 자동 갱신 시도)

  static Future<String?> getValidAccessToken() async {
    String? accessToken = await getAccessToken();

    if (accessToken == null) return null;

    // ✅ 토큰 만료 여부 확인
    bool isExpired = JwtDecoder.isExpired(accessToken);
    if (!isExpired) return accessToken;

    // 🔄 만료됐으면 refresh 시도
    final newToken = await refreshAccessToken();
    return newToken; // null이면 로그인 다시 해야 함
  }

  /// ♻️ accessToken 재발급 요청 (refreshToken 사용)
  static Future<String?> refreshAccessToken() async {
    final refreshToken = await getRefreshToken();
    if (refreshToken == null) return null;

    final response = await http.post(
      Uri.parse(_refreshEndpoint),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'refreshToken': refreshToken}),
    );

    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      final newAccessToken = data['accessToken'];
      final newRefreshToken = data['refreshToken'];
      await saveTokens(newAccessToken, newRefreshToken);
      return newAccessToken;
    } else {
      return null;
    }
  }

  /// 🚪 로그아웃 (토큰 제거)
  static Future<void> logout() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove('accessToken');
    await prefs.remove('refreshToken');
  }

  /// ✅ 로그인 상태 여부
  static Future<bool> isLoggedIn() async {
    final token = await getAccessToken();
    return token != null;
  }
}
