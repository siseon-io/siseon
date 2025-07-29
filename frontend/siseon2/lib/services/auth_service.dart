import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:jwt_decoder/jwt_decoder.dart';

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

  /// ✅ 유효한 accessToken 반환 (없거나 만료되면 자동 갱신 시도)
  static Future<String?> getValidAccessToken() async {
    String? accessToken = await getAccessToken();

    if (accessToken != null && !JwtDecoder.isExpired(accessToken)) {
      return accessToken;
    }

    // 🔄 토큰이 없거나 만료됐다면 refresh 시도
    return await refreshAccessToken();
  }

  /// ♻️ accessToken 재발급 요청 (refreshToken을 JSON body로 전송)
  static Future<String?> refreshAccessToken() async {
    final refreshToken = await getRefreshToken();
    if (refreshToken == null) return null;

    final response = await http.post(
      Uri.parse(_refreshEndpoint),
      headers: {
        'Content-Type': 'application/json',
      },
      body: jsonEncode({
        'refreshToken': refreshToken,
      }),
    );

    if (response.statusCode == 200) {
      final data = jsonDecode(response.body);
      final newAccessToken = data['accessToken'];
      final newRefreshToken = data['refreshToken'];

      await saveTokens(newAccessToken, newRefreshToken);
      return newAccessToken;
    } else {
      print('🔴 토큰 재발급 실패: ${response.statusCode}, ${response.body}');
      return null;
    }
  }

  /// 🚪 로그아웃 (토큰 제거)
  static Future<void> logout() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove('accessToken');
    await prefs.remove('refreshToken');
  }
  /// 🚮 저장된 토큰 전체 삭제 (회원 탈퇴 시 사용)
  static Future<void> clearTokens() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove('accessToken');
    await prefs.remove('refreshToken');
  }
  /// ✅ 로그인 상태 여부 확인
  static Future<bool> isLoggedIn() async {
    final accessToken = await getAccessToken();
    if (accessToken == null) return false;
    return !JwtDecoder.isExpired(accessToken);
  }
}
