// lib/services/auth_service.dart
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:http/http.dart' as http;
import 'package:jwt_decoder/jwt_decoder.dart';

class AuthService {
  static const String _baseUrl = 'http://i13b101.p.ssafy.io:8080';
  static const String _refreshEndpoint = '$_baseUrl/api/auth/refresh';

  static const _kAccess = 'accessToken';
  static const _kRefresh = 'refreshToken';

  // ── Storage ────────────────────────────────────────────────────────────────
  static Future<void> _saveTokens(String access, String refresh) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_kAccess, access);
    await prefs.setString(_kRefresh, refresh);
    final exp = _safeExp(access);
    // 디버그: 새 토큰 만료시간
    // ignore: avoid_print
    print('[AuthService] saved access exp=$exp');
  }

  static Future<String?> getAccessToken() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString(_kAccess);
  }

  static Future<String?> getRefreshToken() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString(_kRefresh);
  }

  static Future<void> clearTokens() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove(_kAccess);
    await prefs.remove(_kRefresh);
  }

  static Future<void> logout() => clearTokens();

  // ── Helpers ────────────────────────────────────────────────────────────────
  static DateTime? _safeExp(String token) {
    try {
      final payload = JwtDecoder.decode(token);
      final exp = payload['exp'];
      if (exp is int) {
        return DateTime.fromMillisecondsSinceEpoch(exp * 1000);
      } else if (exp is num) {
        return DateTime.fromMillisecondsSinceEpoch((exp * 1000).toInt());
      }
    } catch (_) {}
    return null;
  }

  static bool _isExpired(String token, {Duration skew = const Duration(seconds: 30)}) {
    final exp = _safeExp(token);
    if (exp == null) return true;
    return DateTime.now().isAfter(exp.subtract(skew));
  }

  // ── Public API ─────────────────────────────────────────────────────────────
  /// 유효한 accessToken (만료면 자동 리프레시)
  static Future<String?> getValidAccessToken() async {
    var at = await getAccessToken();
    if (at != null && !_isExpired(at)) return at;
    return await refreshAccessToken();
  }
// ✅ 기존 화면들과 호환용 공개 메서드 복구
  static Future<void> saveTokens(String accessToken, String refreshToken) {
    return _saveTokens(accessToken, refreshToken);
  }

// ✅ (선택) 로그인 응답 JSON 바로 넣는 버전
  static Future<void> saveTokensFromResponse(Map<String, dynamic> data) {
    final access = (data['accessToken'] ?? data['access_token'] ?? data['access']) as String?;
    final refresh = (data['refreshToken'] ?? data['refresh_token'] ?? data['refresh']) as String?;
    if (access == null || refresh == null) {
      throw Exception('토큰이 응답에 없습니다: $data');
    }
    return _saveTokens(access, refresh);
  }

  /// 리프레시로 재발급
  static Future<String?> refreshAccessToken() async {
    final rt = await getRefreshToken();
    if (rt == null) {
      // ignore: avoid_print
      print('[AuthService] no refreshToken');
      return null;
    }

    final resp = await http.post(
      Uri.parse(_refreshEndpoint),
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'refreshToken': rt}), // ※ 서버 스펙이 다르면 키 이름 맞춰줘
    );

    if (resp.statusCode != 200) {
      // ignore: avoid_print
      print('[AuthService] refresh failed: ${resp.statusCode} ${resp.body}');
      return null;
    }

    final Map<String, dynamic> data = jsonDecode(resp.body);

    // 서버가 어떤 키를 쓰든 유연하게 파싱
    final newAccess = (data['accessToken'] ?? data['access_token'] ?? data['access']) as String?;
    final newRefresh = (data['refreshToken'] ?? data['refresh_token'] ?? data['refresh']) as String? ?? rt;

    if (newAccess == null) {
      // ignore: avoid_print
      print('[AuthService] refresh OK but access token missing. body=$data');
      return null;
    }

    // 저장 & 만료 확인 로그
    await _saveTokens(newAccess, newRefresh);
    final exp = _safeExp(newAccess);
    // ignore: avoid_print
    print('[AuthService] got new access. exp=$exp');

    return newAccess;
  }

  /// 로그인 여부
  static Future<bool> isLoggedIn() async {
    final at = await getAccessToken();
    return at != null && !_isExpired(at);
  }
}
