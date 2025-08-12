// lib/services/chat_api.dart
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/models/chat_models.dart';

class ChatApi {
  // 백엔드 Base URL
  static const String _baseUrl = 'https://i13b101.p.ssafy.io/ai/api';

  // 401이면 refresh 후 1회 재시도
  Future<http.Response> _withAuthRetry(
      Future<http.Response> Function(String token) doRequest,
      ) async {
    String? token = await AuthService.getValidAccessToken();
    if (token == null) throw Exception('로그인 필요: accessToken 없음');
    token = token.trim();

    http.Response resp = await doRequest(token);

    if (resp.statusCode == 401) {
      final newToken = await AuthService.refreshAccessToken();
      if (newToken == null) {
        throw Exception('로그인 만료: 다시 로그인해 주세요.');
      }
      resp = await doRequest(newToken.trim());
    }
    return resp;
  }

  Map<String, String> _headers(String token) => {
    'Authorization': 'Bearer $token',
    'Content-Type': 'application/json; charset=utf-8',
    'Accept': 'application/json',
  };

  // ---- 유틸: UTF-8로 안전 파싱 ------------------------------------------------
  Map<String, dynamic> _decodeMap(http.Response resp) {
    final txt = utf8.decode(resp.bodyBytes); // ✅ 한글 깨짐 방지
    final data = jsonDecode(txt);
    if (data is Map<String, dynamic>) return data;
    throw Exception('Invalid JSON (expected object): $txt');
  }

  List<dynamic> _decodeList(http.Response resp) {
    final txt = utf8.decode(resp.bodyBytes); // ✅ 한글 깨짐 방지
    final data = jsonDecode(txt);
    if (data is List) return data;
    throw Exception('Invalid JSON (expected array): $txt');
  }
  // ---------------------------------------------------------------------------

  // 질문 전송
  Future<ChatResponse> sendQuestion({
    required int profileId,
    required String question,
  }) async {
    final uri = Uri.parse('$_baseUrl/chat');

    final resp = await _withAuthRetry(
          (t) => http.post(
        uri,
        headers: _headers(t),
        body: jsonEncode({'profileId': profileId, 'question': question}),
      ),
    );

    if (resp.statusCode != 200) {
      // 서버 메시지 보여주도록 그대로 포함
      throw Exception('Chat failed: ${resp.statusCode} ${resp.reasonPhrase} ${utf8.decode(resp.bodyBytes)}');
    }

    final map = _decodeMap(resp);
    return ChatResponse.fromJson(map);
  }

  // 과거 대화 불러오기
  Future<List<ChatMessage>> fetchHistory({
    required int profileId,
    int limit = 200,
  }) async {
    final uri = Uri.parse('$_baseUrl/chat/history')
        .replace(queryParameters: {'profileId': '$profileId', 'limit': '$limit'});

    final resp = await _withAuthRetry(
          (t) => http.get(uri, headers: _headers(t)),
    );

    if (resp.statusCode != 200) {
      throw Exception('History failed: ${resp.statusCode} ${resp.reasonPhrase} ${utf8.decode(resp.bodyBytes)}');
    }

    final list = _decodeList(resp)
        .cast<Map<String, dynamic>>()
        .map(ChatMessage.fromJson)
        .toList();

    return list;
  }
}
