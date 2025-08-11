// lib/services/faq_service.dart
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';
import 'package:siseon2/services/auth_service.dart';

class Faq {
  final int id;
  final String question;
  final String answer;
  final String? category;
  final List<String>? tags;
  final int displayOrd;
  final DateTime updatedAt;
  final DateTime createdAt;

  Faq({
    required this.id,
    required this.question,
    required this.answer,
    this.category,
    this.tags,
    required this.displayOrd,
    required this.updatedAt,
    required this.createdAt,
  });

  factory Faq.fromJson(Map<String, dynamic> j) => Faq(
    id: j['id'] as int,
    question: j['question'] as String,
    answer: j['answer'] as String,
    category: j['category'] as String?,
    tags: (j['tags'] as List?)?.map((e) => e.toString()).toList(),
    displayOrd: j['display_ord'] as int,
    updatedAt: DateTime.parse(j['updated_at'] as String),
    createdAt: DateTime.parse(j['created_at'] as String),
  );
}

class FaqService {
  static const _base = 'http://i13b101.p.ssafy.io:8000/api';

  // ✅ 캐시 네임스페이스 버전(깨진 캐시 무시)
  static const _ns = 'v2';
  static String _etagKey(String? c, String? q) =>
      'faq_etag_$_ns:${c ?? "all"}:${q ?? ""}';
  static String _cacheKey(String? c, String? q) =>
      'faq_cache_$_ns:${c ?? "all"}:${q ?? ""}';

  // ── UTF-8 안전 파서 ─────────────────────────────────────────────────────────
  static String _utf8Text(http.Response resp) => utf8.decode(resp.bodyBytes);

  static List<dynamic> _decodeList(http.Response resp) {
    final txt = _utf8Text(resp);
    final data = jsonDecode(txt);
    if (data is List) return data;
    throw Exception('Invalid JSON (expected array): $txt');
  }

  static Map<String, dynamic> _decodeMap(http.Response resp) {
    final txt = _utf8Text(resp);
    final data = jsonDecode(txt);
    if (data is Map<String, dynamic>) return data;
    throw Exception('Invalid JSON (expected object): $txt');
  }

  static String _errorText(http.Response resp) {
    final txt = _utf8Text(resp);
    try {
      final j = jsonDecode(txt);
      if (j is Map) {
        if (j['message'] is String) return j['message'] as String;
        if (j['detail'] is String) return j['detail'] as String;
        if (j['error'] is String) return j['error'] as String;
      }
    } catch (_) {}
    return txt;
  }

  // ── API ─────────────────────────────────────────────────────────────────────
  static Future<(List<Faq> list, String? etag, bool fromCache)> getFaqs({
    String? category,
    String? q,
  }) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) throw Exception('로그인 필요');

    final sp = await SharedPreferences.getInstance();
    final prevEtag = sp.getString(_etagKey(category, q));

    final uri = Uri.parse('$_base/faq').replace(queryParameters: {
      if (category != null && category.isNotEmpty) 'category': category,
      if (q != null && q.isNotEmpty) 'q': q,
    });

    final headers = <String, String>{
      'Authorization': 'Bearer $token',
      'Accept': 'application/json',
      'Content-Type': 'application/json; charset=utf-8',
      if (prevEtag != null && prevEtag.isNotEmpty) 'If-None-Match': prevEtag,
    };

    http.Response res = await http.get(uri, headers: headers).timeout(
      const Duration(seconds: 10),
    );

    // 304이면 캐시 사용
    if (res.statusCode == 304) {
      final cached = sp.getString(_cacheKey(category, q));
      if (cached != null) {
        try {
          final list = (jsonDecode(cached) as List)
              .map((e) => Faq.fromJson((e as Map).cast<String, dynamic>()))
              .toList();
          return (list, prevEtag, true);
        } catch (_) {
          // 캐시 손상 → 강제 재요청
        }
      }
      // ETag는 있는데 캐시가 없거나 손상된 경우: If-None-Match 제거 후 다시 요청
      final freshHeaders = Map<String, String>.from(headers)
        ..remove('If-None-Match');
      res = await http.get(uri, headers: freshHeaders).timeout(
        const Duration(seconds: 10),
      );
    }

    if (res.statusCode != 200) {
      throw Exception('FAQ load failed: ${res.statusCode} ${_errorText(res)}');
    }

    final etag = res.headers['etag'];
    final txt = _utf8Text(res); // ✅ 핵심: UTF-8 강제 디코딩
    final list = (jsonDecode(txt) as List)
        .map((e) => Faq.fromJson((e as Map).cast<String, dynamic>()))
        .toList();

    // 캐시 저장(원문 JSON 텍스트로)
    await sp.setString(_etagKey(category, q), etag ?? '');
    await sp.setString(_cacheKey(category, q), txt);

    return (list, etag, false);
  }

  static Future<({String answer, int faqId})> answerFromFaq({
    required int faqId,
    required int profileId,
  }) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) throw Exception('로그인 필요');

    final uri = Uri.parse('$_base/faq/$faqId/answer');
    final res = await http
        .post(
      uri,
      headers: {
        'Authorization': 'Bearer $token',
        'Accept': 'application/json',
        'Content-Type': 'application/json; charset=utf-8',
      },
      body: jsonEncode({"profileId": profileId}),
    )
        .timeout(const Duration(seconds: 10));

    if (res.statusCode != 200) {
      throw Exception(
          'FAQ answer failed: ${res.statusCode} ${_errorText(res)}');
    }

    final j = _decodeMap(res);
    return (
    answer: j['answer'] as String,
    faqId: (j['faq_id'] ?? j['faqId']) as int,
    );
  }
}
