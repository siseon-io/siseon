// lib/services/stats_service.dart
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/services/auth_service.dart';

class StatsService {
  static const _base = 'i13b101.p.ssafy.io:8080';

  /// 기간별 통계 조회
  static Future<List<PostureStats>> fetchPostureStats({
    required int profileId,
    required String period,         // "daily" | "weekly" | "monthly"
    DateTime? from,                 // 옵션
    DateTime? to,                   // 옵션
    int? year,                      // 옵션
  }) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      throw Exception('로그인 토큰 없음');
    }

    final qp = <String, String>{
      'profileId': '$profileId',
      'period': period,
      if (from != null) 'from': from.toIso8601String(),
      if (to != null) 'to': to.toIso8601String(),
      if (year != null) 'year': '$year',
    };

    final uri = Uri.http(_base, '/api/posture-stats', qp);
    print('[StatsService] 요청 URL: $uri');

    final res = await http.get(
      uri,
      headers: {
        'Authorization': 'Bearer $token',
        'Content-Type': 'application/json',
      },
    );

    print('[StatsService] 응답 코드: ${res.statusCode}');
    final previewLen = res.body.length > 500 ? 500 : res.body.length;
    print('[StatsService] 응답 바디(앞 500자): ${res.body.substring(0, previewLen)}');

    if (res.statusCode != 200) {
      throw Exception('통계 불러오기 실패: ${res.statusCode}');
    }

    try {
      final decoded = jsonDecode(res.body);
      if (decoded is! List) {
        throw Exception('예상과 다른 응답 형식: $decoded');
      }

      final list = decoded
          .map((e) => PostureStats.fromJson(e as Map<String, dynamic>))
          .toList();

      print('[StatsService] 파싱 개수: ${list.length}');

      if (list.isNotEmpty) {
        // 시작시간 기준 오름차순 정렬
        list.sort((a, b) => a.startAt.compareTo(b.startAt));
        final months = list
            .map((e) => '${e.startAt.year}-${e.startAt.month.toString().padLeft(2, '0')}')
            .toSet()
            .toList()
          ..sort();
        print('[StatsService] 포함 월: $months');
      }

      return list;
    } catch (e) {
      throw Exception('응답 파싱 실패: $e');
    }
  }

  /// 가장 최신 배치 1건 가져오기
  /// - createdAt 없이 endAt(또는 startAt)로 최신성 판정
  static Future<PostureStats?> fetchLatestPosture({
    required int profileId,
    Duration lookback = const Duration(hours: 12),
  }) async {
    final now = DateTime.now();
    final from = now.subtract(lookback);

    final list = await fetchPostureStats(
      profileId: profileId,
      period: 'daily',
      from: from,
      to: now,
    );

    if (list.isEmpty) return null;

    // 최신 순(내림차순): endAt > startAt
    list.sort((a, b) => _bestTimestamp(b).compareTo(_bestTimestamp(a)));

    final latest = list.first;
    print('[StatsService] 최신 슬롯: slotIndex=${latest.slotIndex} @ ${_bestTimestamp(latest).toIso8601String()}');
    return latest;
  }

  // createdAt이 없으므로 endAt 우선, 없으면 startAt
  static DateTime _bestTimestamp(PostureStats s) => s.endAt; // startAt만 있다면 s.startAt로 바꿔도 OK
}
