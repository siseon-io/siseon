// lib/services/stats_service.dart
import 'dart:convert';
import 'package:http/http.dart' as http;

import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/services/auth_service.dart';

class StatsService {
  // ✅ 새 베이스 (HTTPS + /siseon 프리픽스)
  static const String _base = 'https://i13b101.p.ssafy.io/siseon';
  static const String _path = '/api/posture-stats';

  static Uri _buildUri(Map<String, String> qp) =>
      Uri.parse('$_base$_path').replace(queryParameters: qp);

  // ── 1) Day 집계 (period=day) ────────────────────────────────────────────────
  static Future<List<PostureStatsDay>> fetchDayStats({
    required int profileId,
    DateTime? from,
    DateTime? to,
  }) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) throw Exception('로그인 토큰 없음');

    final qp = <String, String>{
      'profileId': '$profileId',
      'period': 'day',
      if (from != null) 'from': from.toIso8601String(),
      if (to != null) 'to': to.toIso8601String(),
    };

    final uri = _buildUri(qp);
    print('[StatsService][DAY] GET $uri');

    final res = await http.get(uri, headers: {
      'Authorization': 'Bearer $token',
      'Accept': 'application/json',
    });

    print('[StatsService][DAY] status=${res.statusCode}');
    print('[StatsService][DAY] body<= ${res.body.substring(0, res.body.length.clamp(0, 500))}');

    if (res.statusCode != 200) {
      throw Exception('Day 통계 불러오기 실패: ${res.statusCode}');
    }

    final decoded = jsonDecode(res.body);
    if (decoded is! List) throw Exception('Day 응답 형식 오류: $decoded');

    final items = decoded
        .map((e) => PostureStatsDay.fromJson(e as Map<String, dynamic>))
        .toList()
      ..sort((a, b) => a.statDate.compareTo(b.statDate));

    return items;
  }

  // ── 2) Minute 원본 (period=daily|weekly|monthly) ────────────────────────────
  static Future<List<PostureStatsMinute>> fetchMinuteStats({
    required int profileId,
    required String period, // 'daily' | 'weekly' | 'monthly'
    DateTime? from,
    DateTime? to,
  }) async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) throw Exception('로그인 토큰 없음');

    final qp = <String, String>{
      'profileId': '$profileId',
      'period': period.toLowerCase(),
      if (from != null) 'from': from.toIso8601String(),
      if (to != null) 'to': to.toIso8601String(),
    };

    final uri = _buildUri(qp);
    print('[StatsService][MIN] GET $uri');

    final res = await http.get(uri, headers: {
      'Authorization': 'Bearer $token',
      'Accept': 'application/json',
    });

    print('[StatsService][MIN] status=${res.statusCode}');
    print('[StatsService][MIN] body<= ${res.body.substring(0, res.body.length.clamp(0, 500))}');

    if (res.statusCode != 200) {
      throw Exception('Minute 통계 불러오기 실패: ${res.statusCode}');
    }

    final decoded = jsonDecode(res.body);
    if (decoded is! List) throw Exception('Minute 응답 형식 오류: $decoded');

    final list = decoded
        .map((e) => PostureStatsMinute.fromJson(e as Map<String, dynamic>))
        .toList()
      ..sort((a, b) => a.startAt.compareTo(b.startAt));

    if (list.isNotEmpty) {
      final months = list
          .map((e) => '${e.startAt.year}-${e.startAt.month.toString().padLeft(2, '0')}')
          .toSet()
          .toList()
        ..sort();
      print('[StatsService][MIN] 포함 월: $months');
    }
    return list;
  }

  // ── 3) 하위호환 래퍼( minute 전용 ) ─────────────────────────────────────────
  static Future<List<PostureStatsMinute>> fetchPostureStats({
    required int profileId,
    required String period, // "daily" | "weekly" | "monthly"
    DateTime? from,
    DateTime? to,
    int? year, // 사용 안 함
  }) async {
    if (period.toLowerCase() == 'day') {
      throw Exception('fetchPostureStats(day)는 지원하지 않습니다. fetchDayStats()를 사용하세요.');
    }
    return fetchMinuteStats(
      profileId: profileId,
      period: period,
      from: from,
      to: to,
    );
  }

  // ── 4) 최신 슬롯 1건 (minute 기준) ─────────────────────────────────────────
  static Future<PostureStatsMinute?> fetchLatestPosture({
    required int profileId,
    Duration lookback = const Duration(hours: 12),
  }) async {
    final now = DateTime.now();
    final from = now.subtract(lookback);

    final list = await fetchMinuteStats(
      profileId: profileId,
      period: 'daily',
      from: from,
      to: now,
    );
    if (list.isEmpty) return null;

    list.sort((a, b) => b.endAt.compareTo(a.endAt));
    final latest = list.first;

    print('[StatsService] 최신 슬롯: slotIndex=${latest.slotIndex} @ ${latest.endAt.toIso8601String()}');
    return latest;
  }
}
