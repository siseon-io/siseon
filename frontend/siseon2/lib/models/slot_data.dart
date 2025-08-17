// lib/models/slot_data.dart

/// ─────────────────────────────────────────────────────────────────────────────
/// Day 집계 응답 모델 (period=day)
///   [
///     { "profileId": 89, "statDate": "2025-08-01",
///       "goodCount": 120, "badCount": 30, "totalCount": 150 },
///     ...
///   ]
/// ─────────────────────────────────────────────────────────────────────────────
class PostureStatsDay {
  final int profileId;
  final DateTime statDate; // 날짜만 오지만 DateTime 파싱 OK
  final int goodCount;
  final int badCount;
  final int totalCount;

  PostureStatsDay({
    required this.profileId,
    required this.statDate,
    required this.goodCount,
    required this.badCount,
    required this.totalCount,
  });

  factory PostureStatsDay.fromJson(Map<String, dynamic> j) => PostureStatsDay(
    profileId: _asInt(j['profileId'] ?? j['profile_id']),
    statDate: _asDate(j['statDate'] ?? j['stat_date']),
    goodCount: _asInt(j['goodCount'] ?? j['good_count']),
    badCount: _asInt(j['badCount'] ?? j['bad_count']),
    totalCount: _asInt(j['totalCount'] ?? j['total_count']),
  );
}

/// ─────────────────────────────────────────────────────────────────────────────
/// Minute 원본 응답 모델 (period=daily|weekly|monthly)
///   [ { id, profileId, monitorCoord?, userCoord?, startAt, endAt,
///       durationSeconds, slotIndex, validPosture?|valid?, badReasons?, summary? }, ... ]
/// ─────────────────────────────────────────────────────────────────────────────

/// 🔹 나쁜자세 세부 사유 (배너/조언 표시용)
class BadReason {
  final String? label;
  final String? code;
  final String? cue;
  final String? ergonomics;
  final double? angle;
  final double? threshold;
  final String? severity;
  final String? direction;

  BadReason({
    this.label,
    this.code,
    this.cue,
    this.ergonomics,
    this.angle,
    this.threshold,
    this.severity,
    this.direction,
  });

  factory BadReason.fromJson(Map<String, dynamic> j) => BadReason(
    label: j['label'] as String?,
    code: j['code'] as String?,
    cue: j['cue'] as String?,
    ergonomics: j['ergonomics'] as String?,
    angle: (j['angle'] as num?)?.toDouble(),
    threshold: (j['threshold'] as num?)?.toDouble(),
    severity: j['severity'] as String?,
    direction: j['direction'] as String?,
  );
}

/// 🔹 나쁜자세 묶음 (valid + reasons + summary)
class BadReasons {
  final bool? valid; // 서버가 여기에도 valid를 내려줌 (예: false)
  final List<BadReason> reasons;
  final String? summary; // 예: "거북목(148.8°), …"

  BadReasons({
    this.valid,
    required this.reasons,
    this.summary,
  });

  factory BadReasons.fromJson(Map<String, dynamic> j) => BadReasons(
    valid: _asBoolOrNull(j['valid']),
    summary: j['summary'] as String?,
    reasons: (j['reasons'] as List<dynamic>? ?? [])
        .map((e) => BadReason.fromJson(e as Map<String, dynamic>))
        .toList(),
  );
}

class PostureStatsMinute {
  final int id;
  final int profileId;

  /// 백엔드에서 null 가능
  final Map<String, dynamic>? monitorCoord;
  final Map<String, dynamic>? userCoord;

  final DateTime startAt;
  final DateTime endAt;
  final int durationSeconds; // 보통 60
  final int slotIndex; // 10분 단위 인덱스 등

  /// 서버가 `validPosture` 또는 `valid`로 줄 수 있어 모두 허용
  final bool? validPosture;

  /// 🔹 추가: 나쁜자세 상세
  final BadReasons? badReasons;

  /// 🔹 선택: top-level summary (혹시 이 위치로도 내려오면 사용)
  final String? summary;

  PostureStatsMinute({
    required this.id,
    required this.profileId,
    required this.monitorCoord,
    required this.userCoord,
    required this.startAt,
    required this.endAt,
    required this.durationSeconds,
    required this.slotIndex,
    required this.validPosture,
    this.badReasons,
    this.summary,
  });

  factory PostureStatsMinute.fromJson(Map<String, dynamic> j) => PostureStatsMinute(
    id: _asInt(j['id']),
    profileId: _asInt(j['profileId'] ?? j['profile_id']),
    monitorCoord: _asMapOrNull(j['monitorCoord'] ?? j['monitor_coord']),
    userCoord: _asMapOrNull(j['userCoord'] ?? j['user_coord']),
    startAt: _asDate(j['startAt'] ?? j['start_at']),
    endAt: _asDate(j['endAt'] ?? j['end_at']),
    durationSeconds:
    _asInt(j['durationSeconds'] ?? j['duration_seconds'] ?? j['duration']),
    slotIndex: _asInt(j['slotIndex'] ?? j['slot_index'] ?? j['slot']),
    // valid: 서버가 validPosture 또는 valid 로 줄 수 있음
    validPosture: _asBoolOrNull(j['validPosture'] ?? j['valid_posture'] ?? j['valid']),
    // badReasons: camelCase / snake_case 모두 대응
    badReasons: (() {
      final raw = j['badReasons'] ?? j['bad_reasons'];
      if (raw is Map<String, dynamic>) return BadReasons.fromJson(raw);
      if (raw is Map) return BadReasons.fromJson(Map<String, dynamic>.from(raw));
      return null;
    })(),
    // 혹시 top-level 로 summary가 오면 받아둠
    summary: j['summary'] as String?,
  );
}

/// ─────────────────────────────────────────────────────────────────────────────
/// ✅ 기존 코드 호환용 별칭 (이전 PostureStats == minute 원본)
///    기존에 PostureStats를 쓰던 곳은 그대로 컴파일됩니다.
/// ─────────────────────────────────────────────────────────────────────────────
typedef PostureStats = PostureStatsMinute;

/// ─────────────────────────────────────────────────────────────────────────────
/// 내부 유틸
/// ─────────────────────────────────────────────────────────────────────────────
int _asInt(dynamic v) {
  if (v == null) throw ArgumentError('null int field');
  if (v is int) return v;
  return int.parse(v.toString());
}

DateTime _asDate(dynamic v) {
  if (v == null) throw ArgumentError('null date field');
  if (v is DateTime) return v;
  return DateTime.parse(v.toString());
}

bool? _asBoolOrNull(dynamic v) {
  if (v == null) return null;
  if (v is bool) return v;
  final s = v.toString().toLowerCase();
  if (s == 'true' || s == '1') return true;
  if (s == 'false' || s == '0') return false;
  return null; // 알 수 없는 포맷은 null
}

Map<String, dynamic>? _asMapOrNull(dynamic v) {
  if (v == null) return null;
  if (v is Map<String, dynamic>) return v;
  if (v is Map) return Map<String, dynamic>.from(v);
  // 문자열로 온 경우(예: JSON 스트링)는 프론트에서 굳이 디코딩 안 함
  return null;
}
