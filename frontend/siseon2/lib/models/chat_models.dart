// lib/models/chat_models.dart
import 'dart:convert';

/// ─────────────────────────────────────────────────────────────────────────────
/// 시간 파서 정책
/// - 서버가 "YYYY-MM-DDTHH:mm:ss(.SSS)" 같은 '오프셋 없는 로컬'을 준다고 가정하되,
///   혹시 Z(UTC)나 ±HH:MM 오프셋이 섞여 와도 안전하게 처리한다.
/// - 내부 보관값은 항상 '로컬 시각(DateTime, isUtc=false)'로 통일.
/// - 서버로 보낼 때도 '오프셋 없는 로컬 ISO'로 준다(서버 기대값 맞춤).
/// ─────────────────────────────────────────────────────────────────────────────

DateTime _parseServerTime(dynamic raw) {
  try {
    // 이미 DateTime이면 로컬로 통일
    if (raw is DateTime) {
      return raw.isUtc ? raw.toLocal() : raw;
    }

    // epoch 숫자 지원(초/밀리초 자동 판별) - 서버가 숫자로 보낼 가능성 대비
    if (raw is num) {
      final int v = raw.toInt();
      // 10^11(= ~1973년 ms) 기준으로 초/밀리초 추정
      final bool isSeconds = v.abs() < 100000000000; // 1e11
      final dtUtc = isSeconds
          ? DateTime.fromMillisecondsSinceEpoch(v * 1000, isUtc: true)
          : DateTime.fromMillisecondsSinceEpoch(v, isUtc: true);
      return dtUtc.toLocal();
    }

    if (raw is String) {
      final s = raw.trim();

      // DateTime.parse는:
      // - Z 또는 ±HH:MM 있으면 UTC로 파싱(isUtc=true),
      // - 없으면 로컬로 파싱(isUtc=false).
      final dt = DateTime.parse(s);
      return dt.isUtc ? dt.toLocal() : dt;
    }
  } catch (_) {
    // fallthrough
  }

  // 파싱 실패 시 현재 로컬 시각으로
  return DateTime.now();
}

/// 오프셋 없는 로컬 ISO 문자열로 변환 (서버 기대 형식)
String _toLocalIso(DateTime dt) => dt.toLocal().toIso8601String();

class ChatMessage {
  final String role;        // 'user' | 'assistant'
  final String content;
  final DateTime createdAt; // 항상 '로컬 시각'

  ChatMessage({
    required this.role,
    required this.content,
    required this.createdAt,
  });

  factory ChatMessage.fromJson(Map<String, dynamic> j) {
    return ChatMessage(
      role: (j['role'] ?? '') as String,
      content: (j['content'] ?? '') as String,
      // 서버는 기본 'created_at' 사용, 혹시 캐시에 camelCase가 섞였으면 fallback.
      createdAt: _parseServerTime(j['created_at'] ?? j['createdAt']),
    );
  }

  Map<String, dynamic> toJson({bool snakeCase = true}) => {
    'role': role,
    'content': content,
    (snakeCase ? 'created_at' : 'createdAt'): _toLocalIso(createdAt),
  };

  ChatMessage copyWith({
    String? role,
    String? content,
    DateTime? createdAt,
  }) =>
      ChatMessage(
        role: role ?? this.role,
        content: content ?? this.content,
        createdAt: createdAt ?? this.createdAt,
      );
}

class ChatResponse {
  final String summary;
  final DateTime createdAt; // 항상 '로컬 시각'

  ChatResponse({
    required this.summary,
    required this.createdAt,
  });

  factory ChatResponse.fromJson(Map<String, dynamic> j) {
    return ChatResponse(
      summary: (j['summary'] ?? '') as String,
      createdAt: _parseServerTime(j['created_at'] ?? j['createdAt']),
    );
  }

  Map<String, dynamic> toJson({bool snakeCase = true}) => {
    'summary': summary,
    (snakeCase ? 'created_at' : 'createdAt'): _toLocalIso(createdAt),
  };
}

/// (옵션) 리스트 파서 유틸
List<ChatMessage> parseMessages(String body) {
  try {
    final decoded = jsonDecode(body);
    if (decoded is List) {
      return decoded
          .map((e) => ChatMessage.fromJson((e as Map).cast<String, dynamic>()))
          .toList();
    }
  } catch (_) {
    // ignore
  }
  return const <ChatMessage>[];
}
