class ChatMessage {
  final String role;        // "user" | "assistant"
  final String content;
  final DateTime createdAt;

  ChatMessage({
    required this.role,
    required this.content,
    required this.createdAt,
  });

  factory ChatMessage.fromJson(Map<String, dynamic> j) => ChatMessage(
    role: j['role'] as String,
    content: j['content'] as String? ?? '',
    createdAt: DateTime.parse(j['created_at'] as String).toLocal(),
  );
}

class ChatResponse {
  final String summary;
  final Map<String, dynamic> details;
  final DateTime createdAt;

  ChatResponse({
    required this.summary,
    required this.details,
    required this.createdAt,
  });

  factory ChatResponse.fromJson(Map<String, dynamic> j) => ChatResponse(
    summary: j['summary'] as String? ?? '',
    details: (j['details'] as Map?)?.cast<String, dynamic>() ?? <String, dynamic>{},
    createdAt: DateTime.parse(j['created_at'] as String).toLocal(),
  );
}
