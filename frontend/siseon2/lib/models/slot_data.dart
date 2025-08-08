// lib/models/slot_data.dart
class PostureStats {
  final int id;
  final int profileId;
  final int durationSeconds;
  final bool validPosture;
  final DateTime startAt;
  final DateTime endAt;
  final int slotIndex;

  PostureStats({
    required this.id,
    required this.profileId,
    required this.durationSeconds,
    required this.validPosture,
    required this.startAt,
    required this.endAt,
    required this.slotIndex,
  });

  factory PostureStats.fromJson(Map<String, dynamic> json) {
    final _valid = (json['validPosture'] ?? json['valid_posture']) as bool;
    return PostureStats(
      id: json['id'] as int,
      profileId: (json['profileId'] ?? json['profile_id']) as int,
      durationSeconds:
      (json['durationSeconds'] ?? json['duration_seconds'] ?? json['duration']) as int,
      validPosture: _valid,
      startAt: DateTime.parse((json['startAt'] ?? json['start_at']) as String),
      endAt: DateTime.parse((json['endAt'] ?? json['end_at']) as String),
      slotIndex: (json['slotIndex'] ?? json['slot_index'] ?? json['slot']) as int,
    );
  }
}
