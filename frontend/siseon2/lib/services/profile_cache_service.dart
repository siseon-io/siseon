// lib/services/profile_cache_service.dart
import 'dart:convert';
import 'package:flutter/foundation.dart';            // ✅ 추가: ValueNotifier
import 'package:shared_preferences/shared_preferences.dart';

class ProfileCacheService {
  static const _key = 'selectedProfile';

  /// ✅ 전역 프로필 변경 알림 (null=미선택/삭제)
  static final ValueNotifier<Map<String, dynamic>?> profile =
  ValueNotifier<Map<String, dynamic>?>(null);

  static Future<void> saveProfile(Map<String, dynamic> profileMap) async {
    final prefs = await SharedPreferences.getInstance();

    // id만 있으면 profileId로 복사
    if (profileMap.containsKey('id') && !profileMap.containsKey('profileId')) {
      profileMap['profileId'] = profileMap['id'];
    }

    await prefs.setString(_key, jsonEncode(profileMap));

    // ✅ 변경 알림
    profile.value = Map<String, dynamic>.from(profileMap);
  }

  static Future<Map<String, dynamic>?> loadProfile() async {
    final prefs = await SharedPreferences.getInstance();
    final jsonStr = prefs.getString(_key);
    if (jsonStr == null) return null;
    final map = jsonDecode(jsonStr) as Map<String, dynamic>;
    // ✅ 앱 최초 구동 시에도 한 번 채워두면 좋음
    if (profile.value == null) {
      profile.value = Map<String, dynamic>.from(map);
    }
    return map;
  }

  static Future<void> clearProfile() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove(_key);
    // ✅ 변경 알림
    profile.value = null;
  }
}
