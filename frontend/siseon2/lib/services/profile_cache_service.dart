// lib/services/profile_cache_service.dart
import 'dart:convert';
import 'package:shared_preferences/shared_preferences.dart';

class ProfileCacheService {
  static const _key = 'selectedProfile';

  static Future<void> saveProfile(Map<String, dynamic> profile) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_key, jsonEncode(profile));
  }

  static Future<Map<String, dynamic>?> loadProfile() async {
    final prefs = await SharedPreferences.getInstance();
    final jsonStr = prefs.getString(_key);
    if (jsonStr == null) return null;
    return jsonDecode(jsonStr);
  }

  static Future<void> clearProfile() async {
    final prefs = await SharedPreferences.getInstance();
    prefs.remove(_key);
  }
}
