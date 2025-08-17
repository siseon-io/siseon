// lib/pages/settings/stats_page.dart
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:fl_chart/fl_chart.dart';

import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/services/stats_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/widgets/rect_card.dart';
import 'dart:convert'; // 한글 깨짐 복구용(utf8/latin1)
import 'package:siseon2/pages/daily_hour_detail_page.dart';

class StatsPage extends StatefulWidget {
  const StatsPage({super.key});

  @override
  State<StatsPage> createState() => _StatsPageState();
}

// ──────────────────────────────────────────────────────────────
// 한 페이지(자세) 모델
class _BadTipPage {
  final String name;        // summary에서 추출(각도 제거)
  final String cue;         // 교정 팁
  final String ergo;        // 환경 팁
  final DateTime timeLocal; // 해당 스냅샷 endAt(로컬)
  _BadTipPage({
    required this.name,
    required this.cue,
    required this.ergo,
    required this.timeLocal,
  });
}

// ✅ 최신 minute 상태용
enum _PostureStatus { good, bad, none }
// ──────────────────────────────────────────────────────────────

class _StatsPageState extends State<StatsPage> {
  // THEME
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color errorRed = Color(0xFFF87171);
  static const TextStyle _label = TextStyle(color: Colors.white70, fontSize: 12);

  // STATE
  List<PostureStatsMinute> _dailyMins = []; // period=daily
  List<PostureStatsDay> _weeklyDays = [];   // period=day (최근7일)
  List<PostureStatsDay> _monthlyDays = [];  // period=day (최근 12개월)
  bool _isLoading = true;
  String? _error;

  int? _touchedWeekIndex; // 0~6
  int _touchedWeekGood = 0;
  int _touchedWeekBad = 0;

  int? _touchedMonth; // 0~11(회전 후)
  int _touchedMonthGood = 0;
  int _touchedMonthBad = 0;

  // ✅ 일간 도넛(원형) 터치 상태
  int? _touchedPie; // null=닫힘, 0=좋음, 1=나쁨

  // ✅ 최신 상태/시간
  _PostureStatus _latestStatus = _PostureStatus.none;
  DateTime? _latestMinuteTime;

  // ✅ 최신 나쁜자세 → 페이지 뷰(아래 회색만 넘김)
  final PageController _badPageController = PageController(initialPage: 0);
  List<_BadTipPage> _badPages = [];
  int _currentBadPage = 0;

  @override
  void initState() {
    super.initState();
    _fetchStats(); // 자동 새로고침 없음(한 번만 로드)
  }

  Future<void> _fetchStats() async {
    setState(() {
      _isLoading = true;
      _error = null;
      _touchedWeekIndex = null;
      _touchedMonth = null;
      _touchedPie = null;
      _badPages = [];
      _currentBadPage = 0;

      // 최신 상태 초기화
      _latestStatus = _PostureStatus.none;
      _latestMinuteTime = null;
    });

    try {
      final profile = await ProfileCacheService.loadProfile();
      final profileId = profile?['profileId'] ?? profile?['id'];
      if (profileId == null) {
        throw Exception('프로필을 찾을 수 없어요. 프로필을 먼저 선택해주세요.');
      }

      final now = DateTime.now();
      final todayStart = DateTime(now.year, now.month, now.day);
      final todayEnd = DateTime(now.year, now.month, now.day, 23, 59, 59, 999);
      final firstMonth = DateTime(now.year, now.month - 11, 1);

      final results = await Future.wait([
        StatsService.fetchMinuteStats(profileId: profileId, period: 'daily'),
        StatsService.fetchDayStats(
          profileId: profileId,
          from: todayStart.subtract(const Duration(days: 6)),
          to: todayEnd,
        ),
        StatsService.fetchDayStats(
          profileId: profileId,
          from: firstMonth,
          to: todayEnd,
        ),
      ]);

      if (!mounted) return;
      setState(() {
        _dailyMins   = results[0] as List<PostureStatsMinute>;
        _weeklyDays  = results[1] as List<PostureStatsDay>;
        _monthlyDays = results[2] as List<PostureStatsDay>;
      });

      _computeLatestStatus(); // ✅ 최신 minute 기준 상태 계산
      _buildBadPages();       // ← 최신 나쁜자세를 페이지로 구성
    } catch (e) {
      if (!mounted) return;
      setState(() => _error = '통계를 불러오지 못했어요.\n(${e.toString()})');
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  // ✅ 최신 minute 상태 계산 (validPosture / valid / badReasons.valid)
  void _computeLatestStatus() {
    if (_dailyMins.isEmpty) {
      setState(() {
        _latestStatus = _PostureStatus.none;
        _latestMinuteTime = null;
      });
      return;
    }

    // 최신 endAt 가진 minute 찾기
    PostureStatsMinute latest = _dailyMins.first;
    for (final m in _dailyMins) {
      if (m.endAt.isAfter(latest.endAt)) latest = m;
    }

    bool? valid = latest.validPosture;
    try {
      final v2 = (latest as dynamic).valid;
      if (v2 is bool) valid = v2;
    } catch (_) {}
    try {
      final br = (latest as dynamic).badReasons;
      final v3 = (br as dynamic).valid;
      if (v3 is bool) valid = v3;
    } catch (_) {}

    final t = latest.endAt.toLocal();
    setState(() {
      if (valid == true) {
        _latestStatus = _PostureStatus.good;
        _latestMinuteTime = t;
      } else if (valid == false) {
        _latestStatus = _PostureStatus.bad;
        _latestMinuteTime = t;
      } else {
        _latestStatus = _PostureStatus.none;
        _latestMinuteTime = null;
      }
    });
  }

  // ===== 한글 깨짐(모지박) 복구 =====
  String _fixKoreanIfGarbled(String s) {
    final looksGarbled = RegExp(r'(Ã.|Â.|ì.|í.|ë.|ê.|°|±|²|³|¼|½|¾)').hasMatch(s) &&
        !RegExp(r'[가-힣]').hasMatch(s);
    if (!looksGarbled) return s;
    try {
      final repaired = utf8.decode(latin1.encode(s));
      if (RegExp(r'[가-힣]').hasMatch(repaired)) return repaired;
      return s;
    } catch (_) {
      return s;
    }
  }

  String _cleanText(String? input) {
    if (input == null) return '';
    final fixed = _fixKoreanIfGarbled(input);
    return fixed.replaceAll(RegExp(r'\s+'), ' ').trim();
  }

  String _normalizeKey(String s) =>
      _cleanText(s).replaceAll(RegExp(r'\s+'), '');

  // ===== 최신 나쁜자세 → 페이지 리스트 생성 =====
  void _buildBadPages() {
    // 1) 최신 bad 찾기
    PostureStatsMinute? latest;
    for (int i = _dailyMins.length - 1; i >= 0; i--) {
      final it = _dailyMins[i];
      bool? valid = it.validPosture;
      try { final v2 = (it as dynamic).valid; if (v2 is bool) valid = v2; } catch (_) {}
      try { final br = (it as dynamic).badReasons; final v3 = (br as dynamic).valid; if (v3 is bool) valid = v3; } catch (_) {}
      if (valid == false) { latest = it; break; }
    }
    if (latest == null) {
      setState(() { _badPages = []; });
      return;
    }

    final timeLocal = latest.endAt.toLocal();

    // 2) summary에서 자세 이름만(각도 제거) 추출
    final names = <String>[];
    try {
      final br = (latest as dynamic).badReasons;
      final sum = _cleanText((br as dynamic).summary?.toString());
      if (sum.isNotEmpty) {
        names.addAll(
          sum.split(',').map((e) => e.split('(').first).map(_cleanText).where((e) => e.isNotEmpty),
        );
      }
    } catch (_) {}
    // fallback: 아무것도 없으면 reasons의 label 사용
    if (names.isEmpty) {
      try {
        final rs = ((latest as dynamic).badReasons as dynamic).reasons;
        if (rs is Iterable) {
          for (final r in rs) {
            final lbl = _cleanText((r as dynamic).label?.toString());
            if (lbl.isNotEmpty) names.add(lbl);
          }
        }
      } catch (_) {}
    }

    // 3) reasons를 label 기준으로 맵 구성(팁 찾기)
    final tipByLabel = <String, Map<String, String>>{};
    try {
      final rs = ((latest as dynamic).badReasons as dynamic).reasons;
      if (rs is Iterable) {
        for (final r in rs) {
          final lbl  = _cleanText((r as dynamic).label?.toString());
          final cue  = _cleanText((r as dynamic).cue?.toString());
          final ergo = _cleanText((r as dynamic).ergonomics?.toString());
          if (lbl.isEmpty) continue;
          tipByLabel[_normalizeKey(lbl)] = {'cue': cue, 'ergo': ergo};
        }
      }
    } catch (_) {}

    // 4) 페이지 리스트 만들기 (summary 순서 그대로)
    final pages = <_BadTipPage>[];
    for (final nm in names) {
      final key = _normalizeKey(nm);
      final tip = tipByLabel[key] ?? const {'cue': '', 'ergo': ''};
      pages.add(_BadTipPage(
        name: nm,
        cue: tip['cue'] ?? '',
        ergo: tip['ergo'] ?? '',
        timeLocal: timeLocal,
      ));
    }

    setState(() {
      _badPages = pages;
      _currentBadPage = 0;
    });
  }

  // HELPERS
  String _formatDuration(int seconds) {
    if (seconds <= 0) return '0분';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return '${h}시간 ${m}분';
    return '${m}분';
  }

  String _formatHM(DateTime d) => DateFormat('HH:mm').format(d);

  DateTime _dateForWeekIndex(int index) {
    final now = DateTime.now();
    final startOfWeek = now.subtract(Duration(days: now.weekday % 7)); // Sun=0
    return DateTime(startOfWeek.year, startOfWeek.month, startOfWeek.day)
        .add(Duration(days: index));
  }

  List<T> _rotateLeft<T>(List<T> list, int k) {
    if (list.isEmpty) return list;
    final r = k % list.length;
    if (r == 0) return List<T>.from(list);
    return [...list.sublist(r), ...list.sublist(0, r)];
  }

  Widget _legendMini() {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: const [
        Icon(Icons.square, color: errorRed, size: 10),
        SizedBox(width: 4),
        Text('잘못된 자세', style: _label),
        SizedBox(width: 10),
        Icon(Icons.square, color: primaryBlue, size: 10),
        SizedBox(width: 4),
        Text('올바른 자세', style: _label),
      ],
    );
  }

  // BUILD
  @override
  Widget build(BuildContext context) {
    if (_isLoading) {
      return const Scaffold(
        backgroundColor: backgroundBlack,
        body: Center(child: CircularProgressIndicator(color: primaryBlue)),
      );
    }

    // ▶ 자세히보기 네비 (오른쪽→왼쪽 슬라이드)
    void _openDailyDetail() {
      Navigator.of(context).push(
        PageRouteBuilder(
          transitionDuration: const Duration(milliseconds: 220),
          pageBuilder: (_, __, ___) => DailyHourDetailPage(dailyMins: _dailyMins),
          transitionsBuilder: (_, animation, __, child) {
            final tween = Tween<Offset>(
              begin: const Offset(1.0, 0.0),
              end: Offset.zero,
            ).chain(CurveTween(curve: Curves.easeOutCubic));
            return SlideTransition(position: animation.drive(tween), child: child);
          },
        ),
      );
    }

    if (_error != null) {
      return Scaffold(
        backgroundColor: backgroundBlack,
        appBar: _appBar(),
        body: Center(
          child: Padding(
            padding: const EdgeInsets.all(24),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                const Icon(Icons.error_outline, color: errorRed, size: 36),
                const SizedBox(height: 12),
                Text(_error!, style: const TextStyle(color: Colors.white70), textAlign: TextAlign.center),
                const SizedBox(height: 16),
                FilledButton(
                  style: FilledButton.styleFrom(backgroundColor: primaryBlue),
                  onPressed: _fetchStats,
                  child: const Text('다시 시도', style: TextStyle(color: Colors.white)),
                ),
              ],
            ),
          ),
        ),
      );
    }

    return Scaffold(
      backgroundColor: backgroundBlack,
      appBar: _appBar(),
      // ✅ 페이지 아무 곳이나 탭해도 툴팁 닫히도록 래핑
      body: GestureDetector(
        behavior: HitTestBehavior.translucent,
        onTap: () {
          if (_touchedWeekIndex != null || _touchedMonth != null || _touchedPie != null) {
            setState(() {
              _touchedWeekIndex = null;
              _touchedWeekGood = 0;
              _touchedWeekBad = 0;
              _touchedMonth = null;
              _touchedMonthGood = 0;
              _touchedMonthBad = 0;
              _touchedPie = null; // ✅ 도넛 툴팁 닫기
            });
          }
        },
        child: RefreshIndicator(
          color: Colors.white,
          backgroundColor: primaryBlue,
          onRefresh: _fetchStats, // 🔄 사용자가 당겨서 새로고침할 때만
          child: SafeArea(
            child: ListView(
              padding: const EdgeInsets.all(16),
              children: [
                // ✅ 최신 minute이 "좋음"이면 파란 배너
                if (_latestStatus == _PostureStatus.good)
                  RectCard(
                    outlineColor: primaryBlue,
                    padding: const EdgeInsets.all(12),
                    child: Row(
                      crossAxisAlignment: CrossAxisAlignment.center,
                      children: [
                        const Icon(Icons.check_circle, color: primaryBlue),
                        const SizedBox(width: 8),
                        const Expanded(
                          child: Text(
                            '올바른 자세입니다. 유지해주세요!',
                            style: TextStyle(
                              color: Colors.white,
                              fontSize: 16,
                              fontWeight: FontWeight.w700,
                            ),
                          ),
                        ),
                        if (_latestMinuteTime != null)
                          Text(_formatHM(_latestMinuteTime!),
                              style: const TextStyle(color: Colors.white70, fontSize: 12)),
                      ],
                    ),
                  ),
                if (_latestStatus == _PostureStatus.good) const SizedBox(height: 20),

                // ✅ 최신 minute이 "좋음"이 아닐 때만 나쁜자세 카드 노출
                if (_latestStatus != _PostureStatus.good && _badPages.isNotEmpty)
                  RectCard(
                    outlineColor: errorRed,
                    padding: const EdgeInsets.all(12),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        // ── 헤더(고정)
                        Row(
                          crossAxisAlignment: CrossAxisAlignment.center,
                          children: [
                            const Icon(Icons.error_outline, color: errorRed),
                            const SizedBox(width: 8),
                            Expanded(
                              child: Text(
                                '${_badPages.map((e) => e.name).join(', ')}이(가) 감지되었습니다.',
                                style: const TextStyle(
                                  color: Colors.white,
                                  fontSize: 16,
                                  fontWeight: FontWeight.w700,
                                ),
                              ),
                            ),
                            Text(_formatHM(_badPages.first.timeLocal),
                                style: const TextStyle(color: Colors.white70, fontSize: 12)),
                          ],
                        ),
                        const SizedBox(height: 10),

                        // ── 아래 밝은 회색 블록만 PageView
                        SizedBox(
                          height: 118,
                          child: PageView.builder(
                            controller: _badPageController,
                            itemCount: _badPages.length,
                            onPageChanged: (i) => setState(() => _currentBadPage = i),
                            itemBuilder: (_, i) {
                              final p = _badPages[i];
                              return Container(
                                margin: const EdgeInsets.only(bottom: 2),
                                padding: const EdgeInsets.all(10),
                                decoration: BoxDecoration(
                                  color: Colors.white.withOpacity(0.08),
                                  borderRadius: BorderRadius.circular(10),
                                ),
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Text(p.name,
                                        style: const TextStyle(
                                            color: Colors.white,
                                            fontWeight: FontWeight.w700)),
                                    if (p.cue.isNotEmpty) ...[
                                      const SizedBox(height: 4),
                                      Text('교정 팁: ${p.cue}',
                                          style: const TextStyle(color: Colors.white70, fontSize: 12)),
                                    ],
                                    if (p.ergo.isNotEmpty) ...[
                                      const SizedBox(height: 2),
                                      Text('환경 팁: ${p.ergo}',
                                          style: const TextStyle(color: Colors.white70, fontSize: 12)),
                                    ],
                                  ],
                                ),
                              );
                            },
                          ),
                        ),
                        const SizedBox(height: 8),

                        // 점 인디케이터
                        Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: List.generate(_badPages.length, (i) {
                            final active = i == _currentBadPage;
                            return AnimatedContainer(
                              duration: const Duration(milliseconds: 200),
                              margin: const EdgeInsets.symmetric(horizontal: 4),
                              width: active ? 8 : 6,
                              height: active ? 8 : 6,
                              decoration: BoxDecoration(
                                color: active ? primaryBlue : Colors.white24,
                                shape: BoxShape.circle,
                              ),
                            );
                          }),
                        ),
                      ],
                    ),
                  ),
                if (_latestStatus != _PostureStatus.good && _badPages.isNotEmpty)
                  const SizedBox(height: 20),

                // 일간 도넛
                RectCard(
                  elevated: true,
                  outlineColor: Colors.white.withOpacity(0.16),
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(children: [
                        const Expanded(
                          child: Text(
                            '일간 자세 비율',
                            style: TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w600),
                          ),
                        ),
                        _legendMini()
                      ]),
                      const SizedBox(height: 12),
                      SizedBox(height: 220, child: _AveragePieChart()),

                      // 자세히 보기 버튼
                      const SizedBox(height: 8),
                      if (_dailyMins.isNotEmpty)
                        Align(
                          alignment: Alignment.centerRight,
                          child: TextButton.icon(
                            onPressed: _openDailyDetail,
                            icon: const Icon(Icons.chevron_right, color: Colors.white70, size: 18),
                            label: const Text('자세히 보기', style: TextStyle(color: Colors.white70)),
                            style: TextButton.styleFrom(
                              foregroundColor: Colors.white70,
                              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                            ),
                          ),
                        ),
                    ],
                  ),
                ),
                const SizedBox(height: 20),

                // 주간 스택 바
                RectCard(
                  elevated: true,
                  outlineColor: Colors.white.withOpacity(0.16),
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(children: [
                        const Expanded(
                          child: Text(
                            '주간 자세 통계',
                            style: TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w600),
                          ),
                        ),
                        _legendMini()
                      ]),
                      const SizedBox(height: 12),
                      SizedBox(height: 240, child: _StackedWeeklyBarChart()),
                    ],
                  ),
                ),
                const SizedBox(height: 20),

                // 월별 트렌드
                RectCard(
                  elevated: true,
                  outlineColor: Colors.white.withOpacity(0.16),
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(children: [
                        const Expanded(
                          child: Text(
                            '연간 월별 자세 추이',
                            style: TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w600),
                          ),
                        ),
                        _legendMini()
                      ]),
                      const SizedBox(height: 12),
                      SizedBox(height: 240, child: _MonthlyTrendChart()),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  AppBar _appBar() {
    return AppBar(
      backgroundColor: backgroundBlack,
      elevation: 0,
      iconTheme: const IconThemeData(color: Colors.white),
      title: const Text('통계',
          style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
      actions: [
        IconButton(
          tooltip: '새로고침',
          onPressed: _fetchStats,
          icon: const Icon(Icons.refresh),
        ),
      ],
    );
  }

  // CHARTS
  Widget _AveragePieChart() {
    if (_dailyMins.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    int totalGood = 0;
    int totalBad = 0;
    for (final s in _dailyMins) {
      if (s.validPosture == true) {
        totalGood += s.durationSeconds;
      } else {
        totalBad += s.durationSeconds;
      }
    }
    final total = totalGood + totalBad;
    if (total == 0) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    String pct(int sec) => ((sec / total) * 100).toStringAsFixed(0);

    return LayoutBuilder(builder: (context, constraints) {
      const tipW = 160.0;
      final left = constraints.maxWidth - tipW - 8; // 오른쪽 상단 정렬

      return Stack(
        children: [
          PieChart(
            PieChartData(
              sectionsSpace: 2,
              centerSpaceRadius: 40,
              pieTouchData: PieTouchData(
                touchCallback: (event, response) {
                  if (event is FlTapUpEvent) {
                    final idx = response?.touchedSection?.touchedSectionIndex;
                    setState(() {
                      if (idx == null) {
                        _touchedPie = null; // 빈 영역 → 닫기
                      } else if (_touchedPie == idx) {
                        _touchedPie = null; // 동일 섹션 다시 탭 → 닫기
                      } else {
                        _touchedPie = idx;   // 다른 섹션 → 열기
                      }
                    });
                  }
                },
              ),
              sections: [
                // ✅ 내부 텍스트 제거(title: '')
                PieChartSectionData(
                  value: totalGood.toDouble(),
                  color: primaryBlue,
                  title: '',
                ),
                PieChartSectionData(
                  value: totalBad.toDouble(),
                  color: errorRed,
                  title: '',
                ),
              ],
            ),
          ),

          // ✅ 툴팁 떠 있을 때 차트 아무 곳이나 탭하면 닫히는 투명 레이어
          if (_touchedPie != null)
            Positioned.fill(
              child: GestureDetector(
                behavior: HitTestBehavior.translucent,
                onTap: () => setState(() => _touchedPie = null),
              ),
            ),

          // ✅ 커스텀 팁(오늘 통계)
          if (_touchedPie != null)
            Positioned(
              top: 8,
              left: left,
              child: Container(
                width: tipW,
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.85),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('오늘 통계',
                        style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
                    const SizedBox(height: 6),
                    Row(children: [
                      const Icon(Icons.square, color: errorRed, size: 10),
                      const SizedBox(width: 6),
                      Expanded(
                        child: Text(
                          '나쁨: ${_formatDuration(totalBad)} (${pct(totalBad)}%)',
                          style: const TextStyle(color: Colors.white),
                        ),
                      ),
                    ]),
                    Row(children: [
                      const Icon(Icons.square, color: primaryBlue, size: 10),
                      const SizedBox(width: 6),
                      Expanded(
                        child: Text(
                          '좋음: ${_formatDuration(totalGood)} (${pct(totalGood)}%)',
                          style: const TextStyle(color: Colors.white),
                        ),
                      ),
                    ]),
                  ],
                ),
              ),
            ),
        ],
      );
    });
  }

  Widget _StackedWeeklyBarChart() {
    if (_weeklyDays.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    final dayWise = List.generate(7, (_) => {'good': 0, 'bad': 0});
    for (final d in _weeklyDays) {
      final w = d.statDate.weekday % 7; // Mon=1..Sun=7 -> 1..6,0
      dayWise[w]['good'] = dayWise[w]['good']! + d.goodCount * 60;
      dayWise[w]['bad']  = dayWise[w]['bad']!  + d.badCount * 60;
    }

    final hasData = dayWise.any((e) => (e['good'] ?? 0) > 0 || (e['bad'] ?? 0) > 0);
    if (!hasData) return const Center(child: Text('데이터 없음', style: _label));

    final todayIdx = DateTime.now().weekday % 7; // 0=일..6=토
    final shift = (todayIdx + 1) % 7;
    const baseLabels = ['일', '월', '화', '수', '목', '금', '토'];
    final rotatedLabels = _rotateLeft(baseLabels, shift);
    final rotatedDayWise = _rotateLeft(dayWise, shift);

    return LayoutBuilder(builder: (context, constraints) {
      final chartWidth = constraints.maxWidth;
      final barSpacing = chartWidth / 7;
      const tooltipWidth = 140.0;

      double leftFor(int index) {
        final pos = barSpacing * index;
        if (pos + tooltipWidth > chartWidth) {
          return chartWidth - tooltipWidth - 8;
        }
        return pos;
      }

      return Stack(
        children: [
          BarChart(
            BarChartData(
              gridData: FlGridData(show: false),
              borderData: FlBorderData(show: false),
              titlesData: FlTitlesData(
                bottomTitles: AxisTitles(
                  sideTitles: SideTitles(
                    showTitles: true,
                    getTitlesWidget: (value, meta) =>
                        Text(rotatedLabels[value.toInt()], style: _label),
                  ),
                ),
                leftTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
                rightTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
                topTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
              ),
              barTouchData: BarTouchData(
                enabled: true,
                handleBuiltInTouches: false,
                touchCallback: (event, response) {
                  if (event is FlTapUpEvent) {
                    final i = response?.spot?.touchedBarGroupIndex;
                    setState(() {
                      if (i == null) {
                        // ✅ 빈 영역 탭 → 닫기
                        _touchedWeekIndex = null;
                        _touchedWeekGood = 0;
                        _touchedWeekBad = 0;
                      } else if (_touchedWeekIndex == i) {
                        // 동일 막대 다시 탭 → 닫기
                        _touchedWeekIndex = null;
                        _touchedWeekGood = 0;
                        _touchedWeekBad = 0;
                      } else {
                        // 다른 막대 탭 → 갱신
                        _touchedWeekIndex = i;
                        _touchedWeekGood = rotatedDayWise[i]['good']!;
                        _touchedWeekBad  = rotatedDayWise[i]['bad']!;
                      }
                    });
                  }
                },
                touchTooltipData: BarTouchTooltipData(getTooltipItem: (_, __, ___, ____) => null),
              ),
              barGroups: List.generate(7, (i) {
                final goodH = (rotatedDayWise[i]['good']! / 3600).toDouble();
                final badH  = (rotatedDayWise[i]['bad']!  / 3600).toDouble();
                final sum = goodH + badH;
                return BarChartGroupData(x: i, barRods: [
                  BarChartRodData(
                    toY: sum,
                    width: 14,
                    borderRadius: BorderRadius.circular(4),
                    rodStackItems: [
                      BarChartRodStackItem(0, goodH, primaryBlue),
                      BarChartRodStackItem(goodH, sum, errorRed),
                    ],
                  )
                ]);
              }),
            ),
          ),

          // ✅ 툴팁 떠 있을 때 차트 아무 곳이나 탭하면 닫히는 투명 레이어
          if (_touchedWeekIndex != null)
            Positioned.fill(
              child: GestureDetector(
                behavior: HitTestBehavior.translucent,
                onTap: () {
                  setState(() {
                    _touchedWeekIndex = null;
                    _touchedWeekGood = 0;
                    _touchedWeekBad = 0;
                  });
                },
              ),
            ),

          if (_touchedWeekIndex != null)
            Positioned(
              top: 8,
              left: leftFor(_touchedWeekIndex!),
              child: Container(
                width: tooltipWidth,
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.85),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Builder(builder: (_) {
                      final actualIdx = (_touchedWeekIndex! + shift) % 7;
                      final dateText = DateFormat('M월 d일').format(_dateForWeekIndex(actualIdx));
                      return Text('$dateText 통계',
                          style: const TextStyle(color: Colors.white, fontWeight: FontWeight.bold));
                    }),
                    const SizedBox(height: 6),
                    Row(children: [
                      const Icon(Icons.square, color: errorRed, size: 10),
                      const SizedBox(width: 6),
                      Expanded(
                        child: Text(_formatDuration(_touchedWeekBad),
                            style: const TextStyle(color: Colors.white)),
                      ),
                    ]),
                    Row(children: [
                      const Icon(Icons.square, color: primaryBlue, size: 10),
                      const SizedBox(width: 6),
                      Expanded(
                        child: Text(_formatDuration(_touchedWeekGood),
                            style: const TextStyle(color: Colors.white)),
                      ),
                    ]),
                  ],
                ),
              ),
            ),
        ],
      );
    });
  }

  Widget _MonthlyTrendChart() {
    if (_monthlyDays.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    final now = DateTime.now();
    final months = List.generate(12, (i) => DateTime(now.year, now.month - 11 + i, 1));
    final labels = months.map((d) => '${d.month}월').toList();

    final List<int> goodSec = List.filled(12, 0);
    final List<int> badSec  = List.filled(12, 0);

    for (final d in _monthlyDays) {
      final ym = DateTime(d.statDate.year, d.statDate.month, 1);
      final idx = months.indexWhere((m) => m.year == ym.year && m.month == ym.month);
      if (idx == -1) continue;

      goodSec[idx] += d.goodCount * 60;
      badSec[idx]  += d.badCount * 60;
    }

    final hasGood = goodSec.any((v) => v > 0);
    final hasBad  = badSec.any((v) => v > 0);
    if (!hasGood && !hasBad) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    final goodSpots = <FlSpot>[];
    final badSpots  = <FlSpot>[];
    for (int i = 0; i < 12; i++) {
      if (goodSec[i] > 0) goodSpots.add(FlSpot(i.toDouble(), goodSec[i] / 3600.0));
      if (badSec[i]  > 0) badSpots.add(FlSpot(i.toDouble(), badSec[i] / 3600.0));
    }

    final List<LineChartBarData> bars = [];
    if (goodSpots.isNotEmpty) {
      bars.add(LineChartBarData(
        isCurved: false,
        color: primaryBlue,
        barWidth: 3,
        dotData: FlDotData(show: true),
        spots: goodSpots,
      ));
    }
    if (badSpots.isNotEmpty) {
      bars.add(LineChartBarData(
        isCurved: false,
        color: errorRed,
        barWidth: 3,
        dotData: FlDotData(show: true),
        spots: badSpots,
      ));
    }

    return LayoutBuilder(builder: (context, constraints) {
      const tooltipWidth = 140.0;
      double leftForIndex(int idx) {
        final chartWidth = constraints.maxWidth;
        final spacing = chartWidth / 12;
        final pos = spacing * idx;
        if (pos + tooltipWidth > chartWidth) {
          return chartWidth - tooltipWidth - 8;
        }
        return pos;
      }

      return Stack(
        children: [
          LineChart(
            LineChartData(
              minX: 0,
              maxX: 11,
              minY: 0,
              gridData: FlGridData(show: false),
              borderData: FlBorderData(show: false),
              titlesData: FlTitlesData(
                bottomTitles: AxisTitles(
                  sideTitles: SideTitles(
                    showTitles: true,
                    interval: 1,
                    getTitlesWidget: (v, _) {
                      final i = v.toInt().clamp(0, 11);
                      return Text(labels[i], style: _label.copyWith(fontSize: 10));
                    },
                  ),
                ),
                leftTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
                rightTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
                topTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
              ),
              lineTouchData: LineTouchData(
                enabled: true,
                handleBuiltInTouches: false,
                touchCallback: (event, response) {
                  if (event is FlTapUpEvent) {
                    final hasSpot = response?.lineBarSpots != null && response!.lineBarSpots!.isNotEmpty;
                    final idx = hasSpot ? response!.lineBarSpots!.first.x.toInt().clamp(0, 11) : null;

                    setState(() {
                      if (idx == null) {
                        // ✅ 빈 영역 탭 → 닫기
                        _touchedMonth = null;
                        _touchedMonthGood = 0;
                        _touchedMonthBad = 0;
                      } else if (_touchedMonth == idx) {
                        // 동일 포인트 다시 탭 → 닫기
                        _touchedMonth = null;
                        _touchedMonthGood = 0;
                        _touchedMonthBad = 0;
                      } else {
                        // 다른 포인트 탭 → 갱신
                        _touchedMonth = idx;
                        _touchedMonthGood = goodSec[idx];
                        _touchedMonthBad  = badSec[idx];
                      }
                    });
                  }
                },
                touchTooltipData: LineTouchTooltipData(getTooltipItems: (_) => []),
              ),
              lineBarsData: bars,
            ),
          ),

          // ✅ 툴팁 떠 있을 때 차트 아무 곳이나 탭하면 닫히는 투명 레이어
          if (_touchedMonth != null)
            Positioned.fill(
              child: GestureDetector(
                behavior: HitTestBehavior.translucent,
                onTap: () {
                  setState(() {
                    _touchedMonth = null;
                    _touchedMonthGood = 0;
                    _touchedMonthBad = 0;
                  });
                },
              ),
            ),

          if (_touchedMonth != null)
            Positioned(
              top: 8,
              left: leftForIndex(_touchedMonth!),
              child: Container(
                width: tooltipWidth,
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.85),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('${labels[_touchedMonth!]} 통계',
                        style: const TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
                    const SizedBox(height: 6),
                    Row(children: [
                      const Icon(Icons.square, color: errorRed, size: 10),
                      const SizedBox(width: 6),
                      Expanded(child: Text(_formatDuration(_touchedMonthBad),
                          style: const TextStyle(color: Colors.white))),
                    ]),
                    Row(children: [
                      const Icon(Icons.square, color: primaryBlue, size: 10),
                      const SizedBox(width: 6),
                      Expanded(child: Text(_formatDuration(_touchedMonthGood),
                          style: const TextStyle(color: Colors.white))),
                    ]),
                  ],
                ),
              ),
            ),
        ],
      );
    });
  }

  @override
  void dispose() {
    _badPageController.dispose(); // ✅ PageController 정리
    super.dispose();
  }
}
