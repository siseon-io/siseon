import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:fl_chart/fl_chart.dart';

import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/services/stats_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import 'package:siseon2/widgets/rect_card.dart';

class StatsPage extends StatefulWidget {
  const StatsPage({super.key});

  @override
  State<StatsPage> createState() => _StatsPageState();
}

class _StatsPageState extends State<StatsPage> {
  // THEME
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color errorRed = Color(0xFFF87171);
  static const TextStyle _label = TextStyle(
      color: Colors.white70, fontSize: 12);

  // STATE

  List<PostureStatsMinute> _dailyMins = []; // period=daily
  List<PostureStatsDay> _weeklyDays = []; // period=day (최근7일)
  List<PostureStatsDay> _monthlyDays = []; // ✅ 폴백용(day 집계 12개월)
  bool _isLoading = true;
  String? _error;

  int? _touchedWeekIndex; // 0~6
  int _touchedWeekGood = 0;
  int _touchedWeekBad = 0;

  int? _touchedMonth; // 0~11(회전 후)
  int _touchedMonthGood = 0;
  int _touchedMonthBad = 0;

  @override
  void initState() {
    super.initState();
    _fetchStats();
  }

  Future<void> _fetchStats() async {
    setState(() {
      _isLoading = true;
      _error = null;
      _touchedWeekIndex = null;
      _touchedMonth = null;
    });

    try {
      final profile = await ProfileCacheService.loadProfile();
      final profileId = profile?['profileId'] ?? profile?['id'];
      if (profileId == null) throw Exception('프로필을 찾을 수 없어요. 프로필을 먼저 선택해주세요.');

      final now = DateTime.now();
      final todayStart = DateTime(now.year, now.month, now.day);
      final todayEnd = DateTime(
          now.year,
          now.month,
          now.day,
          23,
          59,
          59,
          999);

      // ✅ 최근 12개월 시작(해당 월 1일 00:00)
      final firstMonth = DateTime(now.year, now.month - 11, 1);

      // 병렬 호출
      final results = await Future.wait([
        // 일간 도넛: minute(daily)
        StatsService.fetchMinuteStats(profileId: profileId, period: 'daily'),

        // 주간 스택바: day 집계(최근 7일)
        StatsService.fetchDayStats(
          profileId: profileId,
          from: todayStart.subtract(const Duration(days: 6)),
          to: todayEnd,
        ),

        // ✅ 월간 트렌드: day 집계(최근 12개월)로 변경
        StatsService.fetchDayStats(
          profileId: profileId,
          from: firstMonth,
          to: todayEnd,
        ),
      ]);

      if (!mounted) return;
      setState(() {
        _dailyMins = results[0] as List<PostureStatsMinute>;
        _weeklyDays = results[1] as List<PostureStatsDay>;
        _monthlyDays = results[2] as List<PostureStatsDay>; // ✅
      });
    } catch (e) {
      if (!mounted) return;
      setState(() => _error = '통계를 불러오지 못했어요.\n(${e.toString()})');
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }


  // HELPERS
  String _formatDuration(int seconds) {
    if (seconds <= 0) return '0분';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return '${h}시간 ${m}분';
    return '${m}분';
  }

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
                Text(_error!, style: const TextStyle(color: Colors.white70),
                    textAlign: TextAlign.center),
                const SizedBox(height: 16),
                FilledButton(
                  style: FilledButton.styleFrom(backgroundColor: primaryBlue),
                  onPressed: _fetchStats,
                  child: const Text(
                      '다시 시도', style: TextStyle(color: Colors.white)),
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
      body: RefreshIndicator(
        color: Colors.white,
        backgroundColor: primaryBlue,
        onRefresh: _fetchStats,
        child: SafeArea(
          child: ListView(
            padding: const EdgeInsets.all(16),
            children: [
              // 일간 도넛
              RectCard(
                elevated: true,
                outlineColor: Colors.white.withOpacity(0.16),
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(children: [
                      const Expanded(child: Text('일간 자세 비율', style: TextStyle(
                          color: Colors.white,
                          fontSize: 16,
                          fontWeight: FontWeight.w600))),
                      _legendMini()
                    ]),
                    const SizedBox(height: 12),
                    SizedBox(height: 220, child: _AveragePieChart()),
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
                      const Expanded(child: Text('주간 자세 통계', style: TextStyle(
                          color: Colors.white,
                          fontSize: 16,
                          fontWeight: FontWeight.w600))),
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
                          child: Text('연간 월별 자세 추이', style: TextStyle(
                              color: Colors.white,
                              fontSize: 16,
                              fontWeight: FontWeight.w600))),
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

    final goodPct = (totalGood / total * 100).toStringAsFixed(0);
    final badPct = (totalBad / total * 100).toStringAsFixed(0);

    return PieChart(
      PieChartData(
        sectionsSpace: 2,
        centerSpaceRadius: 40,
        sections: [
          PieChartSectionData(
            value: totalGood.toDouble(),
            color: primaryBlue,
            title: '${_formatDuration(totalGood)}\n($goodPct%)',
            titleStyle: const TextStyle(
                color: Colors.white, fontWeight: FontWeight.bold, fontSize: 12),
          ),
          PieChartSectionData(
            value: totalBad.toDouble(),
            color: errorRed,
            title: '${_formatDuration(totalBad)}\n($badPct%)',
            titleStyle: const TextStyle(
                color: Colors.white, fontWeight: FontWeight.bold, fontSize: 12),
          ),
        ],
      ),
    );
  }

  Widget _StackedWeeklyBarChart() {
    if (_weeklyDays.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    // 일(0)~토(6) 누적(초) — day 집계의 분 단위 “개수”를 초로 환산(*60)
    final dayWise = List.generate(7, (_) => {'good': 0, 'bad': 0});
    for (final d in _weeklyDays) {
      final w = d.statDate.weekday % 7; // Mon=1..Sun=7 -> 1..6,0
      dayWise[w]['good'] = dayWise[w]['good']! + d.goodCount * 60;
      dayWise[w]['bad'] = dayWise[w]['bad']! + d.badCount * 60;
    }

    final hasData = dayWise.any((e) =>
    (e['good'] ?? 0) > 0 || (e['bad'] ?? 0) > 0);
    if (!hasData) return const Center(child: Text('데이터 없음', style: _label));

    final todayIdx = DateTime
        .now()
        .weekday % 7; // 0=일..6=토
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
                leftTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false)),
                rightTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false)),
                topTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false)),
              ),
              barTouchData: BarTouchData(
                enabled: true,
                handleBuiltInTouches: false,
                touchCallback: (event, response) {
                  if (event is FlTapUpEvent && response?.spot != null) {
                    final i = response!.spot!.touchedBarGroupIndex;
                    setState(() {
                      if (_touchedWeekIndex == i) {
                        _touchedWeekIndex = null;
                        _touchedWeekGood = 0;
                        _touchedWeekBad = 0;
                      } else {
                        _touchedWeekIndex = i;
                        _touchedWeekGood = rotatedDayWise[i]['good']!;
                        _touchedWeekBad = rotatedDayWise[i]['bad']!;
                      }
                    });
                  }
                },
                touchTooltipData: BarTouchTooltipData(
                    getTooltipItem: (_, __, ___, ____) => null),
              ),
              barGroups: List.generate(7, (i) {
                final goodH = (rotatedDayWise[i]['good']! / 3600).toDouble();
                final badH = (rotatedDayWise[i]['bad']! / 3600).toDouble();
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
          if (_touchedWeekIndex != null)
            Positioned(
              top: 8,
              left: leftFor(_touchedWeekIndex!),
              child: Container(
                width: tooltipWidth,
                padding: const EdgeInsets.symmetric(
                    horizontal: 12, vertical: 8),
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.85),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Builder(builder: (_) {
                      final actualIdx = (_touchedWeekIndex! + shift) % 7;
                      final dateText =
                      DateFormat('M월 d일').format(_dateForWeekIndex(actualIdx));
                      return Text('$dateText 통계',
                          style: const TextStyle(
                              color: Colors.white,
                              fontWeight: FontWeight.bold));
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

    // 최근 12개월 Year-Month 버킷 고정 (왼→오: 오래된→현재달)
    final now = DateTime.now();
    final months = List.generate(
        12, (i) => DateTime(now.year, now.month - 11 + i, 1));
    final labels = months.map((d) => '${d.month}월').toList();

    // 월별 누적(초) — day 집계의 분 단위 "개수"를 초로 환산(*60)
    final List<int> goodSec = List.filled(12, 0);
    final List<int> badSec = List.filled(12, 0);

    for (final d in _monthlyDays) {
      final ym = DateTime(d.statDate.year, d.statDate.month, 1);
      final idx = months.indexWhere((m) =>
      m.year == ym.year && m.month == ym.month);
      if (idx == -1) continue;

      goodSec[idx] += d.goodCount * 60;
      badSec[idx] += d.badCount * 60;
    }

    final hasGood = goodSec.any((v) => v > 0);
    final hasBad = badSec.any((v) => v > 0);
    if (!hasGood && !hasBad) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    // 0값은 점 생략
    final goodSpots = <FlSpot>[];
    final badSpots = <FlSpot>[];
    for (int i = 0; i < 12; i++) {
      if (goodSec[i] > 0) goodSpots.add(
          FlSpot(i.toDouble(), goodSec[i] / 3600.0));
      if (badSec[i] > 0) badSpots.add(FlSpot(i.toDouble(), badSec[i] / 3600.0));
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
                    showTitles: true, interval: 1,
                    getTitlesWidget: (v, _) {
                      final i = v.toInt().clamp(0, 11);
                      return Text(labels[i], style: _label.copyWith(
                          fontSize: 10));
                    },
                  ),
                ),
                leftTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false)),
                rightTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false)),
                topTitles: const AxisTitles(
                    sideTitles: SideTitles(showTitles: false)),
              ),
              lineTouchData: LineTouchData(
                enabled: true,
                handleBuiltInTouches: false,
                touchCallback: (event, response) {
                  if (event is FlTapUpEvent &&
                      response?.lineBarSpots != null &&
                      response!.lineBarSpots!.isNotEmpty) {
                    final idx = response.lineBarSpots!.first.x.toInt().clamp(
                        0, 11);
                    setState(() {
                      if (_touchedMonth == idx) {
                        _touchedMonth = null;
                        _touchedMonthGood = 0;
                        _touchedMonthBad = 0;
                      } else {
                        _touchedMonth = idx;
                        _touchedMonthGood = goodSec[idx];
                        _touchedMonthBad = badSec[idx];
                      }
                    });
                  }
                },
                touchTooltipData: LineTouchTooltipData(
                    getTooltipItems: (_) => []),
              ),
              lineBarsData: bars,
            ),
          ),
          if (_touchedMonth != null)
            Positioned(
              top: 8,
              left: leftForIndex(_touchedMonth!),
              child: Container(
                width: tooltipWidth,
                padding: const EdgeInsets.symmetric(
                    horizontal: 12, vertical: 8),
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.85),
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('${labels[_touchedMonth!]} 통계',
                        style: const TextStyle(
                            color: Colors.white, fontWeight: FontWeight.bold)),
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
}