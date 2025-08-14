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
  // ── THEME ────────────────────────────────────────────────────────────────────
  static const Color backgroundBlack = Color(0xFF0D1117);
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color errorRed = Color(0xFFF87171);
  static const TextStyle _label = TextStyle(
      color: Colors.white70, fontSize: 12);

  // ── STATE ────────────────────────────────────────────────────────────────────
  List<PostureStats> _daily = [];
  List<PostureStats> _weekly = [];
  List<PostureStats> _monthly = [];

  bool _isLoading = true;
  String? _error;

  // weekly tooltip
  int? _touchedWeekIndex; // 0(Sun) ~ 6(Sat)
  int _touchedWeekGood = 0;
  int _touchedWeekBad = 0;

  // monthly tooltip (회전 후 인덱스 0~11)
  int? _touchedMonth;
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
      if (profileId == null) {
        throw Exception('프로필을 찾을 수 없어요. 프로필을 먼저 선택해주세요.');
      }

      // ✅ 조회 범위 정의
      final now = DateTime.now();
      final todayStart = DateTime(now.year, now.month, now.day);

      final fromDaily = todayStart; // 오늘 00:00 ~ 지금
      final toDaily = now;

      final fromWeekly = todayStart.subtract(const Duration(days: 6)); // 최근 7일
      final toWeekly = now;

      final fromMonthly = todayStart.subtract(
          const Duration(days: 365)); // 최근 12개월
      final toMonthly = now;

      // ✅ from/to 넣어서 호출
      final results = await Future.wait([
        StatsService.fetchPostureStats(
          profileId: profileId,
          period: 'daily',
          from: fromDaily,
          to: toDaily,
        ),
        StatsService.fetchPostureStats(
          profileId: profileId,
          period: 'weekly',
          from: fromWeekly,
          to: toWeekly,
        ),
        StatsService.fetchPostureStats(
          profileId: profileId,
          period: 'monthly',
          from: fromMonthly,
          to: toMonthly,
        ),
      ]);

      setState(() {
        _daily = results[0];
        _weekly = results[1];
        _monthly = results[2];
      });
    } catch (e) {
      setState(() => _error = '통계 데이터를 불러오지 못했어요.\n(${e.toString()})');
    } finally {
      if (mounted) setState(() => _isLoading = false);
    }
  }

  // ── HELPERS ──────────────────────────────────────────────────────────────────
  String _formatDuration(int seconds) {
    if (seconds <= 0) return '0분';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return '${h}시간 ${m}분';
    return '${m}분';
  }

  // 일(0)~토(6) 인덱스 기준으로 이번 주 해당 날짜 (기준: 일요일 시작)
  DateTime _dateForWeekIndex(int index) {
    final now = DateTime.now();
    final startOfWeek = now.subtract(Duration(days: now.weekday % 7)); // Sun=0
    return DateTime(startOfWeek.year, startOfWeek.month, startOfWeek.day)
        .add(Duration(days: index));
  }

  // 배열 왼쪽 회전: k칸
  List<T> _rotateLeft<T>(List<T> list, int k) {
    if (list.isEmpty) return list;
    final r = k % list.length;
    if (r == 0) return List<T>.from(list);
    return [...list.sublist(r), ...list.sublist(0, r)];
  }

  // 카드 오른쪽 상단 범례
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

  // ── BUILD ────────────────────────────────────────────────────────────────────
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
                Text(
                  _error!,
                  style: const TextStyle(color: Colors.white70),
                  textAlign: TextAlign.center,
                ),
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
              // ── 카드 1: 일간 도넛 ──
              RectCard(
                elevated: true,
                outlineColor: Colors.white.withOpacity(0.16),
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        const Expanded(
                          child: Text(
                            '일간 자세 비율',
                            style: TextStyle(
                              color: Colors.white,
                              fontSize: 16,
                              fontWeight: FontWeight.w600,
                            ),
                          ),
                        ),
                        _legendMini(),
                      ],
                    ),
                    const SizedBox(height: 12),
                    SizedBox(height: 220, child: _AveragePieChart()),
                  ],
                ),
              ),
              const SizedBox(height: 20),

              // ── 카드 2: 주간 스택 바 ──
              RectCard(
                elevated: true,
                outlineColor: Colors.white.withOpacity(0.16),
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        const Expanded(
                          child: Text(
                            '주간 자세 통계',
                            style: TextStyle(
                              color: Colors.white,
                              fontSize: 16,
                              fontWeight: FontWeight.w600,
                            ),
                          ),
                        ),
                        _legendMini(),
                      ],
                    ),
                    const SizedBox(height: 12),
                    SizedBox(height: 240, child: _StackedWeeklyBarChart()),
                  ],
                ),
              ),
              const SizedBox(height: 20),

              // ── 카드 3: 월별 트렌드 ──
              RectCard(
                elevated: true,
                outlineColor: Colors.white.withOpacity(0.16),
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        const Expanded(
                          child: Text(
                            '연간 월별 자세 추이',
                            style: TextStyle(
                              color: Colors.white,
                              fontSize: 16,
                              fontWeight: FontWeight.w600,
                            ),
                          ),
                        ),
                        _legendMini(),
                      ],
                    ),
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

  // ── CHARTS ───────────────────────────────────────────────────────────────────
  Widget _AveragePieChart() {
    if (_daily.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    int totalGood = 0;
    int totalBad = 0;
    for (final s in _daily) {
      if (s.validPosture) {
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
    if (_weekly.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    // 일(0)~토(6) 누적 (초)
    final dayWise = List.generate(7, (_) => {'good': 0, 'bad': 0});
    for (final s in _weekly) {
      final w = s.startAt.weekday % 7; // Mon=1..Sun=7 -> 1..6,0(일)
      if (s.validPosture) {
        dayWise[w]['good'] = dayWise[w]['good']! + s.durationSeconds;
      } else {
        dayWise[w]['bad'] = dayWise[w]['bad']! + s.durationSeconds;
      }
    }

    final hasData = dayWise.any((d) =>
    (d['good'] ?? 0) > 0 || (d['bad'] ?? 0) > 0);
    if (!hasData) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    // ✅ 오늘을 맨 오른쪽(인덱스 6)에 두기 위해 왼쪽으로 (오늘+1)칸 회전
    final todayIdx = DateTime
        .now()
        .weekday % 7; // 0=일 ... 6=토
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
                    final i = response!.spot!.touchedBarGroupIndex; // 회전 후 인덱스
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
                      // ✅ 회전 후 인덱스를 실제 요일 인덱스로 역매핑
                      final actualIdx = (_touchedWeekIndex! + shift) % 7;
                      final dateText = DateFormat('M월 d일').format(
                          _dateForWeekIndex(actualIdx));
                      return Text(
                        '$dateText 통계',
                        style: const TextStyle(
                            color: Colors.white, fontWeight: FontWeight.bold),
                      );
                    }),
                    const SizedBox(height: 6),
                    Row(
                      children: [
                        const Icon(Icons.square, color: errorRed, size: 10),
                        const SizedBox(width: 6),
                        Expanded(
                          child: Text(
                            _formatDuration(_touchedWeekBad),
                            style: const TextStyle(color: Colors.white),
                          ),
                        ),
                      ],
                    ),
                    Row(
                      children: [
                        const Icon(Icons.square, color: primaryBlue, size: 10),
                        const SizedBox(width: 6),
                        Expanded(
                          child: Text(
                            _formatDuration(_touchedWeekGood),
                            style: const TextStyle(color: Colors.white),
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
        ],
      );
    });
  }

// ── CHART: 월별 트렌드 ───────────────────────────────────────────
  Widget _MonthlyTrendChart() {
    if (_monthly.isEmpty) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    // 월별 누적(초) — 기본 순서: 1월..12월
    final List<int> goodSec = List.filled(12, 0);
    final List<int> badSec = List.filled(12, 0);

    for (final s in _monthly) {
      final mIdx = s.startAt.month - 1; // 0~11
      if (s.validPosture) {
        goodSec[mIdx] += s.durationSeconds;
      } else {
        badSec[mIdx] += s.durationSeconds;
      }
    }

    // 회전: 왼쪽=다음달, 오른쪽=현재달
    final now = DateTime.now();
    final shift = now.month % 12;
    final List<String> labels = List.generate(12, (i) => '${i + 1}월');

    final rotatedLabels = _rotateLeft(labels, shift);
    final rotatedGoodSec = _rotateLeft(goodSec, shift);
    final rotatedBadSec = _rotateLeft(badSec, shift);

    // 데이터 존재 여부 (전체가 0이면 차트 숨김)
    final hasGood = rotatedGoodSec.any((v) => v > 0);
    final hasBad = rotatedBadSec.any((v) => v > 0);
    if (!hasGood && !hasBad) {
      return const Center(child: Text('데이터 없음', style: _label));
    }

    // ✅ spots 만들 때 0인 달은 추가하지 않음 (0선/0점 제거)
    final goodSpots = <FlSpot>[];
    final badSpots = <FlSpot>[];
    for (int i = 0; i < 12; i++) {
      final g = rotatedGoodSec[i];
      final b = rotatedBadSec[i];
      if (g > 0) goodSpots.add(FlSpot(i.toDouble(), g / 3600.0));
      if (b > 0) badSpots.add(FlSpot(i.toDouble(), b / 3600.0));
    }

    // 시리즈: 데이터 있는 것만
    final List<LineChartBarData> bars = [];
    if (goodSpots.isNotEmpty) {
      bars.add(LineChartBarData(
        isCurved: false,
        // 오버슈팅 방지
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
              // 음수로 안 내려가게
              gridData: FlGridData(show: false),
              borderData: FlBorderData(show: false),
              titlesData: FlTitlesData(
                bottomTitles: AxisTitles(
                  sideTitles: SideTitles(
                    showTitles: true, interval: 1,
                    getTitlesWidget: (v, _) {
                      final i = v.toInt().clamp(0, 11);
                      return Text(rotatedLabels[i], style: _label.copyWith(
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
                        _touchedMonthGood = rotatedGoodSec[idx];
                        _touchedMonthBad = rotatedBadSec[idx];
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
                    Text('${rotatedLabels[_touchedMonth!]} 통계',
                        style: const TextStyle(
                            color: Colors.white, fontWeight: FontWeight.bold)),
                    const SizedBox(height: 6),
                    Row(children: [
                      const Icon(Icons.square, color: errorRed, size: 10),
                      const SizedBox(width: 6),
                      Expanded(child: Text(
                          _formatDuration(rotatedBadSec[_touchedMonth!]),
                          style: const TextStyle(color: Colors.white))),
                    ]),
                    Row(children: [
                      const Icon(Icons.square, color: primaryBlue, size: 10),
                      const SizedBox(width: 6),
                      Expanded(child: Text(
                          _formatDuration(rotatedGoodSec[_touchedMonth!]),
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