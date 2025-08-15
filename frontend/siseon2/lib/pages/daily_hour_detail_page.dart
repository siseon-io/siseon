import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:fl_chart/fl_chart.dart';
import 'dart:convert'; // utf8/latin1 복구용

import 'package:siseon2/models/slot_data.dart';
import 'package:siseon2/widgets/rect_card.dart';

// ── THEME (StatsPage와 톤 맞춤)
const _bg = Color(0xFF0D1117);
const _primary = Color(0xFF3B82F6);
const _error = Color(0xFFF87171);
const _label = TextStyle(color: Colors.white70, fontSize: 12);

// ── (중첩 금지) 시간 버킷 모델: 파일 최상위에 둔다!
class _HourBucket {
  int goodSec = 0;
  int badSec = 0;
  final Map<String, int> reasonSec = {}; // label -> seconds
}

class DailyHourDetailPage extends StatefulWidget {
  final List<PostureStatsMinute> dailyMins;
  const DailyHourDetailPage({Key? key, required this.dailyMins}) : super(key: key);

  @override
  State<DailyHourDetailPage> createState() => _DailyHourDetailPageState();
}

class _DailyHourDetailPageState extends State<DailyHourDetailPage> {
  late final List<_HourBucket> _buckets;

  @override
  void initState() {
    super.initState();
    _buckets = _buildHourBuckets(widget.dailyMins);
  }

  // ===== 모지박(인코딩 깨짐) 복구 =====
  String _fixKoreanIfGarbled(String s) {
    final looksGarbled =
        RegExp(r'(Ã.|Â.|ì.|í.|ë.|ê.|°|±|²|³|¼|½|¾)').hasMatch(s) && !RegExp(r'[가-힣]').hasMatch(s);
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

  List<_HourBucket> _buildHourBuckets(List<PostureStatsMinute> mins) {
    final buckets = List.generate(24, (_) => _HourBucket());

    for (final m in mins) {
      final hour = m.endAt.toLocal().hour.clamp(0, 23);
      final sec = (m.durationSeconds <= 0) ? 60 : m.durationSeconds;

      // valid 판정 수집(서버 필드 호환)
      bool? valid = m.validPosture;
      try {
        final v2 = (m as dynamic).valid;
        if (v2 is bool) valid = v2;
      } catch (_) {}
      try {
        final br0 = (m as dynamic).badReasons;
        final v3 = (br0 as dynamic).valid;
        if (v3 is bool) valid = v3;
      } catch (_) {}

      if (valid == true) {
        buckets[hour].goodSec += sec;
        continue;
      } else {
        buckets[hour].badSec += sec;
      }

      // ── 사유 라벨 모으기 (reasons[].label → fallback: summary)
      final labels = <String>[];
      try {
        final br = (m as dynamic).badReasons;

        // 1) reasons[].label
        final rs = (br as dynamic).reasons;
        if (rs is Iterable) {
          for (final r in rs) {
            final lbl = _cleanText((r as dynamic).label?.toString());
            if (lbl.isNotEmpty) labels.add(lbl);
          }
        }

        // 2) fallback: summary "거북목(…)" → 괄호 앞만 추출
        if (labels.isEmpty) {
          final sum = _cleanText((br as dynamic).summary?.toString());
          if (sum.isNotEmpty) {
            labels.addAll(sum
                .split(',')
                .map((e) => e.split('(').first)
                .map(_cleanText)
                .where((e) => e.isNotEmpty));
          }
        }
      } catch (_) {
        // ignore, 아래에서 '기타' 처리
      }

      if (labels.isEmpty) {
        buckets[hour].reasonSec['기타'] = (buckets[hour].reasonSec['기타'] ?? 0) + sec;
      } else {
        // 같은 분에 여러 사유면 동일 분배
        final share = sec ~/ labels.length;
        int remain = sec - share * labels.length;
        for (int i = 0; i < labels.length; i++) {
          final add = share + (i == 0 ? remain : 0);
          final key = _cleanText(labels[i]);
          buckets[hour].reasonSec[key] = (buckets[hour].reasonSec[key] ?? 0) + add;
        }
      }
    }
    return buckets;
  }

  String _fmt(int s) {
    if (s <= 0) return '0분';
    final h = s ~/ 3600, m = (s % 3600) ~/ 60;
    return h > 0 ? '${h}시간 ${m}분' : '${m}분';
  }

  Widget _legendMini() => Row(
    mainAxisSize: MainAxisSize.min,
    children: const [
      Icon(Icons.square, color: _error, size: 10),
      SizedBox(width: 4),
      Text('잘못된 자세', style: _label),
      SizedBox(width: 10),
      Icon(Icons.square, color: _primary, size: 10),
      SizedBox(width: 4),
      Text('올바른 자세', style: _label),
    ],
  );

  @override
  Widget build(BuildContext context) {
    final today = DateFormat('yyyy.MM.dd (E)', 'ko_KR').format(DateTime.now());
    final bottomSafe = MediaQuery.of(context).padding.bottom;

    return Scaffold(
      backgroundColor: _bg,
      appBar: AppBar(
        backgroundColor: _bg,
        elevation: 0,
        iconTheme: const IconThemeData(color: Colors.white),
        title:
        const Text('하루 타임라인', style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
      ),
      body: ListView(
        // ✅ 하단이 시스템 영역에 가려지지 않도록 SafeArea만큼 추가
        padding: EdgeInsets.fromLTRB(16, 16, 16, 16 + bottomSafe),
        children: [
          Row(children: [
            Expanded(
                child:
                Text(today, style: const TextStyle(color: Colors.white70, fontSize: 12))),
            _legendMini(),
          ]),
          const SizedBox(height: 12),

          // 오전 (00~12)
          RectCard(
            elevated: true,
            outlineColor: Colors.white.withOpacity(0.16),
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('오전 (00–12)',
                    style: TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w600)),
                const SizedBox(height: 12),
                SizedBox(
                  height: 240,
                  child: _HourChart(
                    buckets: _buckets,
                    start: 0,
                    end: 12,
                    onTap: _showHourSheet,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 16),

          // 오후 (12~24)
          RectCard(
            elevated: true,
            outlineColor: Colors.white.withOpacity(0.16),
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('오후 (12–24)',
                    style: TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.w600)),
                const SizedBox(height: 12),
                SizedBox(
                  height: 240,
                  child: _HourChart(
                    buckets: _buckets,
                    start: 12,
                    end: 24,
                    onTap: _showHourSheet,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  void _showHourSheet(int hour) {
    final b = _buckets[hour];
    final reasons = b.reasonSec.entries.toList()..sort((a, b) => b.value.compareTo(a.value));

    showModalBottomSheet(
      context: context,
      // ✅ 시스템 하단 영역까지 침범하지 않게
      isScrollControlled: true,
      backgroundColor: Colors.transparent, // 내부 컨테이너에 색/라운드 적용
      builder: (ctx) {
        final bottomSafe = MediaQuery.of(ctx).padding.bottom;
        return SafeArea(
          top: false,
          child: Container(
            decoration: const BoxDecoration(
              color: Color(0xFF0B0F14),
              borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
            ),
            padding: EdgeInsets.fromLTRB(16, 12, 16, 16 + bottomSafe), // ✅ 하단 여유
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(children: [
                  Expanded(
                    child: Text(
                      '${hour.toString().padLeft(2, '0')}:00 ~ ${((hour + 1) % 24).toString().padLeft(2, '0')}:00',
                      style: const TextStyle(
                          color: Colors.white, fontSize: 16, fontWeight: FontWeight.w700),
                    ),
                  ),
                  IconButton(
                    onPressed: () => Navigator.pop(ctx),
                    icon: const Icon(Icons.close, color: Colors.white70),
                  ),
                ]),
                const SizedBox(height: 8),
                Row(children: [
                  const Icon(Icons.square, color: _error, size: 12),
                  const SizedBox(width: 6),
                  Expanded(
                      child:
                      Text('나쁨: ${_fmt(b.badSec)}', style: const TextStyle(color: Colors.white))),
                ]),
                Row(children: [
                  const Icon(Icons.square, color: _primary, size: 12),
                  const SizedBox(width: 6),
                  Expanded(
                      child:
                      Text('좋음: ${_fmt(b.goodSec)}', style: const TextStyle(color: Colors.white))),
                ]),
                const SizedBox(height: 12),
                if (reasons.isEmpty)
                  const Text('나쁜자세 세부 원인 없음', style: TextStyle(color: Colors.white70))
                else ...[
                  const Text('나쁜자세 원인별',
                      style: TextStyle(color: Colors.white70, fontWeight: FontWeight.w600)),
                  const SizedBox(height: 8),
                  // 내용이 길어질 경우도 대비해 살짝 스크롤 가능하도록
                  Flexible(
                    child: ListView.builder(
                      shrinkWrap: true,
                      physics: const ClampingScrollPhysics(),
                      itemCount: reasons.length,
                      itemBuilder: (_, i) {
                        final e = reasons[i];
                        final label = _cleanText(e.key);
                        final sec = e.value;
                        final pct =
                        b.badSec > 0 ? ((sec / b.badSec) * 100).toStringAsFixed(0) : '0';
                        return Padding(
                          padding: const EdgeInsets.symmetric(vertical: 6),
                          child: Row(
                            children: [
                              Expanded(
                                  child:
                                  Text(label, style: const TextStyle(color: Colors.white))),
                              Text('${_fmt(sec)}  •  $pct%',
                                  style: const TextStyle(color: Colors.white70)),
                            ],
                          ),
                        );
                      },
                    ),
                  ),
                ],
              ],
            ),
          ),
        );
      },
    );
  }
}

// ── 시간대 스택 막대 차트 위젯 (툴팁/오버레이 제거 버전)
class _HourChart extends StatefulWidget {
  final List<_HourBucket> buckets;
  final int start; // inclusive
  final int end; // exclusive
  final void Function(int hour) onTap;

  const _HourChart({
    Key? key,
    required this.buckets,
    required this.start,
    required this.end,
    required this.onTap,
  }) : super(key: key);

  @override
  State<_HourChart> createState() => _HourChartState();
}

class _HourChartState extends State<_HourChart> {
  static const _primary = Color(0xFF3B82F6);
  static const _error = Color(0xFFF87171);
  static const _label = TextStyle(color: Colors.white70, fontSize: 10);

  @override
  Widget build(BuildContext context) {
    final count = widget.end - widget.start;
    double toH(int s) => s / 3600.0;

    return BarChart(
      BarChartData(
        gridData: FlGridData(show: false),
        borderData: FlBorderData(show: false),
        titlesData: FlTitlesData(
          bottomTitles: AxisTitles(
            sideTitles: SideTitles(
              showTitles: true,
              interval: 1,
              reservedSize: 18,
              getTitlesWidget: (v, _) =>
                  Text((widget.start + v.toInt()).toString(), style: _label),
            ),
          ),
          leftTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
          rightTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
          topTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
        ),
        barTouchData: BarTouchData(
          enabled: true,
          handleBuiltInTouches: false, // 기본 터치 처리/툴팁 비활성
          touchCallback: (event, response) {
            if (event is FlTapUpEvent && response?.spot != null) {
              final idx = response!.spot!.touchedBarGroupIndex;
              final hour = widget.start + idx;
              widget.onTap(hour); // 바텀시트만 표시
            }
          },
          touchTooltipData:
          BarTouchTooltipData(getTooltipItem: (_, __, ___, ____) => null), // 툴팁 제거
        ),
        barGroups: List.generate(count, (i) {
          final hour = widget.start + i;
          final b = widget.buckets[hour];
          final g = toH(b.goodSec);
          final d = toH(b.badSec);
          final sum = g + d;
          return BarChartGroupData(x: i, barRods: [
            BarChartRodData(
              toY: sum,
              width: 14,
              borderRadius: BorderRadius.circular(4),
              rodStackItems: [
                BarChartRodStackItem(0, g, _primary),
                BarChartRodStackItem(g, sum, _error),
              ],
            ),
          ]);
        }),
      ),
    );
  }
}
