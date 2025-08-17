import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'package:siseon2/pages/home_screen.dart';
import 'package:siseon2/models/control_mode.dart';

// 전역 색상 상수
const Color _kBg = Color(0xFF0D1117);
const Color _kCard = Color(0xFF161B22);
const Color _kPrimary = Color(0xFF3B82F6);
const Color _kText = Colors.white;
const Color _kTextSub = Colors.white70;

class TutorialScreen extends StatefulWidget {
  final int profileId;
  final bool fromMenu; // 홈/설정에서 "다시보기"로 온 경우

  const TutorialScreen({
    super.key,
    required this.profileId,
    this.fromMenu = false,
  });

  @override
  State<TutorialScreen> createState() => _TutorialScreenState();

  /// (선택) 프로필별 최초 1회 노출 여부
  static Future<bool> shouldShowForProfile(int profileId) async {
    final prefs = await SharedPreferences.getInstance();
    return !(prefs.getBool('tutorial_seen_$profileId') ?? false);
  }
}

class _TutorialScreenState extends State<TutorialScreen> {
  final PageController _controller = PageController();
  int _index = 0;
  bool _isFinishing = false; // 중복 네비 방지

  // NOTE: 리스트에서 const 제거 (핫리로드 호환)
  late final List<_TourStep> _pages = <_TourStep>[
    // 1) 홈
    _TourStep(
      icon: Icons.home_rounded,
      title: '홈',
      desc:
      '연결 상태, 오늘의 자세 요약, 빠른 진입 기능이 모여 있어요.\n문제 발생 시 카드로 바로 안내돼요.',
    ),
    // 2) 전원 / AI 모드
    _TourStep(
      icon: Icons.toggle_on_rounded,
      title: '전원 / AI 모드',
      desc:
      '홈 상단 우측의 전원 스위치를 켜면 AI 모드가 활성화돼요.\n'
          'AI 모드는 자세를 자동 분석해 기기를 제어하고, OFF로 내리면 전체 기능이 중지돼요.\n'
          '※ 기기가 등록되어 있어야 켤 수 있어요.',
    ),
    // 3) 기기 등록 & 연결  — 홈과 동일한 아이콘(체인/블루투스)
    _TourStep.device(
      title: '기기 등록 & 연결',
      desc:
      '홈 상단의 작은 카드에서 등록 아이콘으로 등록하고,\n'
          '블루투스 아이콘으로 블루투스를 검색해 연결할 수 있어요.\n'
          '연결되면 파란색 블루투스 아이콘이 표시돼요.',
    ),
    // 4) 수동 모드 — 폰 가로(이모지 느낌) + 설명 문구 수정
    _TourStep(
      emoji: '📱', // 가로 느낌을 주고 싶으면 rotateEmojiQuarter=true 사용
      rotateEmojiQuarter: true,
      title: '수동 모드',
      desc:
      '조이스틱으로 모니터를 미세 조정합니다.\n'
          '블루투스로 즉시 모니터암을 지정합니다.',
    ),
    // 5) 자세 감지 & 알림 — 주기/알림 조건 수정
    _TourStep(
      icon: Icons.health_and_safety_rounded,
      title: '자세 감지 & 알림',
      desc:
      '1분마다 스냅샷을 분석합니다.\n'
          '30분간 자세가 나쁘면 교정 알림이 오고,\n'
          '30분 동안 자세가 올바르며 자세가 비슷하면 프리셋 제안 푸시 알림이 와요.',
    ),
    // 6) 통계
    _TourStep(
      icon: Icons.bar_chart_rounded,
      title: '통계',
      desc:
      '일/주/월 그래프로 올바른/나쁜 자세 비율을 확인해요.\n스냅샷에서 교정 팁도 확인 가능!',
    ),
    // 7) 챗봇
    _TourStep(
      icon: Icons.smart_toy_rounded,
      title: '챗봇 SEONY',
      desc:
      '기능 설명, 문제 해결, 교정 팁을 대화로 제공해요.\n예) “프리셋 저장하는 법 알려줘”.',
    ),
  ];

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Future<void> _markSeen() async {
    if (widget.profileId <= 0) return;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setBool('tutorial_seen_${widget.profileId}', true);
  }

  Future<void> _finish() async {
    if (_isFinishing) return;
    _isFinishing = true;

    await _markSeen();
    if (!mounted) return;

    if (widget.fromMenu) {
      Navigator.pop(context); // 다시보기면 뒤로
    } else {
      // 홈으로 바로 진입 (네임드 라우트 X)
      Navigator.of(context).pushAndRemoveUntil(
        MaterialPageRoute(
          builder: (_) => HomeScreen(
            currentMode: ControlMode.off,
            onAiModeSwitch: () {},
            onGoToProfile: () {},
            onModeChange: (_) {},
          ),
        ),
            (route) => false,
      );
    }
  }

  void _next() {
    final bool isLast = _index >= _pages.length - 1;
    if (isLast) {
      _finish();
    } else {
      _controller.nextPage(
        duration: const Duration(milliseconds: 240),
        curve: Curves.easeOut,
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final bool isLast = _index >= _pages.length - 1;

    return Scaffold(
      backgroundColor: _kBg,
      body: SafeArea(
        child: Column(
          children: [
            // 상단 우측 "건너뛰기"
            Align(
              alignment: Alignment.centerRight,
              child: TextButton(
                onPressed: _finish,
                style: TextButton.styleFrom(
                  foregroundColor: Colors.redAccent,
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                ),
                child: const Text('건너뛰기'),
              ),
            ),

            // 페이지뷰
            Expanded(
              child: PageView.builder(
                controller: _controller,
                itemCount: _pages.length,
                onPageChanged: (i) => setState(() => _index = i),
                itemBuilder: (_, i) {
                  final step = _pages[i];
                  return Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 24),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Container(
                          decoration: BoxDecoration(
                            color: _kCard,
                            borderRadius: BorderRadius.circular(28),
                          ),
                          padding: const EdgeInsets.all(24),
                          child: _StepVisual(step: step),
                        ),
                        const SizedBox(height: 24),
                        Text(
                          step.title,
                          textAlign: TextAlign.center,
                          style: const TextStyle(
                            fontSize: 24,
                            fontWeight: FontWeight.w700,
                            color: _kText,
                          ),
                        ),
                        const SizedBox(height: 12),
                        Text(
                          step.desc,
                          textAlign: TextAlign.center,
                          style: const TextStyle(
                            fontSize: 15,
                            height: 1.5,
                            color: _kTextSub,
                          ),
                        ),
                      ],
                    ),
                  );
                },
              ),
            ),

            // 하단 인디케이터 + 버튼
            Padding(
              padding: const EdgeInsets.fromLTRB(24, 8, 24, 24),
              child: Column(
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: List.generate(_pages.length, (i) {
                      final active = i == _index;
                      return AnimatedContainer(
                        duration: const Duration(milliseconds: 200),
                        margin: const EdgeInsets.symmetric(horizontal: 4),
                        height: 8,
                        width: active ? 20 : 8,
                        decoration: BoxDecoration(
                          color: active ? _kPrimary : _kTextSub.withOpacity(0.3),
                          borderRadius: BorderRadius.circular(8),
                        ),
                      );
                    }),
                  ),
                  const SizedBox(height: 16),
                  SizedBox(
                    width: double.infinity,
                    child: ElevatedButton(
                      onPressed: _next,
                      style: ElevatedButton.styleFrom(
                        backgroundColor: _kPrimary,
                        foregroundColor: Colors.white,
                        minimumSize: const Size(double.infinity, 52),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14),
                        ),
                      ),
                      child: Text(isLast ? '시작하기' : '다음'),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

/// 튜토리얼 단계 데이터
class _TourStep {
  final IconData? icon;          // 기본: 머티리얼 아이콘
  final String? emoji;           // 이모지 사용 시
  final bool rotateEmojiQuarter; // 📱을 가로로 표현하고 싶을 때
  final bool isDeviceStep;       // 기기 등록&연결(특수 레이아웃)
  final String title;
  final String desc;

  _TourStep({
    this.icon,
    this.emoji,
    this.rotateEmojiQuarter = false,
    this.isDeviceStep = false,
    required this.title,
    required this.desc,
  });

  // 기기 등록 & 연결 전용 생성자
  factory _TourStep.device({
    required String title,
    required String desc,
  }) {
    return _TourStep(
      isDeviceStep: true,
      title: title,
      desc: desc,
    );
  }
}

/// 상단 비주얼 렌더러
class _StepVisual extends StatelessWidget {
  final _TourStep step;
  const _StepVisual({required this.step});

  @override
  Widget build(BuildContext context) {
    // 기기 등록 & 연결: 홈과 동일한 아이콘을 나란히 표시
    if (step.isDeviceStep) {
      return Row(
        mainAxisSize: MainAxisSize.min,
        children: const [
          Icon(Icons.link, size: 48, color: _kTextSub),               // 등록(체인)
          SizedBox(width: 18),
          Icon(Icons.bluetooth, size: 48, color: _kTextSub),          // 스캔/연결
          SizedBox(width: 18),
          Icon(Icons.bluetooth_connected, size: 48, color: _kPrimary) // 연결(파란색)
        ],
      );
    }

    // 이모지 우선
    if (step.emoji != null && step.emoji!.isNotEmpty) {
      final child = Text(
        step.emoji!,
        style: const TextStyle(fontSize: 72),
      );
      if (step.rotateEmojiQuarter) {
        return Transform.rotate(
          angle: 1.57079632679, // 90°
          child: child,
        );
      }
      return child;
    }

    // 기본 아이콘
    return Icon(step.icon ?? Icons.info_outline, size: 96, color: _kPrimary);
  }
}
