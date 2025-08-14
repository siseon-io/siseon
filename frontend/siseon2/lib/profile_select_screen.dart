import 'dart:async';
import 'dart:convert';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';
import '/root_screen.dart';
import '/pages/profile_create.dart';

class AppColors {
  static const background = Color(0xFF0D1117);
  static const card = Color(0xFF161B22);
  static const cardBorder = Color(0xFF334155);
  static const primary = Color(0xFF3B82F6);
  static const text = Colors.white;
  static const textSub = Colors.white70;
}

class ProfileSelectScreen extends StatefulWidget {
  // ★ 뒤로가기 정책: 로그인 플로우=false, 설정 진입=true
  final bool allowBack;
  const ProfileSelectScreen({super.key, this.allowBack = false});

  @override
  State<ProfileSelectScreen> createState() => _ProfileSelectScreenState();
}

class _ProfileSelectScreenState extends State<ProfileSelectScreen> {
  List<Map<String, dynamic>> _profiles = [];
  bool _isLoading = true;

  int? _selectedIndex;
  bool _isPushing = false;

  static const double _scaleSelected = 1.05;
  static const double _scaleOthers = 0.94;
  static const double _fadeOthers = 0.20;
  static const Duration _cardAnim = Duration(milliseconds: 200);

  @override
  void initState() {
    super.initState();
    fetchProfiles();
    SystemSound.play(SystemSoundType.click);
  }

  Future<void> fetchProfiles() async {
    final token = await AuthService.getValidAccessToken();
    if (token == null) {
      _showError('로그인이 필요합니다.');
      return;
    }
    try {
      final res = await http.get(
        Uri.parse('https://i13b101.p.ssafy.io/siseon/api/profile'),
        headers: {'Authorization': 'Bearer $token'},
      );
      if (!mounted) return;
      if (res.statusCode == 200) {
        final List data = jsonDecode(utf8.decode(res.bodyBytes));
        setState(() {
          _profiles = List<Map<String, dynamic>>.from(data);
          _isLoading = false;
        });
      } else {
        _showError('프로필 조회 실패 (${res.statusCode})');
      }
    } catch (e) {
      _showError('예외 발생: $e');
    }
  }

  void _showError(String msg) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  void _onAddPressed() async {
    if (_isPushing) return;
    setState(() {
      _isPushing = true;
      _selectedIndex = null; // 🔑 선택 상태 초기화
    });
    final result = await Navigator.push(
      context,
      _fadeScaleRoute(const ProfileCreateScreen(), 420),
    );
    if (!mounted) return;
    setState(() => _isPushing = false);
    if (result == true) {
      await fetchProfiles();
      if (!mounted) return;
      setState(() => _selectedIndex = null); // 🔑 복귀 후에도 초기화
    }
  }

  ImageProvider? _provider(String? url) {
    if (url == null) return null;
    if (url.startsWith('http')) return NetworkImage(url);
    if (url.startsWith('assets/')) return AssetImage(url);
    return null;
  }

  /// 탭 → 히어로(그리드→센터) → [확대] → [0.5s 대기] → 좌상단 축소 → 홈
  Future<void> _onProfileSelected(Map<String, dynamic> profile) async {
    if (_isPushing) return;
    await ProfileCacheService.saveProfile(profile);

    // FCM 등록 fire-and-forget
    final pid = profile['id'];
    final auth = await AuthService.getValidAccessToken();
    final fcm = await FirebaseMessaging.instance.getToken();
    if (pid != null && auth != null && fcm != null) {
      unawaited(http.post(
        Uri.parse(
            'https://i13b101.p.ssafy.io/siseon/api/push/register?profileId=$pid&fcmToken=$fcm'),
        headers: {'Authorization': 'Bearer $auth'},
      ));
    }

    if (!mounted) return;
    setState(() => _isPushing = true);

    final tag = 'avatar_$pid';
    final img = _provider(profile['imageUrl']);

    await Navigator.pushReplacement(
      context,
      _avatarMorphRoute(tag: tag, image: img),
    );
  }

  // 모핑(확대→대기→코너縮)의 중간 라우트
  PageRouteBuilder _avatarMorphRoute({
    required String tag,
    ImageProvider? image,
  }) {
    const int heroMs = 600; // 히어로 이동 시간(그리드→센터)
    return PageRouteBuilder(
      transitionDuration: Duration(milliseconds: heroMs),
      opaque: true,
      pageBuilder: (context, anim, __) {
        return AvatarMorphScreen(
          heroTag: tag,
          image: image,
          startDelayMs: heroMs,          // 히어로 끝나고 내부 애니 시작
          pauseAfterExpandMs: 500,       // ✅ 확대 후 0.5초 대기
          // ★ onDone은 AvatarMorphScreen의 context로 실행
          onDone: (ctx) async {
            if (!ctx.mounted) return;
            await Navigator.pushReplacement(
              ctx,
              _fadeScaleRoute(const RootScreen(), 600),
            );
          },
        );
      },
    );
  }

  // 일반 페이지 페이드+스케일
  PageRouteBuilder _fadeScaleRoute(Widget page, int ms) {
    return PageRouteBuilder(
      transitionDuration: Duration(milliseconds: ms),
      pageBuilder: (_, anim, __) => FadeTransition(
        opacity: CurvedAnimation(parent: anim, curve: Curves.easeOutCubic),
        child: ScaleTransition(
          scale: Tween<double>(begin: 0.97, end: 1.0).animate(
            CurvedAnimation(parent: anim, curve: Curves.easeOutCubic),
          ),
          child: page,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final list = [..._profiles];
    if (_profiles.length < 4) list.add({'isAddButton': true, 'id': 'add'});

    // 🔑 Add가 선택된 경우엔 다른 카드 흐림/축소 비활성화
    final bool selectedIsAdd = _selectedIndex != null &&
        _selectedIndex! >= 0 &&
        _selectedIndex! < list.length &&
        (list[_selectedIndex!]['isAddButton'] == true);
    final bool dimOthers = _selectedIndex != null && !selectedIsAdd;

    return WillPopScope(
      // ★ 애니/전환 중 뒤로가기 차단 + 로그인 플로우에서는 종료 방지
      onWillPop: () async {
        if (_isPushing) return false; // 전환 중 크래시 방지
        if (widget.allowBack) {
          return true; // 설정에서 진입: 정상 뒤로가기
        } else {
          // 로그인 직후: 실수 종료 방지
          HapticFeedback.lightImpact();
          if (mounted) {
            ScaffoldMessenger.of(context).showSnackBar(
              const SnackBar(content: Text('프로필을 선택해 주세요.')),
            );
          }
          return false;
        }
      },
      child: Scaffold(
        backgroundColor: AppColors.background,
        appBar: AppBar(
          title: const Text('프로필 선택'),
          backgroundColor: AppColors.background,
          foregroundColor: AppColors.text,
          elevation: 0,
          centerTitle: true,
          automaticallyImplyLeading: widget.allowBack, // ★ 뒤로가기 아이콘 표시 정책
        ),
        body: _isLoading
            ? const Center(child: CircularProgressIndicator(color: Colors.white))
            : Padding(
          padding: const EdgeInsets.all(20),
          child: GridView.builder(
            itemCount: list.length,
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
              crossAxisCount: 2,
              crossAxisSpacing: 24,
              mainAxisSpacing: 24,
              childAspectRatio: 0.85,
            ),
            itemBuilder: (context, i) {
              final p = list[i];
              final bool add = p['isAddButton'] == true;
              final bool sel = _selectedIndex == i;

              final double scale =
              sel ? _scaleSelected : (dimOthers ? _scaleOthers : 1.0);
              final double opacity =
              sel ? 1.0 : (dimOthers ? _fadeOthers : 1.0);

              final String tag = 'avatar_${p['id']}';
              final image = _provider(p['imageUrl']);

              return GestureDetector(
                onTap: () async {
                  if (_isPushing) return;
                  HapticFeedback.selectionClick();

                  if (add) {
                    setState(() => _selectedIndex = null); // 🔑 Add 탭 시 선택 해제
                    return _onAddPressed();
                  }

                  setState(() => _selectedIndex = i);
                  await _onProfileSelected(p);
                },
                child: AnimatedScale(
                  scale: scale.toDouble(),
                  duration: _cardAnim,
                  curve: Curves.easeOutCubic,
                  child: AnimatedOpacity(
                    opacity: opacity,
                    duration: _cardAnim,
                    curve: Curves.easeOutCubic,
                    child: _ProfileCard(
                      name: add ? '프로필 추가' : (p['name'] ?? ''),
                      heroTag: tag,
                      image: image,
                      showAdd: add,
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}

/// ====== 확대 → (대기) → 코너로 둥글게 줄어드는 중간 화면 ======
class AvatarMorphScreen extends StatefulWidget {
  final String heroTag;
  final ImageProvider? image;
  // ★ 자신의 context를 받도록 변경
  final Future<void> Function(BuildContext ctx) onDone;
  final int startDelayMs;       // 히어로 끝나고 시작 딜레이
  final int pauseAfterExpandMs; // ✅ 확대 후 대기 시간

  const AvatarMorphScreen({
    super.key,
    required this.heroTag,
    required this.image,
    required this.onDone,
    this.startDelayMs = 350,
    this.pauseAfterExpandMs = 500,
  });

  @override
  State<AvatarMorphScreen> createState() => _AvatarMorphScreenState();
}

class _AvatarMorphScreenState extends State<AvatarMorphScreen>
    with SingleTickerProviderStateMixin {
  static const int _baseMorphMs = 1400;       // 확대+축소 본연 시간
  static const double _split = 0.40;          // 40%까지 '확대' 구간
  late final AnimationController _c;

  @override
  void initState() {
    super.initState();
    final totalMs = _baseMorphMs + widget.pauseAfterExpandMs;
    _c = AnimationController(
      vsync: this,
      duration: Duration(milliseconds: totalMs),
    )..addStatusListener((s) async {
      if (s == AnimationStatus.completed && mounted) {
        // ★ 자기 context로 내비 (팝된 context 접근 방지)
        await widget.onDone(context);
      }
    });

    // 히어로 이동이 끝난 뒤에 모핑 시작
    Future.delayed(Duration(milliseconds: widget.startDelayMs), () {
      if (mounted) _c.forward();
    });
  }

  @override
  void dispose() {
    _c.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final totalMs = _baseMorphMs + widget.pauseAfterExpandMs;
    final expandMs = (_baseMorphMs * _split).round();     // 확대 구간 시간
    final shrinkMs = _baseMorphMs - expandMs;             // 축소 구간 시간

    return WillPopScope(
      // ★ 애니 중 뒤로가기 차단 (중간 팝으로 인한 크래시 방지)
      onWillPop: () async => false,
      child: Scaffold(
        backgroundColor: Colors.black,
        body: SafeArea(
          minimum: const EdgeInsets.all(12),
          child: AnimatedBuilder(
            animation: _c,
            builder: (_, __) {
              // 컨트롤러 진행(0..1) → 실제 ms → 구간별 진행률로 변환
              final elapsedMs = (_c.value * totalMs);
              double morphT; // 0..1, 0~0.4는 확대, 0.4 고정(대기), 이후 1.0까지 축소

              if (elapsedMs <= expandMs) {
                // 확대(0 → 0.4)
                morphT = _split * (elapsedMs / expandMs);
              } else if (elapsedMs <= expandMs + widget.pauseAfterExpandMs) {
                // ✅ 대기(0.4 고정)
                morphT = _split;
              } else {
                // 축소(0.4 → 1.0)
                final afterPause = elapsedMs - expandMs - widget.pauseAfterExpandMs;
                morphT = _split + (1 - _split) * (afterPause / shrinkMs);
              }

              // 사이즈/모서리/위치 계산
              double box = _segmentLerp(72, 180, 180, 44, morphT, split: _split);
              final double r = morphT < _split
                  ? 18
                  : _lerp(18, box / 2, (morphT - _split) / (1 - _split));
              Alignment align = morphT < _split
                  ? Alignment.center
                  : Alignment.lerp(
                Alignment.center,
                Alignment.topLeft,
                (morphT - _split) / (1 - _split),
              )!;
              final double bgOpacity = _lerp(0.65, 0.35, morphT);

              return Stack(
                children: [
                  // 어두운 배경 + 살짝 블러 효과
                  Positioned.fill(
                    child: Opacity(
                      opacity: bgOpacity,
                      child: BackdropFilter(
                        filter: ImageFilter.blur(sigmaX: 2, sigmaY: 2),
                        child: Container(color: Colors.black),
                      ),
                    ),
                  ),

                  // 아바타 모핑
                  Align(
                    alignment: align,
                    child: Hero(
                      tag: widget.heroTag,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(r),
                        child: Container(
                          width: box,
                          height: box,
                          color: const Color(0xFF1F2937),
                          child: widget.image != null
                              ? Image(image: widget.image!, fit: BoxFit.cover)
                              : const Icon(Icons.person,
                              color: Colors.white30, size: 56),
                        ),
                      ),
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
  }

  // piecewise lerp helper
  double _segmentLerp(double a1, double b1, double a2, double b2, double t,
      {double split = 0.5}) {
    if (t <= split) {
      return _lerp(a1, b1, t / split);
    } else {
      return _lerp(a2, b2, (t - split) / (1 - split));
    }
  }

  double _lerp(double a, double b, double t) => a + (b - a) * t.clamp(0, 1);
}

/// ====== 그리드 카드 ======
class _ProfileCard extends StatelessWidget {
  final String name;
  final String heroTag;
  final ImageProvider? image;
  final bool showAdd;

  const _ProfileCard({
    required this.name,
    required this.heroTag,
    required this.image,
    required this.showAdd,
  });

  @override
  Widget build(BuildContext context) {
    return AnimatedContainer(
      duration: const Duration(milliseconds: 200),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: AppColors.card,
        borderRadius: BorderRadius.circular(24),
        border: Border.all(color: AppColors.cardBorder, width: 1),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.25),
            blurRadius: 12,
            offset: const Offset(0, 6),
          ),
        ],
      ),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Hero(
            tag: heroTag,
            child: ClipRRect(
              borderRadius: BorderRadius.circular(18),
              child: Container(
                width: 72,
                height: 72,
                color: const Color(0xFF1F2937),
                child: image != null
                    ? Image(image: image!, fit: BoxFit.cover)
                    : const Icon(Icons.person, color: Colors.white30, size: 36),
              ),
            ),
          ),
          const SizedBox(height: 10),
          Text(
            name,
            style: const TextStyle(
              fontFamily: 'Pretendard',
              fontSize: 16,
              color: AppColors.text,
              fontWeight: FontWeight.w600,
            ),
            overflow: TextOverflow.ellipsis,
          ),
          if (showAdd)
            const Padding(
              padding: EdgeInsets.only(top: 6),
              child: Icon(Icons.add_circle_outline,
                  color: AppColors.primary, size: 24),
            ),
        ],
      ),
    );
  }
}
