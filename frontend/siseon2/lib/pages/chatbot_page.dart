// lib/pages/chat/chatbot_page.dart
import 'dart:async'; // ✅ 점 애니메이션/폴링 타이머
import 'dart:convert'; // ✅ AssetManifest 파싱
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart'; // ✅ 진행중 상태 저장

import 'package:siseon2/services/chat_api.dart';
import 'package:siseon2/models/chat_models.dart';
import 'package:siseon2/services/profile_cache_service.dart'; // 사용자 아바타
import 'package:siseon2/services/faq_service.dart'; // ✅ FAQ API

class ChatbotPage extends StatefulWidget {
  final int profileId;

  const ChatbotPage({super.key, required this.profileId});

  @override
  State<ChatbotPage> createState() => _ChatbotPageState();
}

class _ChatbotPageState extends State<ChatbotPage> {
  // THEME
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color bubbleBg = Color(0xFF1F2937);
  static const Color inputBg = Color(0xFF111827);
  static const String assistantDisplayName = 'SEONY';
  static const double avatarSize = 32;

  static const double assistantAvatarSize = 46; // SEONY
  static const double userAvatarSize = 40;      // 사용자

  // STATE
  final List<ChatMessage> _messages = [];
  final TextEditingController _controller = TextEditingController();
  final ScrollController _scroll = ScrollController();
  final _timeFmt = DateFormat('HH:mm');

  final ChatApi _api = ChatApi();
  bool _loading = true;
  bool _sending = false; // ✅ 답변 올 때까지 전송 금지

  // 타이핑 인디케이터 상태
  bool _assistantTyping = false;
  Timer? _typingTimer;
  int _typingDots = 1; // 1~3 사이에서 왕복
  int _typingDir = 1;  // 1이면 증가, -1이면 감소

  // ✅ 자동 새로고침(폴링)용
  Timer? _pendingPoll;      // 답변 도착 감시 타이머
  DateTime? _pendingAt;     // 진행중 메시지 보낸 시각
  String? _pendingText;     // 진행중 메시지 텍스트

  ImageProvider? _userAvatar;
  ImageProvider? _assistantAvatarImg; // ✅ SEONY 프로필 이미지

  // TIME
  DateTime nowKstLocal() {
    final kst = DateTime.now().toUtc().add(const Duration(hours: 9));
    return DateTime(
        kst.year, kst.month, kst.day, kst.hour, kst.minute, kst.second, kst.millisecond, kst.microsecond
    );
  }

  String _formatBubbleTime(DateTime dt) => _timeFmt.format(dt.toLocal());
  bool _sameDay(DateTime a, DateTime b) => a.year == b.year && a.month == b.month && a.day == b.day;
  String _dateChipText(DateTime dt) => DateFormat('MM.dd (E)', 'ko_KR').format(dt);

  @override
  void initState() {
    super.initState();
    _loadAssistantAvatar(); // ✅ SEONY 아바타 로드
    _loadUserAvatar();
    _loadHistory(); // 서버 이력
  }

  @override
  void dispose() {
    _pendingPoll?.cancel();   // ✅ 폴링 정리
    _typingTimer?.cancel();
    _controller.dispose();
    _scroll.dispose();
    super.dispose();
  }

  Future<void> _loadAssistantAvatar() async {
    try {
      // AssetManifest에서 chatbot_profile.* 검색 (png/webp/jpg/jpeg/gif)
      final manifestJson = await rootBundle.loadString('AssetManifest.json');
      final Map<String, dynamic> manifest = jsonDecode(manifestJson);

      final exts = ['png', 'webp', 'jpg', 'jpeg', 'gif'];
      String? found;

      // 우선순위: assets/images/chatbot_profile.<ext>
      for (final ext in exts) {
        final cand = 'assets/images/chatbot_profile.$ext';
        if (manifest.keys.contains(cand)) {
          found = cand;
          break;
        }
      }
      // 혹시 다른 하위 폴더/이름 변형이 있을 경우 여유 검색
      found ??= manifest.keys.firstWhere(
            (k) => k.toLowerCase().contains('assets/images/') &&
            k.toLowerCase().contains('chatbot_profile'),
        orElse: () => '',
      );
      if (found != null && found.isNotEmpty) {
        setState(() => _assistantAvatarImg = AssetImage(found!));
      } else {
        // 없으면 임시 플레이스홀더(파란 원+S)
        setState(() => _assistantAvatarImg = null);
      }
    } catch (_) {
      setState(() => _assistantAvatarImg = null);
    }
  }

  Future<void> _loadUserAvatar() async {
    try {
      final prof = await ProfileCacheService.loadProfile();
      final imageUrl = prof?['imageUrl'];
      ImageProvider img;
      if (imageUrl != null && imageUrl.toString().startsWith('http')) {
        img = NetworkImage(imageUrl);
      } else if (imageUrl != null && imageUrl.toString().startsWith('assets/')) {
        img = AssetImage(imageUrl);
      } else {
        img = const AssetImage('assets/images/profile_cat.png');
      }
      if (!mounted) return;
      setState(() => _userAvatar = img);
    } catch (_) {}
  }

  Future<void> _loadHistory() async {
    try {
      final hist = await _api.fetchHistory(profileId: widget.profileId);
      setState(() {
        _messages..clear()..addAll(hist);
        _loading = false;
      });

      // ✅ 진행중(서버에 아직 반영 전)인 로컬 보낸메시지 복원
      await _restorePendingIfAny();

      _jumpToNewest();
    } catch (e) {
      setState(() => _loading = false);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('이전 대화 불러오기 실패: $e')),
        );
      }
      // 실패해도 진행중 복원은 시도
      await _restorePendingIfAny();
    }
  }

  // ── 진행중 메시지 로컬복원 ──────────────────────────────────────────────
  Future<void> _restorePendingIfAny() async {
    final (pendingText, pendingAtIso, wasSending) =
    await _LocalChatState.load(widget.profileId);

    if (pendingText != null && pendingAtIso != null) {
      final pendingAt = DateTime.tryParse(pendingAtIso);
      // 중복 방지: 같은 내용+시간(±2초) 있으면 추가 안 함
      final already = _messages.any((m) =>
      m.role == 'user' &&
          m.content == pendingText &&
          pendingAt != null &&
          (m.createdAt.difference(pendingAt).inSeconds).abs() <= 2);

      if (!already) {
        setState(() {
          _messages.add(ChatMessage(
            role: 'user',
            content: pendingText,
            createdAt: pendingAt ?? nowKstLocal(),
          ));
        });
      }
    }

    if (wasSending == true) {
      // 여전히 답변 대기 중 → 타이핑 인디케이터 재개 + 전송 막기 + ✅ 폴링 시작
      if (!_assistantTyping) _startTyping();
      if (!_sending) setState(() => _sending = true);

      _pendingAt = DateTime.tryParse(pendingAtIso!); // 기준 시각 저장
      _pendingText = pendingText;                    // 기준 텍스트 저장
      _startPendingWatcher();                        // ✅ 자동 새로고침 시작
    }
  }

  // ── 자동 새로고침(2초 간격 폴링) — 진행중이면 서버 이력 재조회 ───────────
  void _startPendingWatcher() {
    _pendingPoll?.cancel();
    if (_pendingAt == null) return;

    _pendingPoll = Timer.periodic(const Duration(seconds: 2), (_) async {
      if (!mounted || !_sending) { _pendingPoll?.cancel(); return; }
      try {
        final hist = await _api.fetchHistory(profileId: widget.profileId);

        // 보낸 시각(±2초 버퍼) 이후에 assistant 메시지가 생겼는지 확인
        if (_hasAssistantAfterPending(hist)) {
          setState(() {
            _messages
              ..clear()
              ..addAll(hist);      // 최신 히스토리로 교체(답변 포함)
            _sending = false;      // 입력 잠금 해제
          });
          _stopTyping();           // 인디케이터 종료
          await _LocalChatState.clear(widget.profileId);
          _pendingPoll?.cancel();
          _pendingAt = null;
          _pendingText = null;
          _jumpToNewest();
        }
      } catch (_) {
        // 네트워크 일시 오류면 다음 틱에서 재시도
      }
    });
  }

  bool _hasAssistantAfterPending(List<ChatMessage> hist) {
    if (_pendingAt == null) return false;
    final fromT = _pendingAt!.subtract(const Duration(seconds: 2)); // 버퍼
    return hist.any((m) =>
    m.role == 'assistant' &&
        m.content.trim().isNotEmpty &&
        !m.createdAt.isBefore(fromT)
    );
  }

  // ── 타이핑 인디케이터 제어 ─────────────────────────────────────────────
  void _startTyping() {
    _typingTimer?.cancel();
    setState(() {
      _assistantTyping = true;
      _typingDots = 1;
      _typingDir = 1;
    });
    _typingTimer = Timer.periodic(const Duration(milliseconds: 500), (_) {
      if (!mounted) return;
      setState(() {
        // 1→2→3→2→1 왕복
        _typingDots += _typingDir;
        if (_typingDots >= 3) {
          _typingDots = 3;
          _typingDir = -1;
        } else if (_typingDots <= 1) {
          _typingDots = 1;
          _typingDir = 1;
        }
      });
    });
  }

  void _stopTyping() {
    _typingTimer?.cancel();
    _typingTimer = null;
    if (mounted) {
      setState(() => _assistantTyping = false);
    }
  }

  // SEND(일반 질문)
  Future<void> _onSend() async {
    final text = _controller.text.trim();
    if (text.isEmpty || _sending) return; // ✅ 중복 전송 방지

    final now = nowKstLocal();
    setState(() {
      _messages.add(ChatMessage(role: 'user', content: text, createdAt: now));
      _sending = true; // ✅ 잠금
    });
    _controller.clear();
    _jumpToNewest();

    _startTyping(); // ✅ 애니메이션 시작

    // ✅ 진행중 기준 저장 + 로컬 저장 + 폴링 시작
    _pendingAt = now;
    _pendingText = text;
    await _LocalChatState.save(widget.profileId, text, now);
    _startPendingWatcher();

    try {
      final res = await _api.sendQuestion(profileId: widget.profileId, question: text);
      final botText = (res.summary.isEmpty) ? '(빈 응답)' : res.summary;
      _addBotBubble(botText, createdAt: res.createdAt);

      // ✅ 응답 도착 → 진행중 해제
      await _LocalChatState.clear(widget.profileId);
      _pendingPoll?.cancel();
      _pendingAt = null;
      _pendingText = null;
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('전송 실패: $e')),
        );
      }
      // 실패 시 진행중 상태 클리어(중복 폭주 방지)
      await _LocalChatState.clear(widget.profileId);
      _pendingPoll?.cancel();
      _pendingAt = null;
      _pendingText = null;
    } finally {
      _stopTyping(); // ✅ 애니메이션 종료
      if (mounted) setState(() => _sending = false); // ✅ 잠금 해제
    }
  }

  void _addBotBubble(String text, {DateTime? createdAt}) {
    setState(() {
      _messages.add(ChatMessage(
        role: 'assistant', content: text, createdAt: createdAt ?? nowKstLocal(),
      ));
    });
    _jumpToNewest();
  }

  void _addUserBubble(String text, {DateTime? createdAt}) {
    setState(() {
      _messages.add(ChatMessage(
        role: 'user', content: text, createdAt: createdAt ?? nowKstLocal(),
      ));
    });
    _jumpToNewest();
  }

  void _jumpToNewest() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_scroll.hasClients) {
        _scroll.animateTo(0, duration: const Duration(milliseconds: 200), curve: Curves.easeOut);
      }
    });
  }

  // FAQ 바텀시트 열기 → 질문/답변 둘 다 바로 추가 (진행중 상태 아님)
  Future<void> _openFaqSheet() async {
    final qa = await showModalBottomSheet<({String q, String a})?>(
      context: context,
      isScrollControlled: true,
      backgroundColor: Colors.transparent,
      builder: (_) => _FaqSheet(profileId: widget.profileId),
    );

    if (qa != null) {
      _addUserBubble('[FAQ] ${qa.q}');
      _addBotBubble(qa.a);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: rootBackground,
      appBar: AppBar(
        backgroundColor: rootBackground,
        elevation: 0,
        title: const Text('SEONY', style: TextStyle(color: Colors.white)),
        actions: [
          IconButton(
            icon: const Icon(Icons.help_outline, color: Colors.white),
            tooltip: 'FAQ 바로답변',
            onPressed: _openFaqSheet,
          ),
        ],
      ),
      body: Column(
        children: [
          _header(),
          if (_loading) const LinearProgressIndicator(minHeight: 2),

          Expanded(
            child: ListView.builder(
              controller: _scroll,
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              itemCount: _messages.length,
              reverse: true,
              itemBuilder: (context, index) {
                final realIdx = _messages.length - 1 - index;
                final m = _messages[realIdx];
                final isUser = m.role == 'user';
                final timeStr = _formatBubbleTime(m.createdAt);

                final bool showDateChip = realIdx == 0
                    ? true
                    : !_sameDay(m.createdAt, _messages[realIdx - 1].createdAt);

                final bubble = _buildBubble(content: m.content, timeStr: timeStr, isUser: isUser);

                return Column(
                  crossAxisAlignment: isUser ? CrossAxisAlignment.end : CrossAxisAlignment.start,
                  children: [
                    if (showDateChip) _dateChip(_dateChipText(m.createdAt)),
                    Padding(
                      padding: const EdgeInsets.symmetric(vertical: 4),
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.start, // 아바타 위정렬
                        mainAxisAlignment: isUser ? MainAxisAlignment.end : MainAxisAlignment.start,
                        children: isUser
                            ? [
                          Flexible(child: bubble),
                          const SizedBox(width: 6),
                          Padding(padding: const EdgeInsets.only(top: 2), child: _userAvatarWidget()),
                        ]
                            : [
                          // ✅ 챗봇 아바타만 살짝 위로 올림 (-6px)
                          Transform.translate(
                            offset: const Offset(0, -6),
                            child: _assistantAvatar(),
                          ),
                          const SizedBox(width: 6),
                          Flexible(child: bubble),
                        ],
                      ),
                    ),
                  ],
                );
              },
            ),
          ),

          if (_assistantTyping) _typingIndicator(), // ✅ 애니메이션 텍스트

          SafeArea(
            top: false,
            child: Container(
              color: const Color(0xFF0B0F14),
              padding: const EdgeInsets.fromLTRB(12, 10, 12, 12),
              child: Row(
                children: [
                  Expanded(
                    child: TextField(
                      controller: _controller,
                      enabled: !_sending, // ✅ 답변대기 중 입력 금지
                      style: const TextStyle(color: Colors.white),
                      decoration: InputDecoration(
                        hintText: _sending ? '답변을 기다리는 중…' : 'SEONY에게 물어보기…',
                        hintStyle: TextStyle(color: Colors.white.withOpacity(0.5)),
                        fillColor: inputBg,
                        filled: true,
                        contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
                        border: OutlineInputBorder(borderSide: BorderSide.none, borderRadius: BorderRadius.circular(14)),
                      ),
                      onSubmitted: (_) => _onSend(),
                    ),
                  ),
                  const SizedBox(width: 8),
                  IconButton(
                    onPressed: _sending ? null : _onSend, // ✅ 대기중이면 버튼 비활성
                    icon: const Icon(Icons.send_rounded, color: Colors.white),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  // ── Sub-widgets
  Widget _header() {
    return Container(
      padding: const EdgeInsets.fromLTRB(16, 10, 16, 6),
      decoration: const BoxDecoration(color: rootBackground),
      child: Row(
        children: [
          _assistantAvatar(),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              '자세·프리셋·앱 사용법, SEONY가 도와드려요.',
              style: TextStyle(color: Colors.white.withOpacity(0.45), fontSize: 12),
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ],
      ),
    );
  }

  // ✅ SEONY 아바타: chatbot_profile 이미지 사용
  Widget _assistantAvatar() {
    if (_assistantAvatarImg == null) {
      return Container(
        width: assistantAvatarSize,
        height: assistantAvatarSize,
        decoration: const BoxDecoration(color: primaryBlue, shape: BoxShape.circle),
        alignment: Alignment.center,
        child: const Text(
          'S',
          style: TextStyle(color: Colors.white, fontWeight: FontWeight.w800, fontSize: 20),
        ),
      );
    }
    return CircleAvatar(
      radius: assistantAvatarSize / 2,
      backgroundImage: _assistantAvatarImg,
      backgroundColor: Colors.transparent,
    );
  }

  Widget _userAvatarWidget() {
    final img = _userAvatar;
    if (img == null) {
      return Container(
        width: avatarSize, height: avatarSize,
        decoration: BoxDecoration(color: Colors.white.withOpacity(0.1), shape: BoxShape.circle),
        alignment: Alignment.center,
        child: const Text('U', style: TextStyle(color: Colors.white70, fontWeight: FontWeight.w700)),
      );
    }
    return CircleAvatar(radius: avatarSize / 2, backgroundImage: img);
  }

  Widget _buildBubble({required String content, required String timeStr, required bool isUser}) {
    return ConstrainedBox(
      constraints: BoxConstraints(maxWidth: MediaQuery.of(context).size.width * 0.75),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
        decoration: BoxDecoration(color: isUser ? primaryBlue : bubbleBg, borderRadius: BorderRadius.circular(14)),
        child: Column(
          crossAxisAlignment: isUser ? CrossAxisAlignment.end : CrossAxisAlignment.start,
          children: [
            if (!isUser) ...[
              const Text(assistantDisplayName, style: TextStyle(color: Colors.white70, fontSize: 11, fontWeight: FontWeight.w600)),
              const SizedBox(height: 2),
            ],
            GestureDetector(
              onLongPress: () {
                Clipboard.setData(ClipboardData(text: content));
                ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('메시지 복사됨')));
              },
              child: Text(content, style: const TextStyle(color: Colors.white, fontSize: 14)),
            ),
            const SizedBox(height: 4),
            Text(timeStr, style: TextStyle(color: Colors.white.withOpacity(0.8), fontSize: 11)),
          ],
        ),
      ),
    );
  }

  Widget _dateChip(String text) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Center(
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
          decoration: BoxDecoration(color: Colors.white12, borderRadius: BorderRadius.circular(12)),
          child: Text(text, style: const TextStyle(color: Colors.white70, fontSize: 11)),
        ),
      ),
    );
  }

  Widget _typingIndicator() {
    final dots = '.' * _typingDots; // 1~3개 점
    return Padding(
      padding: const EdgeInsets.only(left: 12, right: 12, bottom: 8),
      child: Align(
        alignment: Alignment.centerLeft,
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          decoration: BoxDecoration(color: bubbleBg, borderRadius: BorderRadius.circular(14)),
          child: Text(
            'SEONY가 답변을 만드는중 $dots',
            style: const TextStyle(color: Colors.white70, fontSize: 13, fontWeight: FontWeight.w600),
          ),
        ),
      ),
    );
  }
}

// ───────────────────────────────────────────────────────────────────────────────
// FAQ 바텀시트 — 탭 시 서버에 기록하고, 질문/답변을 함께 반환
// ───────────────────────────────────────────────────────────────────────────────
class _FaqSheet extends StatefulWidget {
  final int profileId;
  const _FaqSheet({required this.profileId});

  @override
  State<_FaqSheet> createState() => _FaqSheetState();
}

class _FaqSheetState extends State<_FaqSheet> {
  String _q = '';
  bool _loading = true;
  List<Faq> _list = [];

  final TextEditingController _searchCtrl = TextEditingController();

  @override
  void initState() {
    super.initState();
    _load();
  }

  @override
  void dispose() {
    _searchCtrl.dispose();
    super.dispose();
  }

  Future<void> _load() async {
    setState(() => _loading = true);
    try {
      final (list, _etag, _fromCache) = await FaqService.getFaqs(q: _q);
      setState(() {
        _list = list;
        _loading = false;
      });
    } catch (e) {
      setState(() => _loading = false);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('FAQ 불러오기 실패: $e')));
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return DraggableScrollableSheet(
      initialChildSize: 0.85,
      minChildSize: 0.5,
      maxChildSize: 0.95,
      builder: (_, controller) => Container(
        padding: const EdgeInsets.fromLTRB(16, 12, 16, 16),
        decoration: const BoxDecoration(color: Color(0xFF161B22), borderRadius: BorderRadius.vertical(top: Radius.circular(24))),
        child: Column(
          children: [
            Container(width: 40, height: 4, margin: const EdgeInsets.only(bottom: 12), decoration: BoxDecoration(color: Colors.white24, borderRadius: BorderRadius.circular(2))),
            Row(children: const [Icon(Icons.help_outline, color: Colors.white70), SizedBox(width: 8), Text('FAQ 빠른 답변', style: TextStyle(color: Colors.white, fontSize: 16))]),
            const SizedBox(height: 12),
            TextField(
              controller: _searchCtrl,
              style: const TextStyle(color: Colors.white),
              decoration: InputDecoration(
                hintText: '검색(질문/답변)',
                hintStyle: const TextStyle(color: Colors.white54),
                filled: true,
                fillColor: const Color(0xFF0D1117),
                contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 12),
                border: OutlineInputBorder(borderSide: BorderSide.none, borderRadius: BorderRadius.circular(12)),
                prefixIcon: const Icon(Icons.search, color: Colors.white70),
                suffixIcon: (_q.isNotEmpty || _searchCtrl.text.isNotEmpty)
                    ? IconButton(
                  icon: const Icon(Icons.clear, color: Colors.white70),
                  onPressed: () { _q = ''; _searchCtrl.clear(); _load(); },
                )
                    : null,
              ),
              onSubmitted: (v) { _q = v.trim(); _load(); },
            ),
            const SizedBox(height: 12),
            Expanded(
              child: _loading
                  ? const Center(child: CircularProgressIndicator())
                  : _list.isEmpty
                  ? const Center(child: Text('검색 결과 없음', style: TextStyle(color: Colors.white60)))
                  : ListView.separated(
                controller: controller,
                itemCount: _list.length,
                separatorBuilder: (_, __) => const Divider(height: 1, color: Colors.white10),
                itemBuilder: (_, i) {
                  final f = _list[i];
                  return ListTile(
                    contentPadding: const EdgeInsets.symmetric(horizontal: 4, vertical: 6),
                    title: Text(f.question, style: const TextStyle(color: Colors.white, fontWeight: FontWeight.w600)),
                    subtitle: Padding(
                      padding: const EdgeInsets.only(top: 6.0),
                      child: Text(f.answer, maxLines: 2, overflow: TextOverflow.ellipsis, style: const TextStyle(color: Colors.white70, fontSize: 13)),
                    ),
                    trailing: const Icon(Icons.chevron_right, color: Colors.white70),
                    onTap: () async {
                      try {
                        final qa = await FaqService.answerFromFaq(faqId: f.id, profileId: widget.profileId);
                        if (!mounted) return;
                        Navigator.pop(context, (q: f.question, a: qa.answer));
                      } catch (e) {
                        if (!mounted) return;
                        ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('요청 실패: $e')));
                      }
                    },
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

/// ─────────────────────────────────────────────────────────────────────────────
/// 진행중(응답 대기) 메시지 로컬 저장 유틸
///   - 키: 프로필별로 분리
///   - 저장: text + createdAt + sending=true
///   - 복원: text/createdAt/sending
///   - 클리어: 모두 삭제
/// ─────────────────────────────────────────────────────────────────────────────
class _LocalChatState {
  static String _kText(int pid) => 'chat_pending_text_$pid';
  static String _kAt(int pid) => 'chat_pending_at_$pid';
  static String _kSending(int pid) => 'chat_sending_$pid';

  static Future<void> save(int profileId, String text, DateTime createdAt) async {
    final p = await SharedPreferences.getInstance();
    await p.setString(_kText(profileId), text);
    await p.setString(_kAt(profileId), createdAt.toIso8601String());
    await p.setBool(_kSending(profileId), true);
  }

  static Future<(String?, String?, bool?)> load(int profileId) async {
    final p = await SharedPreferences.getInstance();
    final text = p.getString(_kText(profileId));
    final at = p.getString(_kAt(profileId));
    final sending = p.getBool(_kSending(profileId));
    return (text, at, sending);
  }

  static Future<void> clear(int profileId) async {
    final p = await SharedPreferences.getInstance();
    await p.remove(_kText(profileId));
    await p.remove(_kAt(profileId));
    await p.remove(_kSending(profileId));
  }
}
