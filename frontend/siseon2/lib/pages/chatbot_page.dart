// lib/pages/chat/chatbot_page.dart
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

import 'package:siseon2/services/chat_api.dart';
import 'package:siseon2/models/chat_models.dart';
import 'package:flutter/services.dart';
import 'package:siseon2/services/auth_service.dart';
import 'package:siseon2/services/profile_cache_service.dart';

class ChatbotPage extends StatefulWidget {
  final int profileId; // ✅ 프로필 ID 필요

  const ChatbotPage({super.key, required this.profileId});

  @override
  State<ChatbotPage> createState() => _ChatbotPageState();
}

class _ChatbotPageState extends State<ChatbotPage> {
  // ── THEME ────────────────────────────────────────────────────────────────────
  static const Color primaryBlue = Color(0xFF3B82F6);
  static const Color rootBackground = Color(0xFF161B22);
  static const Color bubbleBg = Color(0xFF1F2937);
  static const Color inputBg = Color(0xFF111827);

  // ── STATE ────────────────────────────────────────────────────────────────────
  final List<ChatMessage> _messages = [];
  final TextEditingController _controller = TextEditingController();
  final ScrollController _scroll = ScrollController();
  final _timeFmt = DateFormat('HH:mm');

  final ChatApi _api = ChatApi();
  bool _loading = true;
  bool _sending = false;

  // ── LIFECYCLE ────────────────────────────────────────────────────────────────
  @override
  void initState() {
    super.initState();
    _loadHistory();
  }

  Future<void> _loadHistory() async {
    try {
      final hist = await _api.fetchHistory(profileId: widget.profileId);
      setState(() {
        _messages
          ..clear()
          ..addAll(hist);
        _loading = false;
      });
      _jumpToNewest(); // reverse=true에서 0으로 스크롤이 최신
    } catch (e) {
      setState(() => _loading = false);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('이전 대화 불러오기 실패: $e')),
        );
      }
    }
  }

  // ── SEND ─────────────────────────────────────────────────────────────────────
  Future<void> _onSend() async {
    final text = _controller.text.trim();
    if (text.isEmpty || _sending) return;

    final now = DateTime.now();
    setState(() {
      _messages.add(ChatMessage(role: 'user', content: text, createdAt: now));
      _sending = true;
    });
    _controller.clear();
    _jumpToNewest();

    try {
      final res = await _api.sendQuestion(
        profileId: widget.profileId,
        question: text,
      );
      final assistantText = _composeAssistantText(res.summary, res.details);

      setState(() {
        _messages.add(ChatMessage(
          role: 'assistant',
          content: assistantText,
          createdAt: res.createdAt,
        ));
      });
      _jumpToNewest();
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('전송 실패: $e')),
        );
      }
    } finally {
      if (mounted) setState(() => _sending = false);
    }
  }

  String _composeAssistantText(String summary, Map<String, dynamic> details) {
    if (details.isEmpty) return summary;
    final tail = details.toString(); // 필요하면 예쁘게 렌더링하도록 후속 개선
    return summary.isEmpty ? tail : '$summary\n\n$tail';
  }

  void _jumpToNewest() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_scroll.hasClients) {
        // reverse: true 이므로 0이 최신쪽
        _scroll.animateTo(
          0,
          duration: const Duration(milliseconds: 200),
          curve: Curves.easeOut,
        );
      }
    });
  }

  // ── UI ───────────────────────────────────────────────────────────────────────
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: rootBackground,
      appBar: AppBar(
        backgroundColor: rootBackground,
        elevation: 0,
        title: const Text('챗봇', style: TextStyle(color: Colors.white)),
        actions: [
          IconButton(
            icon: const Icon(Icons.bug_report, color: Colors.white),
            onPressed: () async {
              final token = await AuthService.getValidAccessToken();
              final prof = await ProfileCacheService.loadProfile();
              final pid = prof?['profileId'] ?? prof?['id'];
              await Clipboard.setData(ClipboardData(text: token ?? ''));
              if (!mounted) return;
              ScaffoldMessenger.of(context).showSnackBar(
                SnackBar(content: Text('토큰 복사됨, profileId=$pid')),
              );
              debugPrint('[DEBUG] token=${token?.substring(0,20)}...  pid=$pid');
            },
            tooltip: '토큰 복사',
          ),
        ],
      ),
      body: Column(
        children: [
          if (_loading)
            const LinearProgressIndicator(minHeight: 2),
          Expanded(
            child: ListView.builder(
              controller: _scroll,
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              itemCount: _messages.length,
              reverse: true, // ✅ 최신이 아래쪽처럼 보이게
              itemBuilder: (context, index) {
                final m = _messages[_messages.length - 1 - index];
                final isUser = m.role == 'user';
                final timeStr = _timeFmt.format(m.createdAt);

                return Align(
                  alignment: isUser ? Alignment.centerRight : Alignment.centerLeft,
                  child: Container(
                    margin: const EdgeInsets.symmetric(vertical: 6),
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                    constraints: BoxConstraints(
                      maxWidth: MediaQuery.of(context).size.width * 0.75,
                    ),
                    decoration: BoxDecoration(
                      color: isUser ? primaryBlue : bubbleBg,
                      borderRadius: BorderRadius.circular(14),
                    ),
                    child: Column(
                      crossAxisAlignment:
                      isUser ? CrossAxisAlignment.end : CrossAxisAlignment.start,
                      children: [
                        Text(
                          m.content,
                          style: const TextStyle(color: Colors.white, fontSize: 14),
                        ),
                        const SizedBox(height: 4),
                        Text(
                          timeStr,
                          style: TextStyle(
                            color: Colors.white.withOpacity(0.8),
                            fontSize: 11,
                          ),
                        ),
                      ],
                    ),
                  ),
                );
              },
            ),
          ),
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
                      style: const TextStyle(color: Colors.white),
                      decoration: InputDecoration(
                        hintText: '메시지를 입력하세요…',
                        hintStyle: TextStyle(color: Colors.white.withOpacity(0.5)),
                        fillColor: inputBg,
                        filled: true,
                        contentPadding: const EdgeInsets.symmetric(
                            horizontal: 14, vertical: 12),
                        border: OutlineInputBorder(
                          borderSide: BorderSide.none,
                          borderRadius: BorderRadius.circular(14),
                        ),
                      ),
                      onSubmitted: (_) => _onSend(),
                    ),
                  ),
                  const SizedBox(width: 8),
                  IconButton(
                    onPressed: _sending ? null : _onSend,
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
}
