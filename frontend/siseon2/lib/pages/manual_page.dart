import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../widgets/pretty_joystick.dart';

class ManualPage extends StatefulWidget {
  const ManualPage({super.key});

  @override
  State<ManualPage> createState() => _ManualPageState();
}

class _ManualPageState extends State<ManualPage> {
  @override
  void initState() {
    super.initState();

    // 3초 뒤 가로모드 전환
    Future.delayed(const Duration(seconds: 3), () {
      SystemChrome.setPreferredOrientations([
        DeviceOrientation.landscapeLeft,
        DeviceOrientation.landscapeRight,
      ]);
    });
  }

  @override
  void dispose() {
    // 페이지 벗어날 때 세로모드 복구
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
    ]);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF10121A),
      appBar: AppBar(
        backgroundColor: Colors.black,
        title: const Text('BACK'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back),
          onPressed: () async {
            // ✅ 세로모드 먼저 복구하고 pop
            await SystemChrome.setPreferredOrientations([
              DeviceOrientation.portraitUp,
            ]);
            if (context.mounted) Navigator.pop(context);
          },
        ),
      ),
      body: Stack(
        children: [
          Positioned(
            bottom: 40,
            left: 30,
            child: Column(
              children: [
                const Text('XY Control', style: TextStyle(color: Colors.white70)),
                const SizedBox(height: 12),
                PrettyJoystick(
                  onMove: (offset) {
                    print('XY 이동: $offset');
                  },
                ),
              ],
            ),
          ),
          Positioned(
            bottom: 40,
            right: 30,
            child: Column(
              children: [
                const Text('Z Control', style: TextStyle(color: Colors.white70)),
                const SizedBox(height: 12),
                PrettyJoystick(
                  onMove: (offset) {
                    print('Z 이동: ${offset.dy}');
                  },
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
