import 'package:flutter/material.dart';
import 'root_screen.dart';

class ProfileSelectScreen extends StatelessWidget {
  static const List<Map<String, String>> dummyProfiles = [
    {'name': '초록이', 'image': 'assets/images/profile_frog.png'},
    {'name': '야옹이', 'image': 'assets/images/profile_cat.png'},
    {'name': '멍멍이', 'image': 'assets/images/profile_dog.png'},
  ];

  const ProfileSelectScreen({super.key});

  void onAddPressed(BuildContext context) {
    // 여기에 프로필 추가 화면으로 이동하는 로직 구현
    print("➕ 프로필 추가 버튼 눌림");
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('프로필 추가 화면으로 이동!')),
    );
  }

  @override
  Widget build(BuildContext context) {
    final profiles = List<Map<String, String>>.from(dummyProfiles);
    final hasAddButton = profiles.length < 4;

    if (hasAddButton) {
      profiles.add({'name': '', 'image': ''}); // "+" 버튼용 빈 항목 추가
    }

    return Scaffold(
      backgroundColor: const Color(0xFF1C1C1E),
      appBar: AppBar(
        title: const Text('프로필 선택'),
        backgroundColor: const Color(0xFF1C1C1E),
        elevation: 0,
        centerTitle: true,
      ),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: GridView.builder(
          itemCount: profiles.length,
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: 2,
            crossAxisSpacing: 24,
            mainAxisSpacing: 24,
            childAspectRatio: 0.85,
          ),
          itemBuilder: (context, index) {
            final profile = profiles[index];

            final isAddButton = profile['name'] == '';

            return GestureDetector(
              onTap: () {
                if (isAddButton) {
                  onAddPressed(context);
                } else {
                  final name = profile['name']!;
                  print('👉 선택된 프로필: $name');
                  Navigator.pushReplacement(
                    context,
                    MaterialPageRoute(
                      builder: (_) => RootScreen(userName: name),
                    ),
                  );
                }
              },
              child: AnimatedContainer(
                duration: const Duration(milliseconds: 200),
                padding: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  color: Colors.white10,
                  borderRadius: BorderRadius.circular(24),
                  border: Border.all(color: Colors.white24, width: 1),
                ),
                child: isAddButton
                    ? Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: const [
                    Icon(Icons.add_circle_outline,
                        size: 48, color: Colors.white70),
                    SizedBox(height: 12),
                    Text(
                      '프로필 추가',
                      style: TextStyle(
                        color: Colors.white70,
                        fontSize: 16,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ],
                )
                    : Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    CircleAvatar(
                      backgroundImage: AssetImage(profile['image']!),
                      radius: 40,
                    ),
                    const SizedBox(height: 12),
                    Text(
                      profile['name']!,
                      style: const TextStyle(
                        fontSize: 18,
                        color: Colors.white,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );
  }
}
