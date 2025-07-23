import 'package:flutter/material.dart';
import 'home_screen.dart'; // ✅ import 추가

class ProfileSelectScreen extends StatelessWidget {
  static const List<Map<String, String>> dummyProfiles = [
    {'name': '철수', 'image': 'assets/images/profile1.png'},
    {'name': '영희', 'image': 'assets/images/profile2.png'},
    {'name': '민수', 'image': 'assets/images/profile3.png'},
  ];

  const ProfileSelectScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        title: const Text('프로필 선택'),
        backgroundColor: Colors.black,
        centerTitle: true,
      ),
      body: GridView.builder(
        padding: const EdgeInsets.all(24),
        gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
          crossAxisCount: 2,
          crossAxisSpacing: 24,
          mainAxisSpacing: 24,
        ),
        itemCount: dummyProfiles.length,
        itemBuilder: (context, index) {
          final profile = dummyProfiles[index];
          return GestureDetector(
            onTap: () {
              final name = profile['name']!;
              print('👉 선택된 프로필: $name');
              Navigator.push(
                context,
                MaterialPageRoute(
                  builder: (_) => HomeScreen(userName: name),
                ),
              );
            },
            child: Column(
              children: [
                CircleAvatar(
                  backgroundImage: AssetImage(profile['image']!),
                  radius: 50,
                ),
                const SizedBox(height: 8),
                Text(
                  profile['name']!,
                  style: const TextStyle(color: Colors.white, fontSize: 16),
                )
              ],
            ),
          );
        },
      ),
    );
  }
}
