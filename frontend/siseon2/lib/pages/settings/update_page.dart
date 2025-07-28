// 📁 lib/pages/settings/update_page.dart

import 'package:flutter/material.dart';

class UpdatePage extends StatelessWidget {
  const UpdatePage({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    const currentVersion = '1.2.0';
    const newVersion = '1.2.1';

    return Scaffold(
      appBar: AppBar(
        title: const Text('펌웨어 업데이트'),
        backgroundColor: Colors.white,
        foregroundColor: const Color(0xFF2563FF),
        elevation: 0,
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: Color(0xFF2563FF)),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 32),
          child: Column(
            children: [
              // 현재 버전
              _buildVersionCard('Current Version', currentVersion),
              const SizedBox(height: 16),
              // 화살표 아이콘
              const Icon(Icons.arrow_downward, size: 40, color: Colors.grey),
              const SizedBox(height: 16),
              // 새 버전
              _buildVersionCard('New Version', newVersion),
              const Spacer(),
              // 업데이트 버튼
              SizedBox(
                width: double.infinity,
                height: 50,
                child: ElevatedButton(
                  onPressed: () {
                    // TODO: 업데이트 로직
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('업데이트를 시작합니다.')),
                    );
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: const Color(0xFF2563FF),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(8),
                    ),
                  ),
                  child: const Text(
                    '업데이트',
                    style: TextStyle(fontSize: 16),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildVersionCard(String title, String version) {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      child: Container(
        width: double.infinity,
        padding: const EdgeInsets.symmetric(vertical: 20, horizontal: 16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(title,
                style: const TextStyle(
                  fontSize: 12,
                  color: Colors.grey,
                )),
            const SizedBox(height: 4),
            Text(
              version,
              style: const TextStyle(
                fontSize: 20,
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
