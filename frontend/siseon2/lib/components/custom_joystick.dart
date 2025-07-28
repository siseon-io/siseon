import 'package:flutter/material.dart';
import 'dart:math';

class CustomJoystick extends StatefulWidget {
  final void Function(Offset offset) onChanged;

  const CustomJoystick({super.key, required this.onChanged});

  @override
  State<CustomJoystick> createState() => _CustomJoystickState();
}

class _CustomJoystickState extends State<CustomJoystick> {
  Offset stickOffset = Offset.zero;
  final double baseRadius = 60;
  final double stickRadius = 25;

  void _updateStick(Offset offset) {
    final dx = offset.dx - baseRadius;
    final dy = offset.dy - baseRadius;
    final distance = sqrt(dx * dx + dy * dy);

    // 바깥으로 튀어나오게 만들기 위해 clamp 안 함
    final result = Offset(dx, dy);
    setState(() {
      stickOffset = result;
    });
    widget.onChanged(result);
  }

  void _resetStick() {
    setState(() {
      stickOffset = Offset.zero;
    });
    widget.onChanged(Offset.zero);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanStart: (details) => _updateStick(details.localPosition),
      onPanUpdate: (details) => _updateStick(details.localPosition),
      onPanEnd: (_) => _resetStick(),
      child: Container(
        width: baseRadius * 2,
        height: baseRadius * 2,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: Colors.grey.withOpacity(0.2),
        ),
        child: Stack(
          children: [
            Positioned(
              left: baseRadius - stickRadius + stickOffset.dx,
              top: baseRadius - stickRadius + stickOffset.dy,
              child: Container(
                width: stickRadius * 2,
                height: stickRadius * 2,
                decoration: const BoxDecoration(
                  color: Colors.blueAccent,
                  shape: BoxShape.circle,
                  boxShadow: [
                    BoxShadow(color: Colors.black26, blurRadius: 4),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
