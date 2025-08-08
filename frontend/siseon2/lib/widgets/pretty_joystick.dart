import 'package:flutter/material.dart';
import 'dart:math';

class PrettyJoystick extends StatefulWidget {
  final Function(Offset offset)? onMove;

  const PrettyJoystick({super.key, this.onMove});

  @override
  State<PrettyJoystick> createState() => _PrettyJoystickState();
}

class _PrettyJoystickState extends State<PrettyJoystick> with SingleTickerProviderStateMixin {
  Offset _stickOffset = Offset.zero;
  late AnimationController _controller;
  late Animation<Offset> _resetAnimation;

  final double baseSize = 140;
  final double stickSize = 60;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this, duration: const Duration(milliseconds: 100));
  }

  void _startResetAnimation() {
    _resetAnimation = Tween(begin: _stickOffset, end: Offset.zero).animate(_controller)
      ..addListener(() {
        setState(() => _stickOffset = _resetAnimation.value);
        widget.onMove?.call(_stickOffset);
      });
    _controller.forward(from: 0);
  }

  void _updateStick(Offset localPosition) {
    final center = baseSize / 2;
    final dx = localPosition.dx - center;
    final dy = localPosition.dy - center;
    final distance = sqrt(dx * dx + dy * dy);
    final maxRadius = center - stickSize / 2;

    final angle = atan2(dy, dx);
    final limitedDistance = min(distance, maxRadius);
    final offset = Offset(
      cos(angle) * limitedDistance,
      sin(angle) * limitedDistance,
    );

    setState(() => _stickOffset = offset);
    widget.onMove?.call(offset / maxRadius);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanUpdate: (details) => _updateStick(details.localPosition),
      onPanEnd: (_) => _startResetAnimation(),
      child: Container(
        width: baseSize,
        height: baseSize,
        decoration: BoxDecoration(
          color: Colors.white.withOpacity(0.1),
          shape: BoxShape.circle,
          border: Border.all(color: Colors.white30),
          boxShadow: const [
            BoxShadow(color: Colors.black26, blurRadius: 10, spreadRadius: 2),
          ],
        ),
        child: Stack(
          children: [
            // 스틱
            Positioned(
              left: (baseSize - stickSize) / 2 + _stickOffset.dx,
              top: (baseSize - stickSize) / 2 + _stickOffset.dy,
              child: Container(
                width: stickSize,
                height: stickSize,
                decoration: BoxDecoration(
                  gradient: const LinearGradient(
                    colors: [Colors.white, Colors.blueAccent],
                    begin: Alignment.topLeft,
                    end: Alignment.bottomRight,
                  ),
                  shape: BoxShape.circle,
                  boxShadow: const [
                    BoxShadow(color: Colors.black26, blurRadius: 6),
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
