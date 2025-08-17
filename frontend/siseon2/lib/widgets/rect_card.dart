import 'package:flutter/material.dart';

class RectCard extends StatelessWidget {
  final Widget child;
  final VoidCallback? onTap;
  final double radius;
  final EdgeInsetsGeometry padding;
  final Color bgColor;
  final Color? outlineColor;   // 없으면 기본 얇은 보더
  final double? height;
  final bool elevated;         // 그림자 on/off

  const RectCard({
    super.key,
    required this.child,
    this.onTap,
    this.radius = 12, // 각 살짝 또렷
    this.padding = const EdgeInsets.all(12),
    this.bgColor = const Color(0xFF161B22),
    this.outlineColor,
    this.height,
    this.elevated = false,
  });

  @override
  Widget build(BuildContext context) {
    final border = outlineColor ?? Colors.white.withOpacity(0.24); // ★ 0.24
    final shadows = elevated
        ? [
      BoxShadow(
        color: Colors.black.withOpacity(0.35),
        blurRadius: 14,
        offset: const Offset(0, 10),
      ),
    ]
        : [
      BoxShadow(
        color: Colors.black.withOpacity(0.16),
        blurRadius: 8,
        offset: const Offset(0, 6),
      ),
    ];

    return Material(
      color: Colors.transparent,
      child: InkWell(
        borderRadius: BorderRadius.circular(radius),
        onTap: onTap,
        child: Container(
          height: height,
          padding: padding,
          decoration: BoxDecoration(
            color: bgColor,
            borderRadius: BorderRadius.circular(radius),
            border: Border.all(color: border, width: 1.4), // ★ 1.4px
            boxShadow: shadows,
          ),
          child: child,
        ),
      ),
    );
  }
}