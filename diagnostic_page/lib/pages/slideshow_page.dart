import 'dart:async';

import 'package:diagnostic_page/widgets/clock.dart';
import 'package:diagnostic_page/widgets/slideshow.dart';
import 'package:flutter/material.dart';

class SlideshowPage extends StatefulWidget {
  const SlideshowPage({super.key});

  @override
  State<SlideshowPage> createState() => _SlideshowPageState();
}

class _SlideshowPageState extends State<SlideshowPage> {
  static const double padding = 8.0;
  Slideshow slide = Slideshow.intakeTitle;
  late Timer _timer;

  @override
  void initState() {
    super.initState();
    _updateSlide();
  }

  @override
  void dispose() {
    _timer.cancel();
    super.dispose();
  }

  void _updateSlide() {
    _timer = Timer.periodic(const Duration(seconds: 1), (Timer t) {
      slide = Slideshow.values[
          slide.index == Slideshow.values.length - 1 ? 0 : slide.index + 1];
      setState(() {}); // Trigger rebuild with updated time and date
    });
  }

  @override
  Widget build(BuildContext context) {
    return Stack(children: [
      Positioned(
        top: 0,
        right: 0,
        child: SizedBox(
          width: 250,
          height: 200,
          child: FittedBox(
            fit: BoxFit.contain,
            child: Image.asset(
              "assets/images/logo.png",
              filterQuality: FilterQuality.medium,
            ),
          ),
        ),
      ),
      Positioned(
        bottom: 2 * padding,
        left: 2 * padding,
        child: clock(),
      ),
      Center(
        child: slideshow(slide),
      )
    ]);
  }
}
