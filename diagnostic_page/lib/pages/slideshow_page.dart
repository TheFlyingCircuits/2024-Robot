import 'dart:async';
import 'package:flutter/material.dart';
import 'package:diagnostic_page/widgets/clock.dart';
import 'package:diagnostic_page/widgets/slideshow.dart';

class SlideshowPage extends StatefulWidget {
  const SlideshowPage({super.key});

  @override
  State<SlideshowPage> createState() => _SlideshowPageState();
}

class _SlideshowPageState extends State<SlideshowPage> {
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
      setState(() {
        slide = Slideshow.values[
            slide.index == Slideshow.values.length - 1 ? 0 : slide.index + 1];
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          Center(child: slideshow(slide)),
          Positioned(
            top: 0,
            right: 0,
            child: SizedBox(
              width: 250,
              height: 200,
              child: FittedBox(
                fit: BoxFit.contain,
                child: Image.asset(
                  'assets/images/logo.png',
                  filterQuality: FilterQuality.medium,
                ),
              ),
            ),
          ),
          clock(),
        ],
      ),
    );
  }
}
