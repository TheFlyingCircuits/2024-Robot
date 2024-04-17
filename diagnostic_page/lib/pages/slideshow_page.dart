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
  Timer? _timer;
  bool isHovering = false;

  @override
  void initState() {
    super.initState();
    _startAutoSlideShow();
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  void _startAutoSlideShow() {
    if (_timer == null || !_timer!.isActive) {
      _timer = Timer.periodic(const Duration(seconds: 5), (Timer t) {
        setState(() {
          slide = Slideshow.values[(slide.index + 1) % Slideshow.values.length];
        });
      });
    }
  }

  void _stopAutoSlideShow() {
    if (_timer != null) {
      _timer!.cancel();
      _timer = null;
    }
  }

  void _toggleSlideShow() {
    if (_timer != null && _timer!.isActive) {
      _stopAutoSlideShow();
    } else {
      _startAutoSlideShow();
    }
  }

  Widget _buildThumbnails() {
    return Visibility(
      visible: isHovering,
      child: Align(
        alignment: Alignment.bottomCenter,
        child: Container(
          height: 100,
          child: ListView.builder(
            scrollDirection: Axis.horizontal,
            itemCount: slideTitles.length,
            itemBuilder: (context, index) {
              return GestureDetector(
                onTap: () {
                  setState(() {
                    slide = Slideshow.values[index];
                    _stopAutoSlideShow();
                  });
                },
                child: Container(
                  width: 120,
                  alignment: Alignment.center,
                  margin: const EdgeInsets.symmetric(horizontal: 10),
                  decoration: BoxDecoration(
                      color: const Color(0xFF181818),
                      borderRadius: BorderRadius.circular(10),
                      border: Border.all(color: const Color(0xFFf06922))),
                  child: Text(
                    slideTitles[index],
                    textAlign: TextAlign.center,
                    style: const TextStyle(
                      color: Colors.white,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          Center(child: slideshow(slide)),
          clock(),
          Positioned(
            left: 0,
            right: 0,
            bottom: 0,
            height: 120, // Increase this height if needed
            child: MouseRegion(
              onEnter: (_) => setState(() => isHovering = true),
              onExit: (_) => setState(() => isHovering = false),
              child: Container(
                color: Colors.transparent,
                child: _buildThumbnails(),
              ),
            ),
          ),
          Positioned(
            bottom: 10,
            right: 10,
            child: IconButton(
              icon: Icon(_timer != null && _timer!.isActive
                  ? Icons.pause
                  : Icons.play_arrow),
              onPressed: _toggleSlideShow,
              // tooltip: 'Toggle Slideshow',
            ),
          ),
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
        ],
      ),
    );
  }
}
