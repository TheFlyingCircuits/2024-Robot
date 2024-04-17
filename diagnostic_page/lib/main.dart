import 'dart:async';

import 'package:diagnostic_page/pages/diagnostic_page.dart';
import 'package:diagnostic_page/pages/slideshow_page.dart';
import 'package:diagnostic_page/services/subsystem_state.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:media_kit/media_kit.dart';
import 'package:window_manager/window_manager.dart';

Future<void> main() async {
  SubsystemState.init();

  WidgetsFlutterBinding.ensureInitialized();
  await windowManager.ensureInitialized();

  double physicalWidth =
      WidgetsBinding.instance.platformDispatcher.views.first.physicalSize.width;
  double physicalHeight = WidgetsBinding
      .instance.platformDispatcher.views.first.physicalSize.height;

  double devicePixelRatio =
      WidgetsBinding.instance.platformDispatcher.views.first.devicePixelRatio;
  double height = physicalHeight / devicePixelRatio;
  double width = physicalWidth / devicePixelRatio;

  windowManager.waitUntilReadyToShow(
      WindowOptions(
        center: true,
        title: 'Diagnostic Display',
        size: Size(width, height),
      ), () async {
    await windowManager.show();
    await windowManager.focus();
    await windowManager.setFullScreen(true);
  });
  MediaKit.ensureInitialized();

  runApp(const MyApp());
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  MyAppState createState() => MyAppState();
}

class MyAppState extends State<MyApp> {
  final FocusNode _focusNode = FocusNode();
  final GlobalKey<NavigatorState> _navigatorKey = GlobalKey<NavigatorState>();
  StreamSubscription<bool>? _connectionStatusSubscription;

  @override
  void initState() {
    super.initState();

    // Listen to the connection status
    _connectionStatusSubscription = SubsystemState.connectionStatus().listen(
      (connected) {
        if (connected) {
          // Navigate to the Diagnostic Page if not already there
          if (_navigatorKey.currentState?.canPop() ?? false) {
            _navigatorKey.currentState?.pop();
          } else {
            _navigatorKey.currentState?.push(
                MaterialPageRoute(builder: (_) => const DiagnosticPage()));
          }
        } else {
          // Navigate back to the home screen if the Diagnostic Page is displayed
          if (_navigatorKey.currentState?.canPop() ?? false) {
            _navigatorKey.currentState?.pop();
          }
        }
      },
      onError: (e) {
        debugPrint('Stream error: $e');
      },
    );
  }

  @override
  void dispose() {
    _focusNode.dispose();
    _connectionStatusSubscription?.cancel();
    super.dispose();
  }

  void _handleKeyEvent(KeyEvent event) async {
    if (event is KeyDownEvent) {
      if (event.logicalKey == LogicalKeyboardKey.keyN) {
        if (_navigatorKey.currentState!.canPop()) {
          _navigatorKey.currentState!.pop();
        } else {
          _navigatorKey.currentState!.push(PageRouteBuilder(
            pageBuilder: (context, animation1, animation2) =>
                const DiagnosticPage(),
            transitionDuration:
                const Duration(seconds: 0), // No transition time
          ));
        }
      }
      if (event.logicalKey == LogicalKeyboardKey.enter) {
        try {
          bool isFullScreen = await windowManager.isFullScreen();
          await windowManager.setFullScreen(!isFullScreen);
        } catch (e) {
          debugPrint('Error toggling full screen: $e');
        }
      }
      // Re-request focus on the main page's FocusNode after navigation
      WidgetsBinding.instance
          .addPostFrameCallback((_) => _focusNode.requestFocus());
    }
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Diagnostic Display',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
            seedColor: const Color(0xFFf06922),
            primary: const Color(0xFFf06922),
            background: const Color(0xFF181818),
            brightness: Brightness.dark),
      ),
      home: Scaffold(
        body: Navigator(
          key: _navigatorKey,
          onGenerateRoute: (settings) => PageRouteBuilder(
            pageBuilder: (context, animation1, animation2) {
              return KeyboardListener(
                focusNode: _focusNode,
                onKeyEvent: _handleKeyEvent,
                child: const Focus(
                  autofocus: true,
                  child: Center(child: SlideshowPage()),
                ),
              );
            },
            transitionDuration:
                const Duration(seconds: 0), // No transition time
          ),
        ),
      ),
    );
  }
}
