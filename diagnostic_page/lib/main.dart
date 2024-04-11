import 'package:diagnostic_page/pages/diagnostic_page.dart';
import 'package:diagnostic_page/services/subsystem_state.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
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
    await windowManager.setFullScreen(false);
  });

  runApp(const MyApp());
}

class MyApp extends StatefulWidget {
  const MyApp({Key? key}) : super(key: key);

  @override
  MyAppState createState() => MyAppState();
}

class MyAppState extends State<MyApp> {
  final FocusNode _focusNode = FocusNode();
  final GlobalKey<NavigatorState> _navigatorKey = GlobalKey<NavigatorState>();

  @override
  void initState() {
    super.initState();
    SubsystemState.armStatus().listen(
      (status) {
        // print("Received status: ${status.status}");
      },
      onError: (e) {
        print("Stream error: $e");
      },
    );
  }

  @override
  void dispose() {
    _focusNode.dispose();
    super.dispose();
  }

  void _handleKeyEvent(KeyEvent event) async {
    if (event is KeyDownEvent) {
      if (event.logicalKey == LogicalKeyboardKey.keyN) {
        if (_navigatorKey.currentState!.canPop()) {
          _navigatorKey.currentState!.pop();
        } else {
          _navigatorKey.currentState!
              .push(MaterialPageRoute(builder: (_) => const DiagnosticPage()));
        }
      }
      if (event.logicalKey == LogicalKeyboardKey.enter) {
        print("enter pressed");
        try {
          bool isFullScreen = await windowManager.isFullScreen();
          // No direct null check here since isFullScreen returns a Future<bool>
          await windowManager.setFullScreen(!isFullScreen);
        } catch (e) {
          print('Error toggling full screen: $e');
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
      title: "Diagnostic Display",
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
          onGenerateRoute: (settings) => MaterialPageRoute(
            builder: (context) => KeyboardListener(
              focusNode: _focusNode,
              onKeyEvent: _handleKeyEvent,
              child: const Focus(
                autofocus: true,
                child: Center(
                  child: Text(
                      'Unfinished home screen. Press enter to go fullscreen, press N to switch to diagnostics'),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
