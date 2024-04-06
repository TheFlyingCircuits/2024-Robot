import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:window_manager/window_manager.dart';

void main() async {
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

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
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
      home: const DiagnosticDisplay(),
    );
  }
}

class DiagnosticDisplay extends StatefulWidget {
  const DiagnosticDisplay({super.key});

  @override
  State<DiagnosticDisplay> createState() => _DiagnosticDisplayState();
}

class _DiagnosticDisplayState extends State<DiagnosticDisplay> {
  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    throw UnimplementedError();
  }

  bool _onKey(KeyEvent event) {
    final key = event.logicalKey.keyLabel;

    if (event is KeyDownEvent) {
      print("Key down: $key");
    } else if (event is KeyUpEvent) {
      print("Key up: $key");
    } else if (event is KeyRepeatEvent) {
      print("Key repeat: $key");
    }

    return false;
  }

  @override
  void initState() {
    super.initState();
    ServicesBinding.instance.keyboard.addHandler(_onKey);
  }

  @override
  void dispose() {
    ServicesBinding.instance.keyboard.removeHandler(_onKey);
    super.dispose();
  }
}
