import 'package:flutter/material.dart';
import 'package:window_manager/window_manager.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await windowManager.ensureInitialized();


  windowManager.waitUntilReadyToShow(
    const WindowOptions(
      center: true,
      title: 'Diagnostic Display',
      size: Size(1920, 1080),
    ), () async {
      await windowManager.show();
      await windowManager.focus();
      await windowManager.setFullScreen(false);
    }
  );

  

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
          brightness: Brightness.dark
        ),
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
    return Scaffold(
      // appBar: AppBar(
      //   backgroundColor: Theme.of(context).colorScheme.primary,
      //   title: const Text('Diagnostic Display'),
      // ),
    );
  }
}

