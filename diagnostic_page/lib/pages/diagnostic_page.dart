import 'package:diagnostic_page/services/subsystem_state.dart';
import 'package:diagnostic_page/widgets/status_card.dart';
import 'package:flutter/material.dart';

class DiagnosticPage extends StatefulWidget {
  const DiagnosticPage({super.key});

  @override
  State<DiagnosticPage> createState() => _DiagnosticPageState();
}

class _DiagnosticPageState extends State<DiagnosticPage> {
  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned(
          bottom: 0,
          right: 0,
          child: SizedBox(
            width: 500,
            height: 400,
            child: FittedBox(
              fit: BoxFit.contain,
              child: Image.asset(
                "images/logo.png",
                filterQuality: FilterQuality.medium,
              ),
            ),
          ),
        ),
        Row(
          children: [
            Expanded(
              flex: 7,
              child: Column(
                children: [
                  Padding(
                    padding: const EdgeInsets.fromLTRB(0.0, 8.0, 8.0, 8.0),
                    child: Row(
                      // mainAxisSize: MainAxisSize.max,
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        // TODO: expanded shouldnt be const
                        const Expanded(
                          child: Column(
                            children: [
                              // const MotorTemps(),
                              // const PDHChannels(),
                              // TODO: row shouldnt be const
                              Row(
                                children: [
                                  Expanded(child: Text("CodePerformanceGraph")),
                                  Expanded(child: Text("InputVoltageGraph")),
                                  // Expanded(child: CodePerformanceGraph()),
                                  // Expanded(child: InputVoltageGraph()),
                                ],
                              ),
                              Row(
                                children: [
                                  Expanded(
                                    child: Text("CANUtilGraph"),
                                  )
                                  // Expanded(child: CANUtilGraph()),
                                ],
                              ),
                            ],
                          ),
                        ),
                        Column(
                          children: [
                            // statusCard("intake"),

                            statusCard("arm"),

                            const SizedBox(height: 4),
                            const SizedBox(
                              width: 500,
                              child: FloatingActionButton.extended(
                                onPressed: SubsystemState.startArmTest,
                                label: Text('Run All Checks'),
                                icon: Icon(Icons.check),
                              ),
                            ),
                            StreamBuilder(
                              stream: SubsystemState.connectionStatus(),
                              builder: (context, snapshot) {
                                bool connected = snapshot.data ?? false;

                                if (connected) {
                                  return const Text(
                                    'Robot Status: Connected',
                                    style: TextStyle(color: Colors.green),
                                  );
                                } else {
                                  return const Text(
                                    'Robot Status: Disconnected',
                                    style: TextStyle(color: Colors.red),
                                  );
                                }
                              },
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ],
    );
  }
}
