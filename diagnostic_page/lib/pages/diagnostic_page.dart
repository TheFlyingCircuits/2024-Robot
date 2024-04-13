import 'package:diagnostic_page/services/subsystem_state.dart';
import 'package:diagnostic_page/widgets/clock.dart';
import 'package:diagnostic_page/widgets/graph.dart';
import 'package:diagnostic_page/widgets/motor_temps.dart';
import 'package:diagnostic_page/widgets/status_card.dart';
import 'package:flutter/material.dart';

class DiagnosticPage extends StatefulWidget {
  const DiagnosticPage({super.key});

  @override
  State<DiagnosticPage> createState() => _DiagnosticPageState();
}

class _DiagnosticPageState extends State<DiagnosticPage> {
  static const double padding = 8.0;
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
        Positioned(
          bottom: 2 * padding,
          left: 2 * padding,
          child: clock(),
        ),
        Row(
          children: [
            Expanded(
              flex: 7,
              child: Column(
                children: [
                  Padding(
                    padding: const EdgeInsets.all(padding),
                    child: Row(
                      // mainAxisSize: MainAxisSize.max,
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Expanded(
                          child: Column(
                            children: [
                              Row(
                                children: [
                                  Expanded(
                                      child: customGraph(
                                    "riocan",
                                    [Colors.green, Colors.yellow],
                                    100,
                                  )),
                                  Expanded(
                                      child: customGraph("canivorecan",
                                          [Colors.green, Colors.yellow], 100)),
                                ],
                              ),
                              Row(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Expanded(
                                      child: motorTemps(),
                                    ),
                                    Expanded(
                                      child: customGraph("batteryvoltage",
                                          [Colors.red, Colors.orange], 16,
                                          horizontalLineHeight: 12),
                                    ),
                                  ]),
                              // Removed the Row containing the Clock widget
                            ],
                          ),
                        ),
                        Column(
                          children: [
                            statusCard(Subsystem.arm),
                            statusCard(Subsystem.climb),
                            statusCard(Subsystem.intake),
                            statusCard(Subsystem.indexer),
                            statusCard(Subsystem.shooter),
                            const SizedBox(height: 4),
                            const SizedBox(
                              width: 500,
                              child: FloatingActionButton.extended(
                                onPressed:
                                    SubsystemState.startAllSubsystemTests,
                                label: Text('Run All Checks'),
                                icon: Icon(Icons.check),
                              ),
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
