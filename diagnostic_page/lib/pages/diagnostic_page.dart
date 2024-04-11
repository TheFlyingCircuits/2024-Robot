import 'package:diagnostic_page/services/subsystem_state.dart';
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
                            // _swerveStatusCard(),
                            _armStatusCard(),
                            // _turretStatusCard(),
                            // _jawStatusCard(),
                            // _intakeStatusCard(),
                            // _manhattanStatusCard(),
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

  Widget _armStatusCard() {
    return ValueListenableBuilder(
      valueListenable: SubsystemState.ranCheckNotifier,
      builder: (context, bool ranCheckValue, child) {
        return Card(
          child: SizedBox(
            width: 500,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                StreamBuilder(
                  stream: SubsystemState.armStatus(),
                  builder: (context, snapshot) {
                    if (!snapshot.hasData) {
                      return Text(DateTime.now()
                          .second
                          .toString()); // Show loading indicator while waiting for data
                    }
                    SubsystemStatus status = snapshot.data!;
                    Color statusColor = Colors.grey;
                    IconData statusIcon = Icons.question_mark_rounded;

                    if (ranCheckValue) {
                      switch (status.status) {
                        case 'NULL':
                          statusColor = Colors.grey;
                          statusIcon = Icons.question_mark_rounded;
                          break;
                        case 'OK':
                          statusColor = Colors.green;
                          statusIcon = Icons.check_circle_outline_rounded;
                          break;
                        case 'WARNING':
                          statusColor = Colors.yellow;
                          statusIcon = Icons.warning_amber_rounded;
                          break;
                        case 'ERROR':
                          statusColor = Colors.red;
                          statusIcon = Icons.error_outline_rounded;
                          break;
                        default:
                          statusColor = Colors.grey;
                          statusIcon = Icons.question_mark_rounded;
                          break;
                      }
                    }

                    return ListTile(
                      title: const Text(
                        'Arm',
                        style: TextStyle(fontSize: 20),
                      ),
                      leading: Icon(
                        statusIcon,
                        color: statusColor,
                      ),
                      subtitle:
                          status.faults.isNotEmpty ? Text(status.faults) : null,
                      trailing: ElevatedButton(
                        onPressed: () async {
                          // Disable the button while the test is running
                          await SubsystemState.startArmTest();
                          // No need to manually update the UI; the stream and ValueNotifier will handle this
                        },
                        child: Text('Run Check'),
                      ),
                    );
                  },
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
