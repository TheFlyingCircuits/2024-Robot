import 'package:diagnostic_page/services/subsystem_state.dart';
import 'package:flutter/material.dart';

Widget statusCard(Subsystem subsystem) {
  RegExp pattern = RegExp(r'^\[\d+\.\d+\]\s');
  return ValueListenableBuilder(
    valueListenable: SubsystemState.ranCheckNotifiers[subsystem]!,
    builder: (context, bool ranCheckValue, child) {
      return Card(
        child: SizedBox(
          width: 500,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              StreamBuilder(
                stream: SubsystemState.subsystemStatus(subsystem),
                builder: (context, AsyncSnapshot<SubsystemStatus> snapshot) {
                  if (!snapshot.hasData) {
                    return const Text('Waiting for data');
                  }
                  SubsystemStatus status = snapshot.data!;
                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;

                  if (ranCheckValue) {
                    switch (status.status) {
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
                  } else {
                    statusColor = Colors.grey;
                    statusIcon = Icons.question_mark_rounded;
                  }

                  bool isRunning = !ranCheckValue &&
                      snapshot.connectionState == ConnectionState.waiting;

                  return ListTile(
                    title: Text(
                      '${subsystem.name[0].toUpperCase()}${subsystem.name.substring(1).toLowerCase()}',
                      style: const TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: ranCheckValue && status.faults.isNotEmpty
                        ? Text(status.faults
                            .split(', ')
                            .map((str) => str.replaceAll(pattern, ''))
                            .join('\n'))
                        : null,
                    trailing: ElevatedButton(
                      onPressed: isRunning
                          ? null
                          : () async {
                              await SubsystemState.startSubsystemTest(
                                  subsystem);
                            },
                      style: ElevatedButton.styleFrom(
                        backgroundColor: isRunning
                            ? Colors.grey
                            : Theme.of(context).primaryColor,
                        disabledBackgroundColor: Colors.grey,
                      ),
                      child: Text(isRunning ? 'Running' : 'Run Check'),
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
