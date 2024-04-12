import 'package:diagnostic_page/services/subsystem_state.dart';
import 'package:flutter/material.dart';

Widget statusCard(String subsystem) {
  RegExp pattern = RegExp(r'^\[\d+\.\d+\]\s');
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
                stream: SubsystemState.subsystemStatus(subsystem),
                builder: (context, snapshot) {
                  if (!snapshot.hasData) {
                    return const Text(
                        "waiting for data"); // Show loading indicator while waiting for data
                  }
                  SubsystemStatus status = snapshot.data!;
                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;

                  if (ranCheckValue) {
                    switch (status.status) {
                      // case 'NULL':
                      //   statusColor = Colors.grey;
                      //   statusIcon = Icons.question_mark_rounded;
                      //   break;
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
                    title: Text(
                      "${subsystem[0].toUpperCase()}${subsystem.substring(1).toLowerCase()}",
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.faults.isNotEmpty
                        ? Text(status.faults
                            .split(", ")
                            .map((str) => str.replaceAll(pattern, ""))
                            .join("\n"))
                        : null,
                    trailing: ElevatedButton(
                      onPressed: () async {
                        // Disable the button while the test is running
                        await SubsystemState.startSubsystemTest(subsystem);
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
