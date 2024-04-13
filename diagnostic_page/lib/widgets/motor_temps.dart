import 'package:flutter/material.dart';
import 'package:diagnostic_page/services/subsystem_state.dart';

Widget motorTemps() {
  return Card(
    child: SizedBox(
      height: 300,
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Row(
          mainAxisSize: MainAxisSize.max,
          children: [
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  _motorTemp('Left Pivot', 0),
                  _motorTemp('Right Pivot', 1),
                  _motorTemp('Right Climb', 2),
                  _motorTemp('Right Climb', 3),
                  _motorTemp('Front Intake', 4),
                  _motorTemp('Back Intake', 5),
                  _motorTemp('Indexer', 7),
                  _motorTemp('Left Shooter', 6),
                  _motorTemp('Right Shooter', 8),
                ],
              ),
            ),
          ],
        ),
      ),
    ),
  );
}

Widget _motorTemp(String name, int motorTempIndex) {
  return StreamBuilder(
    stream: SubsystemState.getMotorTemp(motorTempIndex),
    builder: (context, snapshot) {
      double temp = snapshot.data ?? 0;
      Color tempColor = Colors.green;
      if (temp >= 80) {
        tempColor = Colors.red;
      } else if (temp >= 60) {
        tempColor = Colors.yellow;
      }

      return Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            name,
            style: const TextStyle(fontSize: 20),
          ),
          Text(
            '${temp.round()}°C',
            style: TextStyle(fontSize: 20, color: tempColor),
          ),
        ],
      );
    },
  );
}
