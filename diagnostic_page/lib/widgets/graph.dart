import 'package:fl_chart/fl_chart.dart';
import 'package:flutter/material.dart';
import 'package:diagnostic_page/services/subsystem_state.dart';

Widget customGraph(String graphType, List<Color> gradientColors, double maxY,
    {double horizontalLineHeight = 0}) {
  String name;
  List<Color> gradientColorsTranslucent = [
    gradientColors[0].withOpacity(0.3),
    gradientColors[1].withOpacity(0.3)
  ];

  switch (graphType.toLowerCase()) {
    case "riocan":
      name = "RoboRIO CAN Utilization";
      break;
    case "canivorecan":
      name = "CANivore CAN Utilization";
      break;
    case "batteryvoltage":
      name = "Battery Voltage";
      break;
    default:
      name = "no graph name set";
  }
  return Card(
    clipBehavior: Clip.hardEdge,
    child: SizedBox(
      height: 200,
      child: Stack(
        children: [
          StreamBuilder(
              stream: SubsystemState.getPlot(graphType),
              builder: (context, snapshot) {
                List<FlSpot> data = [];
                if (snapshot.hasData) {
                  for (int i = 0; i < snapshot.data!.length; i++) {
                    data.add(FlSpot(0.033 * i, snapshot.data![i]));
                  }
                }
                return LineChart(
                  duration: Duration.zero,
                  LineChartData(
                      lineTouchData: const LineTouchData(
                        enabled: false,
                      ),
                      titlesData: const FlTitlesData(show: false),
                      gridData: FlGridData(
                        show: true,
                        drawVerticalLine: true,
                        drawHorizontalLine: false,
                        // horizontalInterval: 25,
                        verticalInterval: 2.5,
                        getDrawingHorizontalLine: (value) {
                          return FlLine(
                            color: Colors.grey.withOpacity(0.3),
                            strokeWidth: 1,
                          );
                        },
                        getDrawingVerticalLine: (value) {
                          return FlLine(
                            color: Colors.grey.withOpacity(0.3),
                            strokeWidth: 1,
                          );
                        },
                      ),
                      borderData: FlBorderData(
                        show: false,
                      ),
                      minX: 0,
                      minY: -.01,
                      maxX: 20,
                      maxY: maxY,
                      lineBarsData: [
                        staticHorizontalLine(
                            horizontalLineHeight, gradientColors),
                        LineChartBarData(
                          spots: data,
                          isCurved: true,
                          gradient: LinearGradient(
                            colors: gradientColors,
                          ),
                          barWidth: 2,
                          isStrokeCapRound: true,
                          dotData: const FlDotData(show: false),
                          belowBarData: BarAreaData(
                            show: true,
                            gradient: LinearGradient(
                                colors: gradientColorsTranslucent),
                          ),
                        ),
                      ]),
                );
              }),
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Align(
              alignment: Alignment.topCenter,
              child: Text(
                name,
                style:
                    const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              ),
            ),
          ),
        ],
      ),
    ),
  );
}

LineChartBarData staticHorizontalLine(
    double horizontalLineHeight, List<Color> gradientColors) {
  List<FlSpot> data = [];
  for (int i = 0; i < 608; i++) {
    data.add(FlSpot(0.033 * i, horizontalLineHeight));
  }

  LineChartBarData bar = LineChartBarData(
      spots: data,
      gradient: LinearGradient(colors: gradientColors),
      dotData: const FlDotData(show: false),
      barWidth: 1,
      show: horizontalLineHeight > 0 ? true : false);
  return bar;
}
