import 'dart:async';
import 'package:flutter/material.dart';

class TimeDisplay extends StatefulWidget {
  const TimeDisplay({super.key});

  @override
  TimeDisplayState createState() => TimeDisplayState();
}

class TimeDisplayState extends State<TimeDisplay> {
  late Timer _timer;
  late String _timeString;
  late String _dateString;

  @override
  void initState() {
    super.initState();
    _updateTimeAndDate();
    _syncTimerWithSystemClock();
  }

  @override
  void dispose() {
    _timer.cancel();
    super.dispose();
  }

  void _syncTimerWithSystemClock() {
    final DateTime now = DateTime.now();
    final int secondsToNextMinute = 60 - now.second;
    final Duration initialDelay = Duration(seconds: secondsToNextMinute);
    _timer = Timer(initialDelay, () {
      _updateTimeAndDate();
      // After the initial delay, set the timer to tick every minute
      _timer = Timer.periodic(
          const Duration(minutes: 1), (Timer t) => _updateTimeAndDate());
    });
  }

  void _updateTimeAndDate() {
    final DateTime now = DateTime.now();
    _timeString = _formatDateTime(now);
    _dateString = _formatDate(now);
    setState(() {}); // Trigger rebuild with updated time and date
  }

  String _formatDateTime(DateTime dateTime) {
    String hour =
        dateTime.hour % 12 == 0 ? '12' : (dateTime.hour % 12).toString();
    String minute = dateTime.minute.toString().padLeft(2, '0');
    String period = dateTime.hour >= 12 ? 'PM' : 'AM';
    return '$hour:$minute $period';
  }

  String _formatDate(DateTime dateTime) {
    List<String> months = [
      'January',
      'February',
      'March',
      'April',
      'May',
      'June',
      'July',
      'August',
      'September',
      'October',
      'November',
      'December'
    ];
    return '${months[dateTime.month - 1]} ${dateTime.day}, ${dateTime.year}';
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Text(
          _timeString,
          style: const TextStyle(
            fontSize: 64,
            fontWeight: FontWeight.bold,
          ),
        ),
        Text(
          _dateString,
          style: const TextStyle(
            fontSize: 28,
            color: Colors.grey,
          ),
        ),
      ],
    );
  }
}

Widget clock() {
  return const Positioned(
    bottom: 16,
    left: 16,
    child: TimeDisplay(),
  );
}
