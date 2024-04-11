import 'package:flutter/material.dart';
import 'package:nt4/nt4.dart';

class SubsystemState {
  static const String _robotIP =
      "127.0.0.1"; // real: 10.17.87.2  |  debug: 127.0.0.1
  static bool _isConnected = false;
  static late NT4Client _client;

  static late NT4Subscription _armRanCheckSub;
  static late NT4Subscription _armStatusSub;
  static late NT4Subscription _armFaultsSub;

  static late NT4Topic _armCheckRunningTopic;

  static final ValueNotifier<bool> ranCheckNotifier =
      ValueNotifier<bool>(false);
  // static late NT4Topic _allSystemsCheckRunningTopic;

  static Stream<bool> connectionStatus() async* {
    yield _isConnected;
    bool lastYielded = _isConnected;

    while (true) {
      if (_isConnected != lastYielded) {
        yield _isConnected;
        lastYielded = _isConnected;
      }
      await Future.delayed(const Duration(seconds: 1));
    }
  }

  static void init() {
    _client = NT4Client(
      serverBaseAddress: _robotIP,
      onConnect: () {
        print("connected");
        _isConnected = true;
      },
      onDisconnect: () {
        print("disconnected");
        _isConnected = false;
      },
    );

    _armRanCheckSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Arm/RanCheck');
    _armStatusSub =
        _client.subscribePeriodic('/SmartDashboard/SubsystemStatus/Arm/Status');
    _armFaultsSub =
        _client.subscribePeriodic('/SmartDashboard/SubsystemStatus/Arm/Faults');

    _armCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SubsystemStatus/Arm/SubsystemCheck/running',
        NT4TypeStr.typeBool);
  }

  static Stream<SubsystemStatus> armStatus() async* {
    while (true) {
      // Attempt to safely cast the values. Adjust the types as necessary.
      bool ranCheck = false;
      String status = '';
      String faults = '';

      var ranCheckValue = _armRanCheckSub.currentValue;
      if (ranCheckValue is bool) {
        ranCheck = ranCheckValue;
      }

      var statusValue = _armStatusSub.currentValue;
      if (statusValue is String) {
        status = statusValue;
      } else if (statusValue is List) {
        // If the value is a list, handle accordingly. This is just an example.
        status = statusValue.join(", ");
      }

      var faultsValue = _armFaultsSub.currentValue;
      if (faultsValue is String) {
        faults = faultsValue;
      } else if (faultsValue is List) {
        // Handle the list case
        faults = faultsValue.join(", ");
      }

      // print(
      // 'Emitting armStatus: ranCheck: $ranCheck, status: $status, faults: $faults');

      yield SubsystemStatus(ranCheck: ranCheck, status: status, faults: faults);
      await Future.delayed(
          const Duration(milliseconds: 500)); // Adjust the delay as needed
    }
  }

  static Stream<SubsystemStatus> subsystemStatus(String subsystem) async* {
    NT4Subscription ranCheckSub;
    NT4Subscription statusSub;
    NT4Subscription faultsSub;
    switch (subsystem.toLowerCase()) {
      case "arm":
        ranCheckSub = _armRanCheckSub;
        statusSub = _armStatusSub;
        faultsSub = _armFaultsSub;
      default:
        ranCheckSub = NT4Subscription(topic: "topic");
        statusSub = NT4Subscription(topic: "topic");
        faultsSub = NT4Subscription(topic: "topic");
    }

    while (true) {
      // Attempt to safely cast the values. Adjust the types as necessary.
      bool ranCheck = false;
      String status = '';
      String faults = '';

      var ranCheckValue = ranCheckSub.currentValue;
      if (ranCheckValue is bool) {
        ranCheck = ranCheckValue;
      }

      var statusValue = statusSub.currentValue;
      if (statusValue is String) {
        status = statusValue;
      } else if (statusValue is List) {
        // If the value is a list, handle accordingly. This is just an example.
        status = statusValue.join(", ");
      }

      var faultsValue = faultsSub.currentValue;
      if (faultsValue is String) {
        faults = faultsValue;
      } else if (faultsValue is List) {
        // Handle the list case
        faults = faultsValue.join(", ");
      }

      yield SubsystemStatus(ranCheck: ranCheck, status: status, faults: faults);
      await Future.delayed(
          const Duration(milliseconds: 500)); // Adjust the delay as needed
    }
  }

  static Future<void> startSubsystemTest(String subsystem) async {
    switch (subsystem.toLowerCase()) {
      case "arm":
        return startArmTest();
      default:
    }
  }

  static Future<void> startArmTest() async {
    // Assuming this is where you'd start the test
    _client.addSample(_armCheckRunningTopic, true);

    // Wait for the test to "start"
    await Future.delayed(const Duration(milliseconds: 200));
    // This is a simplistic approach. Ideally, you'd wait for a signal that the test has indeed completed.

    // Signal that the check has run
    ranCheckNotifier.value = true;

    // Optionally, wait for a certain condition to be met before proceeding
    // This is a placeholder for any additional logic you need to ensure the test is complete
  }

  static void isRan() async {
    print(_armRanCheckSub.currentValue);
  }
}

class SubsystemStatus {
  final bool ranCheck;
  final String status;
  final String faults;

  const SubsystemStatus({
    required this.ranCheck,
    required this.status,
    required this.faults,
  });
}
