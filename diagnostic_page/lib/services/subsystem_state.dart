import 'dart:async';

import 'package:flutter/material.dart';
import 'package:nt4/nt4.dart';

class SubsystemState {
  static const String _robotIP =
      '127.0.0.1'; // real: 10.17.87.2  |  sim: 127.0.0.1
  static bool _isConnected = false;
  static late NT4Client _client;

  static final Map<Subsystem, ValueNotifier<bool>> ranCheckNotifiers = {
    Subsystem.arm: ValueNotifier<bool>(false),
    Subsystem.climb: ValueNotifier<bool>(false),
    Subsystem.intake: ValueNotifier<bool>(false),
    Subsystem.indexer: ValueNotifier<bool>(false),
    Subsystem.shooter: ValueNotifier<bool>(false),
  };
  //   // static final ValueNotifier<bool> ranCheckNotifier =
  //     ValueNotifier<bool>(false);
  static final List<NT4Subscription> _motorTempSubs = [];

  static late NT4Subscription _armRanCheckSub;
  static late NT4Subscription _armStatusSub;
  static late NT4Subscription _armFaultsSub;
  static late NT4Subscription _armCheckRunningSub;
  static late NT4Topic _armCheckRunningTopic;

  static late NT4Subscription _climbRanCheckSub;
  static late NT4Subscription _climbStatusSub;
  static late NT4Subscription _climbFaultsSub;
  static late NT4Subscription _climbCheckRunningSub;
  static late NT4Topic _climbCheckRunningTopic;

  // static late NT4Subscription _drivetrainRanCheckSub;
  // static late NT4Subscription _drivetrainStatusSub;
  // static late NT4Subscription _drivetrainFaultsSub;
  // static late NT4Subscription _drivetrainCheckRunningSub;
  // static late NT4Topic _drivetrainCheckRunningTopic;

  static late NT4Subscription _intakeRanCheckSub;
  static late NT4Subscription _intakeStatusSub;
  static late NT4Subscription _intakeFaultsSub;
  static late NT4Subscription _intakeCheckRunningSub;
  static late NT4Topic _intakeCheckRunningTopic;

  static late NT4Subscription _indexerRanCheckSub;
  static late NT4Subscription _indexerStatusSub;
  static late NT4Subscription _indexerFaultsSub;
  static late NT4Subscription _indexerCheckRunningSub;
  static late NT4Topic _indexerCheckRunningTopic;

  static late NT4Subscription _shooterRanCheckSub;
  static late NT4Subscription _shooterStatusSub;
  static late NT4Subscription _shooterFaultsSub;
  static late NT4Subscription _shooterCheckRunningSub;
  static late NT4Topic _shooterCheckRunningTopic;

  static late NT4Subscription _rioCANUtilSub;
  static late NT4Subscription _canivoreCANUtilSub;
  static late NT4Subscription _batteryVoltageSub;

  // static final ValueNotifier<bool> ranCheckNotifier =
  //     ValueNotifier<bool>(false);
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
        print('connected');
        _isConnected = true;
      },
      onDisconnect: () {
        print('disconnected');
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
    _armCheckRunningSub = _client.subscribePeriodic(
      '/SmartDashboard/SubsystemStatus/Arm/SubsystemCheck/running',
    );

    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Arm/MotorTemps/leftPivot'));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Arm/MotorTemps/rightPivot'));

    _climbRanCheckSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Climb/RanCheck');
    _climbStatusSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Climb/Status');
    _climbFaultsSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Climb/Faults');

    _climbCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SubsystemStatus/Climb/SubsystemCheck/running',
        NT4TypeStr.typeBool);
    _climbCheckRunningSub = _client.subscribePeriodic(
      '/SmartDashboard/SubsystemStatus/Climb/SubsystemCheck/running',
    );

    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Climb/MotorTemps/leftClimb'));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Climb/MotorTemps/rightClimb'));

    // _drivetrainRanCheckSub = _client.subscribePeriodic(
    //     '/SmartDashboard/SubsystemStatus/Drivetrain/RanCheck');
    // _drivetrainStatusSub = _client
    //     .subscribePeriodic('/SmartDashboard/SubsystemStatus/Drivetrain/Status');
    // _drivetrainFaultsSub = _client
    //     .subscribePeriodic('/SmartDashboard/SubsystemStatus/Drivetrain/Faults');

    // _drivetrainCheckRunningTopic = _client.publishNewTopic(
    //     '/SmartDashboard/SubsystemStatus/Drivetrain/SubsystemCheck/running',
    //     NT4TypeStr.typeBool);
    // _drivetrainCheckRunningSub = _client.subscribePeriodic(
    //   '/SmartDashboard/SubsystemStatus/Drivetrain/SubsystemCheck/running',
    // );

    _intakeRanCheckSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Intake/RanCheck');
    _intakeStatusSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Intake/Status');
    _intakeFaultsSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Intake/Faults');

    _intakeCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SubsystemStatus/Intake/SubsystemCheck/running',
        NT4TypeStr.typeBool);
    _intakeCheckRunningSub = _client.subscribePeriodic(
      '/SmartDashboard/SubsystemStatus/Intake/SubsystemCheck/running',
    );

    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Intake/MotorTemps/frontIntake'));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Intake/MotorTemps/backIntake'));

    _indexerRanCheckSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Indexer/RanCheck');
    _indexerStatusSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Indexer/Status');
    _indexerFaultsSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Indexer/Faults');

    _indexerCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SubsystemStatus/Indexer/SubsystemCheck/running',
        NT4TypeStr.typeBool);
    _indexerCheckRunningSub = _client.subscribePeriodic(
      '/SmartDashboard/SubsystemStatus/Indexer/SubsystemCheck/running',
    );

    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Indexer/MotorTemps/indexer'));

    _shooterRanCheckSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Shooter/RanCheck');
    _shooterStatusSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Shooter/Status');
    _shooterFaultsSub = _client
        .subscribePeriodic('/SmartDashboard/SubsystemStatus/Shooter/Faults');

    _shooterCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SubsystemStatus/Shooter/SubsystemCheck/running',
        NT4TypeStr.typeBool);
    _shooterCheckRunningSub = _client.subscribePeriodic(
      '/SmartDashboard/SubsystemStatus/Shooter/SubsystemCheck/running',
    );

    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Shooter/MotorTemps/leftShooter'));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/SmartDashboard/SubsystemStatus/Shooter/MotorTemps/rightShooter'));

    _rioCANUtilSub = _client.subscribePeriodic('/SmartDashboard/RIOCANUtil');
    _canivoreCANUtilSub =
        _client.subscribePeriodic('/SmartDashboard/CANivoreCANUtil');
    // TODO: testing only!!
    // _batteryVoltageSub =
    //     _client.subscribePeriodic('/AdvantageKit/SystemStats/BatteryVoltage');
    _batteryVoltageSub =
        _client.subscribePeriodic('/SmartDashboard/BatteryVoltage');

    // for (int i = 0; i < _motorTempSubs.length; i++) {
    //   print(_motorTempSubs[i].topic);
    // }
  }

  static final Map<Subsystem, StreamController<SubsystemStatus>>
      _statusControllers = {};

  static Stream<SubsystemStatus> subsystemStatus(Subsystem subsystem) {
    NT4Subscription ranCheckSub;
    NT4Subscription statusSub;
    NT4Subscription faultsSub;
    NT4Subscription checkRunningSub;

    switch (subsystem) {
      case Subsystem.arm:
        ranCheckSub = _armRanCheckSub;
        statusSub = _armStatusSub;
        faultsSub = _armFaultsSub;
        checkRunningSub = _armCheckRunningSub;
        break;
      case Subsystem.climb:
        ranCheckSub = _climbRanCheckSub;
        statusSub = _climbStatusSub;
        faultsSub = _climbFaultsSub;
        checkRunningSub = _climbCheckRunningSub;
        break;
      // case 'drivetrain':
      //   ranCheckSub = _drivetrainRanCheckSub;
      //   statusSub = _drivetrainStatusSub;
      //   faultsSub = _drivetrainFaultsSub;
      //   checkRunningSub = _drivetrainCheckRunningSub;
      //   break;
      case Subsystem.intake:
        ranCheckSub = _intakeRanCheckSub;
        statusSub = _intakeStatusSub;
        faultsSub = _intakeFaultsSub;
        checkRunningSub = _intakeCheckRunningSub;
        break;
      case Subsystem.indexer:
        ranCheckSub = _indexerRanCheckSub;
        statusSub = _indexerStatusSub;
        faultsSub = _indexerFaultsSub;
        checkRunningSub = _indexerCheckRunningSub;
        break;
      case Subsystem.shooter:
        ranCheckSub = _shooterRanCheckSub;
        statusSub = _shooterStatusSub;
        faultsSub = _shooterFaultsSub;
        checkRunningSub = _shooterCheckRunningSub;
        break;
      default:
        ranCheckSub = NT4Subscription(topic: 'topic');
        statusSub = NT4Subscription(topic: 'topic');
        faultsSub = NT4Subscription(topic: 'topic');
        checkRunningSub = NT4Subscription(topic: 'topic');
    }
    // Check if a controller for the subsystem already exists
    if (!_statusControllers.containsKey(subsystem)) {
      // Create a new broadcast controller if it doesn't exist
      var controller = StreamController<SubsystemStatus>.broadcast();
      _statusControllers[subsystem] = controller;

      // Here you can start your actual logic to add data to the stream
      // For example, this could be a periodic timer or response to some events
      Timer.periodic(const Duration(milliseconds: 500), (_) {
        // Attempt to safely cast the values. Adjust the types as necessary.
        bool ranCheck = false;
        String status = '';
        String faults = '';
        bool checkRunning = false;

        var ranCheckValue = ranCheckSub.currentValue;
        if (ranCheckValue is bool) {
          ranCheck = ranCheckValue;
        }

        var statusValue = statusSub.currentValue;
        if (statusValue is String) {
          status = statusValue;
        } else if (statusValue is List) {
          // If the value is a list, handle accordingly. This is just an example.
          status = statusValue.join(', ');
        }

        var faultsValue = faultsSub.currentValue;
        if (faultsValue is String) {
          faults = faultsValue;
        } else if (faultsValue is List) {
          // Handle the list case
          faults = faultsValue.join(', ');
        }

        var checkRunningValue = checkRunningSub.currentValue;
        if (checkRunningValue is bool) {
          checkRunning = checkRunningValue;
        }

        var subsystemStatus = SubsystemStatus(
          ranCheck: ranCheck,
          status: status,
          faults: faults,
          checkRunning: checkRunning,
        );
        controller.sink.add(subsystemStatus); // Add data to the stream
      });
    }

    // Return the stream associated with the controller
    return _statusControllers[subsystem]!.stream;
  }

  static Future<void> startAllSubsystemTests() async {}

  static Future<void> startSubsystemTest(Subsystem subsystem) async {
    NT4Topic checkRunningTopic;
    NT4Subscription checkRunningSub;
    switch (subsystem) {
      case Subsystem.arm:
        checkRunningTopic = _armCheckRunningTopic;
        checkRunningSub = _armCheckRunningSub;
        break;
      case Subsystem.climb:
        checkRunningTopic = _climbCheckRunningTopic;
        checkRunningSub = _climbCheckRunningSub;
        break;
      // case Subsystem.drivetrain:
      //   checkRunningTopic = _drivetrainCheckRunningTopic;
      //   checkRunningSub = _drivetrainCheckRunningSub;
      //   break;
      case Subsystem.intake:
        checkRunningTopic = _intakeCheckRunningTopic;
        checkRunningSub = _intakeCheckRunningSub;
        break;
      case Subsystem.indexer:
        checkRunningTopic = _indexerCheckRunningTopic;
        checkRunningSub = _indexerCheckRunningSub;
        break;
      case Subsystem.shooter:
        checkRunningTopic = _shooterCheckRunningTopic;
        checkRunningSub = _shooterCheckRunningSub;
        break;
      default:
        checkRunningTopic = _client.publishNewTopic(
            '/SmartDashboard/SubsystemStatus/ERROR/SubsystemCheck/running',
            NT4TypeStr.typeBool);
        checkRunningSub = _client.subscribePeriodic(
            '/SmartDashboard/SubsystemStatus/ERROR/SubsystemCheck/running');
    }
    _client.addSample(checkRunningTopic, true);

    await Future.delayed(const Duration(milliseconds: 200));
    checkRunningSub.stream().listen((data) {
      if (data is bool && !data) {
        ranCheckNotifiers[subsystem]!.value = true;
      }
    });
  }

  static Stream<double> getMotorTemp(int motorIndex) async* {
    await for (Object? value in _motorTempSubs[motorIndex].stream()) {
      if (value != null) {
        yield value as double;
      }
    }
  }

  static Stream<List<double>> getPlot(String graphType) async* {
    List<double> values = List.generate(608, (index) => 0.0);
    NT4Subscription stream;
    switch (graphType.toLowerCase()) {
      case 'riocan':
        stream = _rioCANUtilSub;
        break;
      case 'canivorecan':
        stream = _canivoreCANUtilSub;
        break;
      case 'batteryvoltage':
        stream = _batteryVoltageSub;
        break;
      default:
        stream = _rioCANUtilSub;
    }

    await for (Object? value in stream.stream()) {
      if (value != null) {
        values.removeAt(0);

        values.add(value as double);

        yield values;
      }
    }
  }
}

class SubsystemStatus {
  final bool ranCheck;
  final String status;
  final String faults;
  final bool checkRunning;

  const SubsystemStatus({
    required this.ranCheck,
    required this.status,
    required this.faults,
    required this.checkRunning,
  });
}

enum Subsystem {
  arm,
  climb,
  intake,
  indexer,
  shooter,
}
