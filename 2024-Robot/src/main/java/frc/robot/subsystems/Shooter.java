// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  PIDController leftFlywheelsPID;
  PIDController rightFlywheelsPID;

  public Shooter(ShooterIO io) {}

  @Override
  public void periodic() {
  }
}
