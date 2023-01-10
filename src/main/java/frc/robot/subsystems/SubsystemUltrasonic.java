// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemUltrasonic extends SubsystemBase {

  private Ultrasonic ultrasonic;

  public SubsystemUltrasonic() {
    ultrasonic = new Ultrasonic(0, 1);
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ultrasonic Distance", ultrasonic.getRangeInches());
  }

  public double getDistance() {
    return ultrasonic.getRangeInches();
  }
}
