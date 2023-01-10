// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualCommandDrive extends CommandBase {

  private final SubsystemDrive drive;

  public ManualCommandDrive(SubsystemDrive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    this.drive.drive(RobotContainer.DRIVER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
