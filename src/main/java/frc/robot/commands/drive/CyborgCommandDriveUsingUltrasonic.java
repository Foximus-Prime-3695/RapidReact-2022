// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemUltrasonic;

public class CyborgCommandDriveUsingUltrasonic extends CommandBase {
  private final SubsystemDrive drivetrain;
  private final SubsystemUltrasonic ultrasonic;
  private final double targetDistance;

  /** Creates a new CyborgCommandDriveDistance. */
  public CyborgCommandDriveUsingUltrasonic(SubsystemDrive drivetrain, SubsystemUltrasonic ultrasonic,double distance) {
    this.drivetrain = drivetrain;
    this.ultrasonic = ultrasonic;
    this.targetDistance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(ultrasonic.getDistance() > targetDistance) {
        drivetrain.setLeftSpeed(Constants.AUTONOMOUS_DRIVE_INHIBITOR);
        drivetrain.setRightSpeed(Constants.AUTONOMOUS_DRIVE_INHIBITOR);
      } else {
        drivetrain.setLeftSpeed(-Constants.AUTONOMOUS_DRIVE_INHIBITOR);
        drivetrain.setRightSpeed(-Constants.AUTONOMOUS_DRIVE_INHIBITOR);
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLeftSpeed(0D);
    drivetrain.setRightSpeed(0D);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(ultrasonic.getDistance() - targetDistance) < Constants.DRIVETRAIN_ALLOWABLE_ERROR;
  }
}
