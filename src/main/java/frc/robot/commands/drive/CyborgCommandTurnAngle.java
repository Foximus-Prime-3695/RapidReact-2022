// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemTurret;

public class CyborgCommandTurnAngle extends CommandBase {
  private final SubsystemDrive drivetrain;
  private final SubsystemTurret turret;
  private final double targetYawRotations;

  /** Creates a new CyborgCommandTurnAngle. */
  public CyborgCommandTurnAngle(SubsystemDrive drivetrain, SubsystemTurret turret, double targetYawRotations) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.targetYawRotations = targetYawRotations;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turret.getYawPosition() < targetYawRotations) {
      drivetrain.setLeftSpeed(Constants.AUTONOMOUS_ROTATION_INHIBITOR);
      drivetrain.setRightSpeed(-Constants.AUTONOMOUS_ROTATION_INHIBITOR);
    } else {
      drivetrain.setLeftSpeed(-Constants.AUTONOMOUS_ROTATION_INHIBITOR);
      drivetrain.setRightSpeed(Constants.AUTONOMOUS_ROTATION_INHIBITOR);
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
    return Math.abs(turret.getYawPosition() - targetYawRotations) < Constants.DRIVETRAIN_ALLOWABLE_ERROR;
  }
}
