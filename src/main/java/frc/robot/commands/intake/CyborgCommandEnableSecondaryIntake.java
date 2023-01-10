// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemIntake;

public class CyborgCommandEnableSecondaryIntake extends CommandBase {
  private SubsystemIntake intake;
  private double speed;

  /** Creates a new CyborgCommandToggleIntake. */
  public CyborgCommandEnableSecondaryIntake(SubsystemIntake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSecondaryIntake(speed);// .61D
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSecondaryIntake(0D);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
