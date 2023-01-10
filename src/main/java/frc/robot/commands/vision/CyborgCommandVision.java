// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemReceiver;

public class CyborgCommandVision extends CommandBase {

  private SubsystemReceiver receiver;
  /** Creates a new CyborgCommandVision. */
  public CyborgCommandVision(SubsystemReceiver receiver) {
    this.receiver = receiver;
    addRequirements(receiver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    receiver.adjustTurretFromData();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
