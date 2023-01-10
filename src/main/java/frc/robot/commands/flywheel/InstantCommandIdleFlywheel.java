// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SubsystemFlywheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCommandIdleFlywheel extends InstantCommand {
  private final SubsystemFlywheel flywheel;
  public InstantCommandIdleFlywheel(SubsystemFlywheel flywheel) {
    this.flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.toggleIdle();
  }
}
