// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SubsystemFlywheel;

public class InstantCommandToggleFlywheel extends InstantCommand {
  private final SubsystemFlywheel flywheel;

  public InstantCommandToggleFlywheel(SubsystemFlywheel flywheel) {
    this.flywheel = flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.toggleDisable();
  }
}
