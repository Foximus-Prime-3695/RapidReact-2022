// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemClimb;

public class ManualCommandClimb extends CommandBase {
  private final SubsystemClimb climb;
  private final Joystick operator;

  public ManualCommandClimb(SubsystemClimb climb, Joystick operator) {
    this.climb = climb;
    this.operator = operator;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climb.manualClimb(operator);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
