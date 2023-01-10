// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemIntake;

public class CyborgCommandToggleAdjust extends CommandBase {
  private SubsystemIntake ballIntake;
  private long startTime;

  public CyborgCommandToggleAdjust(SubsystemIntake ballIntake) {
    this.ballIntake = ballIntake;
    addRequirements(ballIntake);
  }

  @Override
  public void initialize() {
    this.startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    this.ballIntake.setAdjustDemand(-0.33D);
  }

  @Override
  public void end(boolean interrupted) {
    this.ballIntake.setAdjustDemand(0D);
    this.ballIntake.disableAdjustCurrentLimit();
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime > 1250L;
  }
}
