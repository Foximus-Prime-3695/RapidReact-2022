// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemTurret;

public class ManualCommandTurret extends CommandBase {

  private final SubsystemTurret turret;

  public ManualCommandTurret(SubsystemTurret turret) {
    this.turret = turret;
    super.addRequirements(turret);
  }

  @Override
  public void execute() {
    this.turret.controlTurret(RobotContainer.OPERATOR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
