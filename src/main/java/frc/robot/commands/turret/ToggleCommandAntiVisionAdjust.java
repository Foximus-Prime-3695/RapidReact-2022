// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class ToggleCommandAntiVisionAdjust extends CommandBase {
  private final SubsystemTurret turret;

  public ToggleCommandAntiVisionAdjust(SubsystemTurret turret) {
    this.turret = turret;
    //addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double yawkP = Util.getAndSetDouble("Yaw Position kP", 0.03D);
    double yawkI = Util.getAndSetDouble("Yaw Position kI", 0D);
    double yawkD = Util.getAndSetDouble("Yaw Position KD", 0D);
    double yawIZone = Util.getAndSetDouble("Yaw Position IZone", 75);
    double yawkF = Util.getAndSetDouble("Yaw Position KF", 0D);
    turret.updateYawPID(yawkP, yawkI, yawkD, yawIZone, yawkF);

    double pitchkP = Util.getAndSetDouble("Pitch Position kP", 0.355D);
    double pitchkI = Util.getAndSetDouble("Pitch Position kI", 0.0002D);
    double pitchIZone = Util.getAndSetDouble("Pitch Position IZone", 0.01D);
    double pitchkD = Util.getAndSetDouble("Pitch Position kD", 0D);
    double pitchkF = Util.getAndSetDouble("Pitch Position kF", 0D);
    turret.updatePitchPID(pitchkP, pitchkI, pitchkD, pitchIZone, pitchkF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchTarget = Util.getAndSetDouble("Anti Vision Pitch Rotations", 15D);
    turret.updateYawTarget(-70D);
    turret.updatePitchTarget(pitchTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setPitchPercentOutput(0);
    turret.setYawPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
