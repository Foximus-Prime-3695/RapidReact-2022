// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandSetYawPosition extends CommandBase {
  private final SubsystemTurret turret;
  private final double targetPosition;

  public CyborgCommandSetYawPosition(SubsystemTurret turret, double targetPosition) {
    if (turret == null)
      throw new NullPointerException();
    if (targetPosition < -102 || 0 < targetPosition)
      throw new IllegalArgumentException();

    this.turret = turret;
    this.targetPosition = targetPosition;

    addRequirements(turret);
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
    turret.updateYawTarget(targetPosition);
    SmartDashboard.putBoolean("Yaw PID Activity", true);
  }

  @Override
  public void end(boolean interrupted) {
    this.turret.setYawPercentOutput(0);
    SmartDashboard.putBoolean("Yaw PID Activity", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.yawEncoder.getPosition() - targetPosition) < 0.5D;
  }
}
