/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandSetTurretPositionManual extends CommandBase {
  private SubsystemTurret turret;
  private double yawPosition, pitchPosition, flywheelTarget;

  public CyborgCommandSetTurretPositionManual(SubsystemTurret turret, double yawTarget, double pitchTarget, double flywheelTarget) {
    this.turret = turret;
    this.yawPosition = yawTarget;
    this.pitchPosition = pitchTarget;
    this.flywheelTarget = flywheelTarget;
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
    if (yawPosition < -100)
      yawPosition = -100D;
    if (yawPosition > -5D)
      yawPosition = -5D;
    turret.updateTarget(yawPosition, pitchPosition);

    Preferences.setDouble("FW Velocity Target", flywheelTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setYawPercentOutput(0);
    turret.setPitchPercentOutput(0);
    SmartDashboard.putBoolean("Turret PID", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
