/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommandSetTurretPosition extends CommandBase {
  private SubsystemTurret turret;
  private double yawPosition, pitchPosition;
  NetworkTable limelight;

  public CyborgCommandSetTurretPosition(SubsystemTurret turret) {
    this.turret = turret;
    this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
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
    yawPosition = RobotContainer.SUB_TURRET.yawEncoder.getPosition()
        + getYawOffset();
    if (yawPosition < -100)
      yawPosition = -100D;
    if (yawPosition > -5D)
      yawPosition = -5D;
    pitchPosition = getPitchTarget();
    if(pitchPosition > 325D)
      pitchPosition = 325D;
    if(pitchPosition < 5D)
      pitchPosition = 5D;
    turret.updateTarget(yawPosition, pitchPosition);

    double yawError = Math.abs(Math.abs(turret.getYawPosition()) - yawPosition);
    double pitchError = Math.abs(turret.getPitchPosition() - pitchPosition);

    if (yawError <= turret.getYawClosedLoopError()) {
      turret.setYawPercentOutput(0);
    }
    if (pitchError <= turret.getPitchClosedLoopError()) {
      turret.setPitchPercentOutput(0);
    }
    Preferences.setDouble("FW Velocity Target", getFlywheelVelocity());
    SmartDashboard.putBoolean("Turret PID", true);
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

  public boolean targetSpotted() {
    return limelight.getEntry("ta").getDouble(0D) == 1D;
  }

  public double getHorizontalOffset() {
    return limelight.getEntry("tx").getDouble(0D);
  }

  public double getVerticalOffset() {
    return limelight.getEntry("ty").getDouble(0D);
  }

  public double getYawOffset() {
    return -getHorizontalOffset() * 1.2912D - 5D;
  }

  public double getDistance() {
    double pitchAngle = getVerticalOffset();
    double a = 1.0006e-5D * Math.pow(pitchAngle, 4D);
    double b = -0.0002D * Math.pow(pitchAngle, 3D);
    double c = 0.0041D * Math.pow(pitchAngle, 2D);
    double d = -0.2449D * pitchAngle;
    double e = 5.7734D;
    return a + b + c + d + e + 3D;
  }

  public double getPitchTarget() {
    double distance = getDistance();
    double a = 0.0431D * Math.pow(distance, 3D);
    double b = -1.4374D * Math.pow(distance, 2D);
    double c = 26.435D * distance;
    double d = 1.7124D;
    return a + b + c + d + 15D;
  }

  public double getFlywheelVelocity() {
    double distance = getDistance();
    double a = -0.4235D * Math.pow(distance, 3D);
    double b = 12.502D * Math.pow(distance, 2D);
    double c = -6.5726D * distance;
    double d = 3024.6D;
    return a + b + c + d - 50D;
  }
}
