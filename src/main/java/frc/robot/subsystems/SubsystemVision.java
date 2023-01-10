// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemVision extends SubsystemBase {
  private final NetworkTable limelight;
  private double[] verticleOffset = new double[3];
  private int voOffset = 0;
  /** Creates a new SubsystemVision. */
  public SubsystemVision() {
    this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    verticleOffset[voOffset] = getVerticalOffset();
    voOffset += 1;
    voOffset %= verticleOffset.length;

    SmartDashboard.putNumber("Distance To Taget Limelight", getDistance());
    SmartDashboard.putBoolean("Target Spotted", targetSpotted());
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
    return a + b + c + d + e;
  }

  public double getPitchTarget() {
    double distance = getDistance();
    double a = 0.0431D * Math.pow(distance, 3D);
    double b = -1.4374D * Math.pow(distance, 2D);
    double c = 26.435D * distance;
    double d = 1.7124D;
    return a + b + c + d;
  }

  public double getFlywheelVelocity() {
    double distance = getDistance();
    double a = -0.4235D * Math.pow(distance, 3D);
    double b = 12.502D * Math.pow(distance, 2D);
    double c = -6.5726D * distance;
    double d = 3024.6D;
    return a + b + c + d;
  }
}
