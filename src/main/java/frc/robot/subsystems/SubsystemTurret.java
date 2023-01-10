/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.turret.ManualCommandTurret;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

/**
 * Turret pitch and yaw
 */
public class SubsystemTurret extends SubsystemBase {

  private CANSparkMax yawMotor, pitchMotor;

  public RelativeEncoder yawEncoder, pitchEncoder;

  private SparkMaxLimitSwitch yawForwardLimit, yawReverseLimit, pitchForwardLimit, pitchReverseLimit;

  private SparkMaxPIDController yawPIDController, pitchPIDController;

  private double yaw_kP, yaw_kI, yaw_kD, yaw_kIZ, yaw_kFF;

  private double pitch_kP, pitch_kI, pitch_kD, pitch_kIZ, pitch_kFF;

  /**
   * Creates a new SubsystemTurret.
   */
  public SubsystemTurret() {
    yawMotor = new CANSparkMax(Constants.YAW_MOTOR_ID, MotorType.kBrushed);
    yawEncoder = yawMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    yawForwardLimit = yawMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    yawReverseLimit = yawMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    yawPIDController = yawMotor.getPIDController();
    yawMotor.setInverted(true);
    yawPIDController.setSmartMotionAllowedClosedLoopError(Constants.TURRET_YAW_ALLOWABLE_ERROR, 0);

    pitchMotor = new CANSparkMax(Constants.PITCH_MOTOR_ID, MotorType.kBrushed);
    pitchMotor.setInverted(false);
    pitchMotor.setIdleMode(IdleMode.kBrake);
    pitchMotor.setSmartCurrentLimit(40);
    pitchEncoder = pitchMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    pitchForwardLimit = pitchMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    pitchReverseLimit = pitchMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    pitchPIDController = pitchMotor.getPIDController();

    setDefaultCommand(new ManualCommandTurret(this));
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Yaw Position", yawEncoder.getPosition());
    SmartDashboard.putNumber("Pitch Position", pitchEncoder.getPosition());

    SmartDashboard.putBoolean("Yaw Forward Limit", yawForwardLimit.isPressed());
    SmartDashboard.putBoolean("Yaw Backward Limit", yawReverseLimit.isPressed());

    SmartDashboard.putBoolean("Pitch Downward Limit", pitchForwardLimit.isPressed());
    SmartDashboard.putBoolean("Pitch Upward Limit", pitchReverseLimit.isPressed());

    SmartDashboard.putNumber("Yaw Bus Voltage", yawMotor.getBusVoltage());
    SmartDashboard.putNumber("Pitch Bus Voltage", pitchMotor.getBusVoltage());

    SmartDashboard.putNumber("Yaw Amps", yawMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pitch Amps", pitchMotor.getOutputCurrent());

    //if (pitchReverseLimit.isPressed())
      //pitchEncoder.setPosition(0D);
  }

  public void controlTurret(Joystick operator) {
    double yawSpeed = Xbox.LEFT_X(operator);
    yawSpeed *= -1D;
    yawSpeed *= Util.getAndSetDouble("Yaw Speed Inhibitor", 0.5D);
    yawSpeed = yawSpeed < -1D ? -1D : 1D < yawSpeed ? 1D : yawSpeed;
    yawMotor.set(yawSpeed);

    double pitchSpeed = Xbox.RIGHT_Y(operator);
    pitchSpeed = pitchSpeed < -1D ? -1D : 1D < pitchSpeed ? 1D : pitchSpeed;
    pitchSpeed *= -1D;
    pitchMotor.set(pitchSpeed * 0.25D);
  }

  public void updateYawPID(double p, double i, double d, double iz, double ff) {
    if (p != yaw_kP)
      yawPIDController.setP(yaw_kP = p);
    if (i != yaw_kI)
      yawPIDController.setI(yaw_kI = i);
    if (d != yaw_kD)
      yawPIDController.setD(yaw_kD = d);
    if (iz != yaw_kIZ)
      yawPIDController.setIZone(yaw_kIZ = iz);
    if (ff != yaw_kFF)
      yawPIDController.setFF(yaw_kFF = ff);

  }

  public void updatePitchPID(double p, double i, double d, double iz, double ff) {
    if (p != pitch_kP)
      pitchPIDController.setP(pitch_kP = p);
    if (i != pitch_kI)
      pitchPIDController.setI(pitch_kI = i);
    if (d != pitch_kD)
      pitchPIDController.setD(pitch_kD = d);
    if (iz != pitch_kIZ)
      pitchPIDController.setIZone(pitch_kIZ = iz);
    if (ff != pitch_kFF)
      pitchPIDController.setFF(pitch_kFF = ff);
    pitchPIDController.setSmartMotionAllowedClosedLoopError(0.1D, 0);
  }

  public void updateTarget(double yaw, double pitch) {
    yawPIDController.setReference(yaw, ControlType.kPosition);
      pitchPIDController.setReference(pitch, ControlType.kPosition);
  }

  public void updateYawTarget(double targetPosition) {
    yawPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  public void updatePitchTarget(double targetPosition) {
    pitchPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  public void setPitchPercentOutput(double speed) {
    pitchMotor.set(speed);
  }

  public void setYawPercentOutput(double speed) {
    yawMotor.set(speed);
  }

  public void setPitchEncoderPosition(double position) {
    pitchEncoder.setPosition(position);
  }

  public void setYawEncoderPosition(double position) {
    yawEncoder.setPosition(position);
  }

  public void setPitchClosedLoopError(double error) {
    pitchPIDController.setSmartMotionAllowedClosedLoopError(error, 0);
  }

  public void setYawClosedLoopError(double error) {
    yawPIDController.setSmartMotionAllowedClosedLoopError(error, 0);
  }

  public double getPitchClosedLoopError() {
    return pitchPIDController.getSmartMotionAllowedClosedLoopError(0);
  }

  public double getYawClosedLoopError() {
    return yawPIDController.getSmartMotionAllowedClosedLoopError(0);
  }

  public double getPitchPosition() {
    return pitchEncoder.getPosition();
  }

  public double getYawPosition() {
    return pitchEncoder.getPosition();
  }

  public boolean getYawLeftLimit() {
    return yawForwardLimit.isPressed();
  }

  public boolean getYawRightLimit() {
    return yawReverseLimit.isPressed();
  }

  public boolean getPitchLowerLimit() {
    return pitchForwardLimit.isPressed();
  }

  public boolean getPitchUpperLimit() {
    return pitchReverseLimit.isPressed();
  }


}
