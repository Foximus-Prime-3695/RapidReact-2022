// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drive.ManualCommandDrive;
import frc.robot.util.Xbox;

public class SubsystemDrive extends SubsystemBase {

  private final CANSparkMax leftMaster, leftSlave;
  private final CANSparkMax rightMaster, rightSlave;

  private final SparkMaxPIDController leftMasterPIDController, leftSlavePIDController;
  private final SparkMaxPIDController rightMasterPIDController, rightSlavePIDController;

  private RelativeEncoder leftMasterEncoder, leftSlaveEncoder;
  private RelativeEncoder rightMasterEncoder, rightSlaveEncoder;

  private boolean idleMode = false;

  /** Creates a new ExampleSubsystem. */
  public SubsystemDrive() {

    setDefaultCommand(new ManualCommandDrive(this));
    leftMaster = new CANSparkMax(Constants.LEFT_MASTER_ID, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.LEFT_SLAVE_ID, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_ID, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.RIGHT_SLAVE_ID, MotorType.kBrushless);

    leftMasterPIDController = leftMaster.getPIDController();
    leftSlavePIDController = leftSlave.getPIDController();
    rightMasterPIDController = rightMaster.getPIDController();
    rightSlavePIDController = rightSlave.getPIDController();

    leftMasterEncoder = leftMaster.getEncoder();
    leftSlaveEncoder = leftSlave.getEncoder();
    rightMasterEncoder = rightMaster.getEncoder();
    rightSlaveEncoder = rightSlave.getEncoder();

    enableCoastMode();
    setInverts();
    setCurrentLimits();
    setRamps();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(Joystick controller) {
    double throttle = Xbox.RT(controller) - Xbox.LT(controller);
    double steering = -Xbox.LEFT_X(controller);

    steering *= Constants.STEERING_INHIBITOR;

    double driveRight = throttle + steering;
    double driveLeft = throttle - steering;

    driveRight = (driveRight < -1 ? -1 : (driveRight > 1 ? 1 : driveRight));
    driveLeft = (driveLeft < -1 ? -1 : (driveLeft > 1 ? 1 : driveLeft));

    driveRight *= Constants.DRIVE_INHIBITOR;
    driveLeft *= Constants.DRIVE_INHIBITOR;

    leftMaster.set(driveLeft);
    leftSlave.set(driveLeft);
    rightMaster.set(driveRight);
    rightSlave.set(driveRight);
  }

  public void setPIDConstants(double kP, double kI, double kD, double kF, double kIZ, double demand) {
    leftMasterPIDController.setP(kP);
    leftMasterPIDController.setI(kI);
    leftMasterPIDController.setD(kD);
    leftMasterPIDController.setFF(kF);
    leftMasterPIDController.setIZone(kIZ);
    leftMasterPIDController.setOutputRange(-demand, demand);
    leftSlavePIDController.setP(kP);
    leftSlavePIDController.setI(kI);
    leftSlavePIDController.setD(kD);
    leftSlavePIDController.setFF(kF);
    leftSlavePIDController.setIZone(kIZ);
    leftSlavePIDController.setOutputRange(-demand, demand);

    rightMasterPIDController.setP(kP);
    rightMasterPIDController.setI(kI);
    rightMasterPIDController.setD(kD);
    rightMasterPIDController.setFF(kF);
    rightMasterPIDController.setIZone(kIZ);
    rightMasterPIDController.setOutputRange(-demand, demand);
    rightSlavePIDController.setP(kP);
    rightSlavePIDController.setI(kI);
    rightSlavePIDController.setD(kD);
    rightSlavePIDController.setFF(kF);
    rightSlavePIDController.setIZone(kIZ);
    rightSlavePIDController.setOutputRange(-demand, demand);
  }

  public double[] getPositions() {
    return new double[] { leftMasterEncoder.getPosition(), leftSlaveEncoder.getPosition(),
        rightMasterEncoder.getPosition(), rightSlaveEncoder.getPosition() };
  }

  public double getLeftMasterPosition() {
    return leftMasterEncoder.getPosition();
  }

  public double getRightMasterPosition() {
    return rightMasterEncoder.getPosition();
  }

  public void setLeftPositionTarget(double leftMasterDestination, double leftSlaveDestination) {
    leftMasterPIDController.setReference(leftMasterDestination, ControlType.kPosition);
    leftSlavePIDController.setReference(leftSlaveDestination, ControlType.kPosition);
  }

  public void setRightPositionTarget(double rightMasterDestination, double rightSlaveDestination) {
    rightMasterPIDController.setReference(rightMasterDestination, ControlType.kPosition);
    rightSlavePIDController.setReference(rightSlaveDestination, ControlType.kPosition);
  }

  public void setLeftSpeed(double speed) {
    leftMaster.set(speed);
    leftSlave.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightMaster.set(speed);
    rightSlave.set(speed);
  }

  public void zeroAllMotors() {
    leftMaster.getEncoder().setPosition(0D);
    leftSlave.getEncoder().setPosition(0D);
    rightMaster.getEncoder().setPosition(0D);
    rightSlave.getEncoder().setPosition(0D);
  }

  public void enableCoastMode() {
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
  }

  public void enableBrakingMode() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);
  }

  public void setInverts() {
    leftMaster.setInverted(Constants.LEFT_MASTER_INVERT);
    leftSlave.setInverted(Constants.LEFT_SLAVE_INVERT);
    rightMaster.setInverted(Constants.RIGHT_MASTER_INVERT);
    rightSlave.setInverted(Constants.RIGHT_SLAVE_INVERT);
  }

  public void setCurrentLimits() {
    leftMaster.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    leftSlave.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    rightMaster.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
    rightSlave.setSmartCurrentLimit(Constants.CURRENT_LIMIT);
  }

  public void setRamps() {
    leftMaster.setOpenLoopRampRate(Constants.RAMP_RATE);
    leftSlave.setOpenLoopRampRate(Constants.RAMP_RATE);
    rightMaster.setOpenLoopRampRate(Constants.RAMP_RATE);
    rightSlave.setOpenLoopRampRate(Constants.RAMP_RATE);
  }

  public void toggleIdleMode() {
    if(idleMode)
      enableBrakingMode();
    else
      enableCoastMode();
    idleMode = !idleMode;
  }
}
