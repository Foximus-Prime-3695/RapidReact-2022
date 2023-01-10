// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.intake.ManualCommandIntake;

public class SubsystemIntake extends SubsystemBase {

  TalonSRX primaryIntake, secondaryIntake, tertiaryIntake;
  TalonSRX intakeAdjustment;

  SupplyCurrentLimitConfiguration adjustCurrentLimit;

  


  /** Creates a new SubsystemBallTransport. */
  public SubsystemIntake() {
    this.intakeAdjustment = new TalonSRX(Constants.INTAKE_ADJUSTMENT_ID);
    
    configMotors();

    this.primaryIntake = new TalonSRX(Constants.PRIMARY_INTAKE_ID);
    this.secondaryIntake = new TalonSRX(Constants.SECONDARY_INTAKE_ID);
    setDefaultCommand(new ManualCommandIntake(this));
    this.primaryIntake.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ball Feed Adjustment Amps", intakeAdjustment.getSupplyCurrent());
    SmartDashboard.putNumber("Secondary Intake Position", getSecondaryIntakePosition());
  }

  public void setAdjustDemand(double demand) {
    this.intakeAdjustment.set(TalonSRXControlMode.PercentOutput, demand);
  }

  public void enableAdjustCurrentLimit() {
    this.intakeAdjustment.enableCurrentLimit(true);
  }

  public void disableAdjustCurrentLimit() {
    this.intakeAdjustment.enableCurrentLimit(false);
  }

  public void setPrimaryIntake(double primaryDemand) {
    this.primaryIntake.set(TalonSRXControlMode.PercentOutput, primaryDemand);
  }

  public void setSecondaryIntake(double secondaryDemand) {
    this.secondaryIntake.set(TalonSRXControlMode.PercentOutput, secondaryDemand);
  }

  public void setMotorOutput(double primaryDemand, double secondaryDemand, double tertiaryDemand) {
    this.primaryIntake.set(TalonSRXControlMode.PercentOutput, primaryDemand);
    this.secondaryIntake.set(TalonSRXControlMode.PercentOutput, secondaryDemand);
    //this.tertiaryIntake.set(TalonSRXControlMode.PercentOutput, tertiaryDemand);
  }

  public void configMotors() {
    this.intakeAdjustment.enableCurrentLimit(false);
  }

  public int getSecondaryIntakePosition() {
    return this.secondaryIntake.getSensorCollection().getQuadraturePosition();
  }

  public void zeroSecondaryIntakeEncoder() {
    this.secondaryIntake.getSensorCollection().setQuadraturePosition(0, 10);
  }
}
