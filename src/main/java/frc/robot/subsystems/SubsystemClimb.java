// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.climb.ManualCommandClimb;
import frc.robot.util.Xbox;

public class SubsystemClimb extends SubsystemBase {
  @SuppressWarnings("unused")
  private final Joystick operator;
  private final CANSparkMax engine;

  /** Creates a new SubsystemClimber. */
  public SubsystemClimb(Joystick operator) {
    this.operator = operator;
    this.engine = new CANSparkMax(Constants.CLIMB_ENGINE_ID, MotorType.kBrushless);

    this.configureMotors();
    super.setDefaultCommand(new ManualCommandClimb(this, operator));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Engine Amps", engine.getOutputCurrent());
    SmartDashboard.putNumber("Climb Engine Encoder Count", engine.getEncoder().getPosition());
  }

  private void configureMotors() {
    this.engine.setIdleMode(IdleMode.kBrake);
    this.engine.setSmartCurrentLimit(60);
  }

  public void manualClimb(Joystick operator) {
    double throttle = Xbox.RT(operator) - Xbox.LT(operator);
    throttle = throttle < -1D ? -1D : 1 < throttle ? 1D : throttle;
    engine.set(throttle * .8D);
  }

  public void zeroEncoder() {
    engine.getEncoder().setPosition(0D);
  }
}
