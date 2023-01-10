// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;

public class CyborgCommadSetPitch extends CommandBase {

  private final SubsystemTurret turret;

  /** Creates a new CyborgCommadSetPitch. */
  public CyborgCommadSetPitch(SubsystemTurret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    double pitchPosition = Util.getAndSetDouble("Test PID Pitch Target", 0D);
    turret.updatePitchTarget(pitchPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turret.setPitchPercentOutput(0D);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
