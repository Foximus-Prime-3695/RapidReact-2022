package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.util.Util;

public class ManualCommandFlywheel extends CommandBase {

  private final SubsystemFlywheel flywheel;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualCommandFlywheel(SubsystemFlywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double p = Util.getAndSetDouble("FW Velocity kP", 0.0014);
    double i = Util.getAndSetDouble("FW Velocity kI", 0.000005);
    double d = Util.getAndSetDouble("FW Velocity kD", 0);
    double f = Util.getAndSetDouble("FW Velocity kF", 0.000185);

    double upperOutLimit = Util.getAndSetDouble("FW Velocity Max Out", 1);
    double lowerOutLimit = Util.getAndSetDouble("FW Velocity Min Out", -1);

    double izone = Util.getAndSetDouble("FW Velocity IZone", 100);

    flywheel.setPIDF(p, i, d, f, izone, lowerOutLimit, upperOutLimit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Util.getAndSetDouble("FW Velocity Target", 3000) / 1.6071D;
    flywheel.setVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setVelocity(0D);
    flywheel.setPercentOutput(0D);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
