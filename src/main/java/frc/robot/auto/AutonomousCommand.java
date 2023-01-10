package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutonomousCommand {
    public Command getAutonomousCommand();
    public boolean requiresFlywheel();
}
