// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.util.Xbox;

public class ManualCommandIntake extends CommandBase {

    private final SubsystemIntake intake;

    /** Creates a new ManualCommandTurret. */
    public ManualCommandIntake(SubsystemIntake intake) {

        this.intake = intake;
        super.addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double intakeAdjust = Xbox.RIGHT_Y(RobotContainer.DRIVER);
        intakeAdjust = intakeAdjust > 1D ? 1D : intakeAdjust < -1D ? -1D : intakeAdjust;
        intakeAdjust *= -1D;
        if (intakeAdjust < 0D) {
            intakeAdjust *= 0.5D;
        }
        intake.setAdjustDemand(intakeAdjust);
        double primaryIntake = 0D, secondaryIntake = 0D, tertiaryIntake = 0D;
        if(RobotContainer.OPERATOR.getRawButton(Xbox.A)) {
            secondaryIntake = 1D;
        }
        if (RobotContainer.OPERATOR.getRawButton(Xbox.X)) {
            primaryIntake = 1D;

        }
        if (RobotContainer.OPERATOR.getRawButton(Xbox.Y)) {
            primaryIntake = -1D;
            secondaryIntake = -1D;
            tertiaryIntake = -1D;
        }
        this.intake.setMotorOutput(primaryIntake, secondaryIntake, tertiaryIntake);
        // this.intake.adjustTheAdjust(intakeAdjust);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
